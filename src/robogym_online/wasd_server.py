# SPDX-License-Identifier: Apache-2.0
"""Serve the ARDY reference stream over a websocket, so a browser can be the tracker.

The browser app runs the physics and the policy itself, in WASM -- what it cannot run is the motion
generator, which is a GPU checkpoint. So the split is by what needs the hardware: this process owns
the generator, the browser owns MuJoCo, the force policy, the operator's keyboard and the picture.

The client sends its command, and its robot's pose. The pose is not optional -- a kinematic
reference is not something a physical gait matches exactly, and a generator that cannot see the
robot lets the two separate for as long as a key is held. Where the generator continues from a pose
window (MotionBricks) the robot's own recent poses go straight into it; where it plans a path
(ARDY), the tracking error steers the plan instead.

**Wire format.** A JSON ``hello`` on connect announces the control rate, the body order and the
exact field layout; frame data is binary because it is not small -- 487 floats per frame, ~97 KB/s
at 50 Hz, which as JSON would be several times that. Each data message is::

    int32 start_index | int32 count | float32 payload, fields in the announced order

Client messages are JSON: ``{"type": "command", "forward": .., "lateral": .., "turn": ..}``,
``{"type": "request", "from": i, "count": n}``, and one of the feedback messages --
``{"type": "style", "name": ".."}`` (one of the styles the hello announced),
``{"type": "context", "qpos": [..], "frame": i}`` (the robot's pose, 7 + ndof in the contract's
order, and the reference frame it is tracking) or
``{"type": "lag", "dx": .., "dy": ..}``.

:class:`RemoteReferenceStream` is the reference client -- same ``set_command`` / ``frames``
interface as the local stream, so ``live_wasd --remote`` exercises this whole path with no browser
involved, and the TypeScript client has a working implementation to mirror.

    TEXT_ENCODER=null python -m robogym_online.wasd_server
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import time
import uuid
from urllib.parse import parse_qs, urlsplit
import struct
from pathlib import Path

import numpy as np

from .scene import DEFAULT_MJCF, DEFAULT_ONNX_DIR, load_contract

DEFAULT_PORT = 8765


def _yaw_of_quat(quat) -> float:
    """Yaw of a wxyz quaternion in degrees, or NaN if the client sent none."""
    if not quat:
        return float("nan")
    w, x, y, z = (float(v) for v in quat[:4])
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _yaw_deg(qpos) -> float:
    """Heading of a reported pose, in degrees -- for the debug line only."""
    w, x, y, z = (float(v) for v in qpos[3:7])
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


_debug: dict = {}
HEADER = struct.Struct("<ii")

# Field order on the wire, with the per-frame shape of each. Fixed here and announced in the hello
# so a client never has to guess; adding a field means bumping both sides.
FIELDS: tuple[tuple[str, tuple[int, ...]], ...] = (
    ("joint_pos", (-1,)),
    ("joint_vel", (-1,)),
    ("body_pos_w", (-1, 3)),
    ("body_quat_w", (-1, 4)),
    ("body_lin_vel_w", (-1, 3)),
    ("body_ang_vel_w", (-1, 3)),
)


def pack_frames(block: dict[str, np.ndarray], start: int) -> bytes:
    """``start``, the frame count, and the fields as float32, in :data:`FIELDS` order."""
    count = block[FIELDS[0][0]].shape[0]
    parts = [HEADER.pack(start, count)]
    parts += [np.ascontiguousarray(block[name], dtype=np.float32).tobytes() for name, _ in FIELDS]
    return b"".join(parts)


def unpack_frames(payload: bytes, n_dofs: int, n_bodies: int) -> tuple[int, dict[str, np.ndarray]]:
    """Inverse of :func:`pack_frames`."""
    start, count = HEADER.unpack_from(payload, 0)
    offset = HEADER.size
    out: dict[str, np.ndarray] = {}
    for name, shape in FIELDS:
        per_frame = n_dofs if shape == (-1,) else n_bodies * shape[1]
        size = count * per_frame * 4
        flat = np.frombuffer(payload, dtype=np.float32, count=count * per_frame, offset=offset)
        out[name] = flat.reshape((count, per_frame) if shape == (-1,) else (count, n_bodies, shape[1]))
        offset += size
    return start, out


class RemoteReferenceStream:
    """Client side of the protocol, with the same interface as the local generator.

    Frames are cached as they arrive and requested a little ahead of what is being consumed, because
    a request that has to travel to the server before the next control step would stall the loop.
    Blocking on a miss is still the fallback -- correctness over smoothness.

    ``lead`` is a latency budget, not just a safety margin: everything already fetched was generated
    under an older command, so the keyboard cannot bite until the robot has consumed it. It has to
    clear the policy's 0.4 s of lookahead and the round trip, and every frame beyond that is added
    steering lag -- fetching two seconds ahead makes the robot visibly late to start walking.
    """

    def __init__(self, url: str, lead: int = 40, block: int = 25) -> None:
        from websockets.sync.client import connect

        self._socket = connect(url)
        hello = json.loads(self._socket.recv())
        if hello.get("type") != "hello":
            raise RuntimeError(f"expected a hello, got {hello.get('type')!r}")
        self.control_dt = float(hello["control_dt"])
        self.body_names = list(hello["body_names"])
        self.styles = tuple(hello.get("styles", ()))
        self._n_dofs = int(hello["n_dofs"])
        self._lead = lead
        self._block = block
        self._frames: dict[str, np.ndarray] | None = None
        self._known = 0  # frames cached, counted from index 0

    def set_context_qpos(self, qpos: np.ndarray, frame: int | None = None) -> None:
        self._socket.send(
            json.dumps(
                {
                    "type": "context",
                    "qpos": [float(v) for v in np.asarray(qpos).reshape(-1)],
                    "frame": frame,
                }
            )
        )

    def set_lag(self, dx: float, dy: float) -> None:
        self._socket.send(json.dumps({"type": "lag", "dx": float(dx), "dy": float(dy)}))

    def set_style(self, name: str) -> None:
        self._socket.send(json.dumps({"type": "style", "name": name}))

    def set_command(self, forward: float, lateral: float, turn_deg: float) -> None:
        self._socket.send(
            json.dumps({"type": "command", "forward": forward, "lateral": lateral, "turn": turn_deg})
        )

    def _fetch(self, start: int, count: int) -> None:
        self._socket.send(json.dumps({"type": "request", "from": start, "count": count}))
        got_start, block = unpack_frames(self._socket.recv(), self._n_dofs, len(self.body_names))
        if got_start != start:
            raise RuntimeError(f"asked for frame {start}, server sent {got_start}")
        if self._frames is None:
            self._frames = {k: v.copy() for k, v in block.items()}
        else:
            # Blocks arrive in order and abut, so appending is enough; a gap would mean a request
            # was lost, which the index check above turns into an error rather than a silent hole.
            self._frames = {k: np.concatenate([self._frames[k], v], axis=0) for k, v in block.items()}
        self._known = self._frames[FIELDS[0][0]].shape[0]

    def frames(self, indices) -> dict[str, np.ndarray]:
        idx = np.atleast_1d(np.asarray(indices, dtype=np.int64))
        want = int(idx.max()) + self._lead
        while self._known <= want:
            self._fetch(self._known, self._block)
        return {key: value[idx] for key, value in self._frames.items()}

    def close(self) -> None:
        self._socket.close()


def _build_stream(
    contract: dict,
    mjcf: Path,
    seed: int | None,
    generator: str,
    camera_req: str | None = None,
    camera_rep: str | None = None,
):
    """The generator behind this connection. One per client: each has its own command and robot."""
    if generator == "motionbricks":
        from .motionbricks_stream import MotionBricksStream
        from .scene import DEFAULT_MOTIONBRICKS_ROOT

        return MotionBricksStream(contract, mjcf, DEFAULT_MOTIONBRICKS_ROOT, seed=seed)
    if generator == "camera_motionbricks":
        from .camera_motionbricks_stream import (
            DEFAULT_CAMERA_REP_ADDR,
            DEFAULT_CAMERA_REQ_ADDR,
            CameraMotionBricksStream,
        )
        from .scene import DEFAULT_MOTIONBRICKS_ROOT

        return CameraMotionBricksStream(
            contract,
            mjcf,
            DEFAULT_MOTIONBRICKS_ROOT,
            req_addr=camera_req or DEFAULT_CAMERA_REQ_ADDR,
            rep_addr=camera_rep or DEFAULT_CAMERA_REP_ADDR,
            seed=seed,
        )
    if generator == "onnx":
        from .motionbricks_onnx import MotionBricksOnnxStream
        from .scene import DEFAULT_PLANNER_ROOT

        return MotionBricksOnnxStream(contract, mjcf, DEFAULT_PLANNER_ROOT, seed=seed)
    if generator == "ardy":
        from .wasd_stream import ReferenceStream

        return ReferenceStream(contract, mjcf, seed=seed)
    raise ValueError(f"unknown generator {generator!r}")


# Use slow_walk at its natural pace. At 0.5 m/s the normal walk is compressed
# without crossing SLOW_WALK_ENTER; both v6 and v12 fell in walk/strafe/stop
# rollouts before the feedback fix. Prefer 0.3 for force work and reversals.
WALK_SPEED_M_S = float(os.environ.get("WASD_WALK_SPEED", "0.3"))
# Explicit gaits validated on the full 6/8/10-degree ramp. These are planner
# target speeds, not promises about measured robot speed. Keep gait identity
# rather than routing these styles through the normal walk's speed thresholds.
STYLE_SPEEDS_M_S = {
    "slow_walk": float(os.environ.get("WASD_SLOW_WALK_SPEED", "0.5")),
    "stealth": float(os.environ.get("WASD_STEALTH_SPEED", "0.8")),
    "object_carrying": float(os.environ.get("WASD_OBJECT_CARRYING_SPEED", "0.8")),
    "careful": float(os.environ.get("WASD_CAREFUL_SPEED", "0.8")),
}


def _apply_walk_speed(stream, style: str) -> None:
    if hasattr(stream, "set_target_speed"):
        speed = STYLE_SPEEDS_M_S.get(style, WALK_SPEED_M_S if style == "walk" else None)
        limit = getattr(stream, "_browser_speed_limit", None)
        if limit is not None:
            speed = limit if speed is None else min(speed, limit)
        stream.set_target_speed(speed)


async def _serve_client(websocket, contract: dict, stream, in_use: dict) -> None:
    """Keep independent ONNX browser sessions; reconnects resume their existing reference."""
    session = None
    fresh = True
    if hasattr(stream, "fork_session"):
        now = time.monotonic()
        sessions = in_use.setdefault("sessions", {})
        for key, entry in list(sessions.items()):
            if entry["client"] is None and now - entry["seen"] > 300:
                del sessions[key]
        request = getattr(websocket, "request", None)
        path = getattr(request, "path", getattr(websocket, "path", "/"))
        token = parse_qs(urlsplit(path).query).get("session", [None])[0]
        token = token if token and len(token) <= 128 else uuid.uuid4().hex
        session = sessions.get(token)
        if session is None:
            if len(sessions) >= 32:
                idle = [(entry["seen"], key) for key, entry in sessions.items() if entry["client"] is None]
                if idle:
                    del sessions[min(idle)[1]]
                else:
                    await websocket.close(code=1013, reason="Generator busy; retry shortly")
                    return
            session = sessions[token] = {"stream": stream.fork_session(), "client": None, "seen": now, "lock": asyncio.Lock()}
        else:
            fresh = False
        previous = session["client"]
        if previous is not None:
            await previous.close()
        session["client"] = websocket
        stream = session["stream"]
    else:
        previous = in_use.get("client")
        if previous is not None:
            await previous.close()
        in_use["client"] = websocket
    if fresh:
        stream.reset()
    # The style is generator state, so a new session would otherwise inherit whatever the previous
    # one was left on -- a run, say -- while the page shows its first entry as selected. Start
    # every session on the first offered style; the client re-selects from there.
    styles = tuple(getattr(stream, "styles", ()))
    if fresh and styles and hasattr(stream, "set_style"):
        stream.set_style(styles[0])
        _apply_walk_speed(stream, styles[0])
    try:
        await websocket.send(
            json.dumps(
                {
                    "type": "hello",
                    "control_dt": stream.control_dt,
                    "n_dofs": len(contract["joint_names"]),
                    "body_names": list(contract["body_names"]),
                    "fields": [[name, list(shape)] for name, shape in FIELDS],
                    # The locomotion styles this generator offers, in selection order, so the client
                    # can present them without knowing what is behind the socket.
                    "styles": list(getattr(stream, "styles", ())),
                    "equipment": ["none", "dumbbells", "kettlebell", "barbell", "exertion"] if hasattr(stream, "set_equipment") else [],
                }
            )
        )
        print(f"client connected: {websocket.remote_address}")
        if session is not None:
            async with session["lock"]:
                await _pump(websocket, stream)
        else:
            await _pump(websocket, stream)
    finally:
        # Only clear the slot if it is still ours: a displaced client's cleanup must not evict the
        # client that displaced it.
        if session is not None and session["client"] is websocket:
            session["client"] = None
            session["seen"] = time.monotonic()
        elif in_use.get("client") is websocket:
            in_use["client"] = None
        print("client disconnected")


async def _pump(websocket, stream) -> None:
    """Answer a client's commands and frame requests until it goes away."""
    async for message in websocket:
        request = json.loads(message)  # a closed connection ends the iteration, not an error
        if request["type"] == "command":
            stream._browser_exertion_movement = request.get('movement_profile') == 'exertion'
            stream._browser_speed_limit = (max(.15, min(.8, float(request["speed_limit"])))
                                           if request.get("speed_limit") is not None else None)
            _apply_walk_speed(stream, getattr(stream, "style", "stealth"))
            stream.set_command(request["forward"], request["lateral"], request["turn"])
        elif request["type"] == "style":
            stream.set_style(request["name"])
            _apply_walk_speed(stream, request["name"])
            print(f"style: {request['name']}")
        elif request["type"] == "context":
            if "equipment" in request and hasattr(stream, "set_equipment"):
                stream.set_equipment(request["equipment"])
            stream.set_context_qpos(np.asarray(request["qpos"], dtype=np.float64), request.get("frame"))
            if os.environ.get("WASD_DEBUG"):
                _debug["n"] = _debug.get("n", 0) + 1
                if _debug["n"] % 50 == 0:
                    print(
                        f"context frame={request.get('frame')} available={stream.available} "
                        f"robot=({request['qpos'][0]:+.2f},{request['qpos'][1]:+.2f}) "
                        f"gap={getattr(stream, 'tracking_error', lambda: float('nan'))():.2f}m "
                        f"robot=({request['qpos'][0]:+.2f},{request['qpos'][1]:+.2f},"
                        f"{_yaw_deg(request['qpos']):+6.1f}) "
                        f"{getattr(stream, 'chain_state', lambda: '')()} "
                        f"clientref={_yaw_of_quat(request.get('ref_quat')):+7.1f} "
                        f"refspeed={getattr(stream, 'reference_speed', lambda: float('nan'))():.2f} "
                        f"mode={getattr(stream, 'current_mode', lambda: '?')()} "
                        f"cmd={stream.command}",
                        flush=True,
                    )
        elif request["type"] == "lag":
            if not os.environ.get("WASD_IGNORE_LAG"):
                stream.set_lag(request["dx"], request["dy"])
            if os.environ.get("WASD_DEBUG"):
                print(
                    f"lag=({request['dx']:+.2f},{request['dy']:+.2f}) "
                    f"cmd={stream.command} frames={stream.available}",
                    flush=True,
                )
        elif request["type"] == "request":
            start, count = int(request["from"]), int(request["count"])
            # Generation is synchronous and can take tens of milliseconds; handing it to a thread
            # keeps this connection's event loop free to take the next command, so steering is not
            # queued behind the generation it should be affecting.
            block = await asyncio.to_thread(stream.frames, np.arange(start, start + count))
            await websocket.send(pack_frames(block, start))
        else:
            raise ValueError(f"unknown request type {request['type']!r}")


async def _main(
    onnx_dir: Path,
    mjcf: Path,
    host: str,
    port: int,
    seed: int | None,
    generator: str,
    camera_req: str | None = None,
    camera_rep: str | None = None,
) -> None:
    import websockets

    contract = load_contract(onnx_dir)
    # Built before the socket opens, so a client's first page load does not wait on a checkpoint.
    print(f"loading the {generator} generator...")
    stream = _build_stream(contract, mjcf, seed, generator, camera_req, camera_rep)
    in_use: dict = {"client": None}
    # No server-side keepalive pings. Through a Cloudflare tunnel the pong does not reliably come
    # back from the browser, and the default 20 s timeout then closes a perfectly live session with
    # "keepalive ping timeout" -- which the page sees as its reference simply stopping. Liveness is
    # already evident: a steering client sends its pose every control step. ONNX clients have
    # independent state, and disconnected sessions are retained briefly for reconnection.
    async with websockets.serve(
        lambda ws: _serve_client(ws, contract, stream, in_use),
        host,
        port,
        max_size=None,
        ping_interval=None,
    ):
        print(f"reference stream on ws://{host}:{port}")
        await asyncio.Future()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--onnx-dir", type=Path, default=DEFAULT_ONNX_DIR)
    parser.add_argument("--mjcf", type=Path, default=DEFAULT_MJCF)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument(
        "--generator",
        default="motionbricks",
        choices=["motionbricks", "camera_motionbricks", "onnx", "ardy"],
        help="which model invents the reference",
    )
    parser.add_argument(
        "--camera-req-addr",
        default=None,
        help="camera_motionbricks: request socket of the camera retarget server (default :28701)",
    )
    parser.add_argument(
        "--camera-rep-addr",
        default=None,
        help="camera_motionbricks: reply socket of the camera retarget server (default :28702)",
    )
    args = parser.parse_args()
    asyncio.run(
        _main(
            args.onnx_dir,
            args.mjcf,
            args.host,
            args.port,
            args.seed,
            args.generator,
            args.camera_req_addr,
            args.camera_rep_addr,
        )
    )


if __name__ == "__main__":
    main()
