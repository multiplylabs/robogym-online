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

    def __init__(self, url: str, lead: int = 40, block: int = 50) -> None:
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
            json.dumps(
                {"type": "command", "forward": forward, "lateral": lateral, "turn": turn_deg}
            )
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


def _build_stream(contract: dict, mjcf: Path, seed: int | None, generator: str):
    """The generator behind this connection. One per client: each has its own command and robot."""
    if generator == "motionbricks":
        from .motionbricks_stream import MotionBricksStream
        from .scene import DEFAULT_MOTIONBRICKS_ROOT

        return MotionBricksStream(contract, mjcf, DEFAULT_MOTIONBRICKS_ROOT, seed=seed)
    if generator == "onnx":
        from .motionbricks_onnx import MotionBricksOnnxStream
        from .scene import DEFAULT_PLANNER_ROOT

        return MotionBricksOnnxStream(contract, mjcf, DEFAULT_PLANNER_ROOT, seed=seed)
    if generator == "ardy":
        from .wasd_stream import ReferenceStream

        return ReferenceStream(contract, mjcf, seed=seed)
    raise ValueError(f"unknown generator {generator!r}")


async def _serve_client(websocket, contract: dict, stream, in_use: dict) -> None:
    """Serve one client off the shared generator, displacing whoever held it.

    One at a time: the generator carries a session's worth of state -- its own motion so far, the
    heading, the correction -- and there is one robot to drive. Loading a second copy is not the
    answer either, at a couple of minutes and several gigabytes.
    
    The newest client wins, rather than being refused. Refusing looks like a broken demo from the
    outside: the page keeps playing its bundled clip and ignores the keyboard, with nothing to say
    why. And the holder is not always a browser -- an editor forwarding the port, or a tab left open
    on another desktop, is enough to lock out the tab actually being used.
    """
    previous = in_use.get("client")
    if previous is not None:
        print("displacing the previous client")
        try:
            await previous.close()
        except Exception:  # noqa: BLE001 - a client already gone is exactly what we wanted
            pass
    in_use["client"] = websocket
    stream.reset()
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
            }
        )
    )
    print(f"client connected: {websocket.remote_address}")
    try:
        await _pump(websocket, stream)
    finally:
        # Only clear the slot if it is still ours: a displaced client's cleanup must not evict the
        # client that displaced it.
        if in_use.get("client") is websocket:
            in_use["client"] = None
        print("client disconnected")


async def _pump(websocket, stream) -> None:
    """Answer a client's commands and frame requests until it goes away."""
    async for message in websocket:
        request = json.loads(message)  # a closed connection ends the iteration, not an error
        if request["type"] == "command":
            stream.set_command(request["forward"], request["lateral"], request["turn"])
        elif request["type"] == "style":
            stream.set_style(request["name"])
            print(f"style: {request['name']}")
        elif request["type"] == "context":
            stream.set_context_qpos(
                np.asarray(request["qpos"], dtype=np.float64), request.get("frame")
            )
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
    onnx_dir: Path, mjcf: Path, host: str, port: int, seed: int | None, generator: str
) -> None:
    import websockets

    contract = load_contract(onnx_dir)
    # Built before the socket opens, so a client's first page load does not wait on a checkpoint.
    print(f"loading the {generator} generator...")
    stream = _build_stream(contract, mjcf, seed, generator)
    in_use: dict = {"client": None}
    async with websockets.serve(
        lambda ws: _serve_client(ws, contract, stream, in_use), host, port, max_size=None
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
        choices=["motionbricks", "onnx", "ardy"],
        help="which model invents the reference",
    )
    args = parser.parse_args()
    asyncio.run(
        _main(args.onnx_dir, args.mjcf, args.host, args.port, args.seed, args.generator)
    )


if __name__ == "__main__":
    main()
