# SPDX-License-Identifier: Apache-2.0
"""The same generator, as an ONNX graph on the CPU.

MotionBricks in Python wants a GPU, three checkpoints and about four gigabytes resident, which is
why the browser demo needs a machine behind it. GR00T Whole-Body Control ships the same model frozen
-- ``gear_sonic_deploy/planner/target_vel/V2/planner_sonic.onnx``, 774 MB, where SONIC's C++
controller loads it and CLAW drives it. Its graph carries MotionBricks' own module names
(``vqvae_pose_model``, ``quantize_cnn_multihead``) and its inputs are MotionBricks' control signals
exactly, so this is the same generator with the sampling loop lifted out into the caller.

Measured on eight CPU threads: **22 ms to produce 2.13 s of motion**, about ninety times real time.
A client needs a call every second or so, so a small virtual machine can host several -- no GPU, no
CUDA, no PyTorch.

The checkpoint is not the public MotionBricks release. It carries twenty-five modes against the
release's fifteen, including squats, kneeling and a wider set of styles, and their indices do not
line up. The names below come from CLAW's own mode table, which is the only mapping for this
checkpoint I have found; each is worth watching before trusting the label.

**Where the sampling loop lives.** The Python agent keeps its own rolling buffer and hands out one
frame at a time; the graph has no memory at all. Every call is given four frames of context and
answers with up to sixty-four, of which this keeps a prefix -- so the context is simply the tail of
what has already been committed, and the receding-horizon bookkeeping that was implicit becomes
explicit here.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .motionbricks_stream import MotionBricksStream

# Where the frozen model lives inside a GR00T Whole-Body Control checkout.
PLANNER_RELATIVE = Path("gear_sonic_deploy/planner/target_vel/V2/planner_sonic.onnx")

# Mode names for this checkpoint, from CLAW's table (`web_wasd_demo/ws_bridge.py`, MOTION_SETS).
# Index 0 is idle, and 7 is absent there as it is here.
ONNX_MODES: dict[str, int] = {
    "idle": 0,
    "slow_walk": 1,
    "walk": 2,
    "run": 3,
    "squat": 4,
    "kneel_two": 5,
    "kneel_one": 6,
    "hand_crawl": 8,
    "idle_boxing": 9,
    "walk_boxing": 10,
    "left_jab": 11,
    "right_jab": 12,
    "random_punches": 13,
    "elbow_crawl": 14,
    "left_hook": 15,
    "right_hook": 16,
    "happy": 17,
    "stealth": 18,
    "injured": 19,
    "careful": 20,
    "object_carrying": 21,
    "crouch": 22,
    "happy_dance": 23,
    "zombie": 24,
    "point": 25,
    "scared": 26,
}

# Styles offered to an operator, in selection order. Kneeling, squatting and the crawls are left out
# for the same reason the Python backend leaves out its crawls: this is a walking tracker, and a
# reference that goes to the floor takes the robot with it.
ONNX_STYLES: tuple[str, ...] = (
    "walk",
    "slow_walk",
    "run",
    "stealth",
    "walk_boxing",
    "injured",
    "happy",
    "happy_dance",
    "zombie",
    "careful",
    "object_carrying",
    "scared",
)

# Frames kept from each generated horizon, at the generator's 30 Hz. The graph offers up to 64 --
# a little over two seconds -- and keeping a prefix is what lets a held key change direction
# promptly: the rest is re-planned against whatever the command is by then. The Python backend never
# needed this constant because its agent does the same bookkeeping internally.
COMMIT_FRAMES = 24

# Threads for the session. The graph is the only heavy thing on this process.
INTRA_OP_THREADS = 8


class MotionBricksOnnxStream(MotionBricksStream):
    """MotionBricks behind an ONNX session, with the command layer inherited unchanged."""

    @staticmethod
    def _assert_joint_order(root: Path, joint_names: list[str]) -> None:
        """Same guarantee as the Python backend, against whichever tree was pointed at.

        The graph emits qpos in MotionBricks' joint order and it is copied across positionally, so
        the check matters as much here -- but the skeleton lives at a different depth in a
        whole-body-control checkout than in a MotionBricks one.
        """
        candidates = [
            root / "assets" / "skeletons" / "g1" / "g1.xml",
            root / "motionbricks" / "assets" / "skeletons" / "g1" / "g1.xml",
            root.parent / "motionbricks" / "assets" / "skeletons" / "g1" / "g1.xml",
        ]
        for path in candidates:
            if path.is_file():
                MotionBricksStream._assert_joint_order(path.parents[3], joint_names)
                return
        print(f"note: no G1 skeleton beside {root}; joint order not verified")

    def _setup_model(self, root: Path, seed: int | None, device: str) -> None:
        import onnxruntime as ort

        path = root if root.suffix == ".onnx" else root / PLANNER_RELATIVE
        if not path.is_file():
            raise FileNotFoundError(
                f"planner graph not found: {path}\n"
                "Point --motionbricks-root at a GR00T-WholeBodyControl checkout, or at the "
                "planner_sonic.onnx itself."
            )
        options = ort.SessionOptions()
        options.intra_op_num_threads = INTRA_OP_THREADS
        # CPU on purpose: this backend exists to not need a GPU.
        self._session = ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])
        self._input_types = {i.name: i.type for i in self._session.get_inputs()}
        self._seed = 0 if seed is None else int(seed)
        self._modes = list(ONNX_MODES)
        # The graph is stateless, so the horizon it was traced with is a property of the output.
        self._horizon_dt = 0.0

    def _reset_model(self) -> None:
        # Nothing to reset: the graph holds no state, and the context is the tail of `_qpos`, which
        # the caller clears.
        self._pending = np.zeros((0, 7 + len(self._builder.joint_names)))

    @property
    def styles(self) -> tuple[str, ...]:
        return tuple(name for name in ONNX_STYLES if name in self._modes)

    def _context(self) -> np.ndarray:
        """The four frames the next generation continues from, nudged towards the robot.

        Before anything has been generated the robot's default pose stands in, repeated: the model
        has to be given somewhere to start, and a standing pose is the honest one.
        """
        window = 4
        if self._qpos.shape[0] >= window:
            context = self._qpos[-window:].copy()
        else:
            seed = np.zeros((window, self._qpos.shape[1]))
            seed[:, 2] = 0.793  # pelvis height of the G1's default stance
            seed[:, 3] = 1.0  # identity orientation, wxyz
            if self._qpos.shape[0]:
                seed[-self._qpos.shape[0] :] = self._qpos
            context = seed
        context[:, 0] += float(self._correction[0])
        context[:, 1] += float(self._correction[1])
        return context[None, ...]

    def _generate_frame(self) -> None:
        """Take one frame, generating another horizon when the last one runs out.

        The command is advanced every frame even when no generation happens. It is not a value but a
        state -- the heading integrates the turn key and the travel direction sweeps towards the
        commanded one -- so advancing it only when the model is called runs both at a twenty-fourth
        of their proper rate: turning crawls, and a reversal that should sweep in a second and a half
        takes closer to a minute.
        """
        facing, movement = self._command_vectors()
        if self._pending.shape[0] == 0:
            mode = self.current_mode()
            feed = {
                "context_mujoco_qpos": self._context(),
                "target_vel": np.array([-1.0]),  # -1: the mode's own pace
                "mode": np.array([ONNX_MODES[mode]]),
                "movement_direction": movement[None, :],
                "facing_direction": facing[None, :],
                "random_seed": np.array([self._seed]),
                "has_specific_target": np.zeros((1, 1)),
                "specific_target_positions": np.zeros((1, 4, 3)),
                "specific_target_headings": np.zeros((1, 4)),
                # Every token count the graph allows; the model picks its own horizon length and
                # says how much of the output is real.
                "allowed_pred_num_tokens": np.ones((1, 11)),
                "height": np.array([-1.0]),
            }
            self._seed += 1
            cast = {"tensor(float)": np.float32, "tensor(int32)": np.int32, "tensor(int64)": np.int64}
            feed = {
                name: np.ascontiguousarray(value, dtype=cast.get(self._input_types[name], np.float32))
                for name, value in feed.items()
            }
            frames, count = self._session.run(None, feed)
            valid = int(np.ravel(count)[0])
            # Only a prefix is kept: the rest is re-planned against whatever the command is by then,
            # which is what makes a held key change direction promptly.
            self._pending = np.asarray(frames[0, : min(valid, COMMIT_FRAMES)], dtype=np.float64)

        self._advance_correction()
        self._qpos = np.concatenate([self._qpos, self._pending[:1]], axis=0)
        self._pending = self._pending[1:]
