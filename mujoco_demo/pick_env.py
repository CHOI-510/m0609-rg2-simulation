"""A one-decision environment for bounded RG2 closing-time adjustments."""
import argparse
import json
from pathlib import Path

import numpy as np

if __package__:
    from .replay import Replay, Trajectory
else:
    from replay import Replay, Trajectory


class PickEnv:
    """reset() reaches the end of approach; step([action]) ends the episode.

    The scalar action is in [-1, 1]. It advances or delays closing by up to
    150 ms when the recorded gaps permit it. Lift timing is held fixed.
    """

    action_low = -1.
    action_high = 1.

    def __init__(self, trajectory_path):
        self.replay = Replay(Trajectory(Path(trajectory_path)))
        approach, close, lift = self.replay.trajectory.phases
        self.max_delay_s = min(.15, close.start - approach.end - self.replay.dt,
                               lift.start - close.end)
        if self.max_delay_s <= 0:
            raise ValueError('trajectory has no timing margin for the closing action')
        self.decision_tick = int(approach.end / self.replay.dt)
        self.ready = False

    def observation(self):
        pick = self.replay.pick
        data = pick.data
        return np.r_[data.qpos[pick.arm_q], data.qvel[pick.arm_v],
                     data.qpos[pick.finger_q], data.site('rg2_tcp').xpos,
                     data.body('cube').xpos].astype(np.float32)

    def reset(self):
        self.replay.reset()
        self.replay.advance(self.decision_tick)
        self.ready = True
        return self.observation(), {'max_close_shift_s': self.max_delay_s}

    def step(self, action):
        if not self.ready:
            raise RuntimeError('call reset() before step()')
        values = np.asarray(action, dtype=float)
        if values.shape != (1,) or not np.isfinite(values).all() or abs(values[0]) > 1.:
            raise ValueError('action must be a one-element vector in [-1, 1]')
        shift = float(values[0] * self.max_delay_s)
        self.replay.advance(self.replay.final_tick, shift)
        self.ready = False
        result = self.replay.summary()
        reward = float(result['grasp_success'])
        return self.observation(), reward, True, False, {**result, 'close_shift_s': shift}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectory', type=Path)
    args = parser.parse_args()
    env = PickEnv(args.trajectory)
    direct = Replay(env.replay.trajectory)
    direct.reset()
    direct.advance(direct.final_tick)
    initial, _ = env.reset()
    end, reward, terminated, truncated, result = env.step([0.])
    baseline_matches = (np.array_equal(direct.pick.data.qpos, env.replay.pick.data.qpos)
                        and np.array_equal(direct.pick.data.qvel, env.replay.pick.data.qvel)
                        and direct.summary() == env.replay.summary())
    print(json.dumps({'baseline_matches': baseline_matches, 'reward': reward,
                      'terminated': terminated, 'truncated': truncated, **result}))
    repeated, _ = env.reset()
    _, shifted_reward, shifted_done, _, shifted = env.step([1.])
    print(json.dumps({'reset_matches': bool(np.array_equal(initial, repeated)),
                      'shifted_reward': shifted_reward, 'terminated': shifted_done, **shifted}))
    valid = (baseline_matches and terminated and not truncated and shifted_done
             and np.array_equal(initial, repeated) and np.isfinite(end).all())
    return 0 if valid else 1


if __name__ == '__main__':
    raise SystemExit(main())
