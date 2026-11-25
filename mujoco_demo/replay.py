"""Load the commanded ROS trajectories and replay them in simulation time."""
from bisect import bisect_right
from contextlib import nullcontext
from dataclasses import dataclass
import hashlib
import json
import math

import numpy as np

if __package__:
    from .model import ARM, GRIPPER
else:
    from model import ARM, GRIPPER

NAMES = (*ARM, GRIPPER)
PHASES = ('approach', 'close', 'lift')


@dataclass
class Phase:
    name: str
    start: float
    times: np.ndarray
    indexes: list
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray

    @property
    def end(self):
        return self.start + self.times[-1]

    def sample(self, elapsed):
        if elapsed >= self.times[-1]:
            return self.positions[-1]
        i = max(0, bisect_right(self.times, elapsed) - 1)
        dt = self.times[i + 1] - self.times[i]
        u = max(0., (elapsed - self.times[i]) / dt)
        left, right = self.positions[i:i+2]
        delta = right - left
        if not self.velocities.size:
            return left + u * delta
        v0, v1 = self.velocities[i:i+2] * dt
        if not self.accelerations.size:
            return left + u * (v0 + u * (3*delta - 2*v0 - v1 + u * (-2*delta + v0 + v1)))
        a0, a1 = self.accelerations[i:i+2] * dt**2
        c3 = 10*delta - 6*v0 - 4*v1 - 1.5*a0 + .5*a1
        c4 = -15*delta + 8*v0 + 7*v1 + 1.5*a0 - a1
        c5 = 6*delta - 3*v0 - 3*v1 - .5*a0 + .5*a1
        return left + u * (v0 + u * (.5*a0 + u * (c3 + u * (c4 + u*c5))))


class Trajectory:
    def __init__(self, path):
        try:
            raw = path.read_bytes()
            self.digest = hashlib.sha256(raw).hexdigest()
            document = json.loads(raw)
            if document['schema'] != 'm0609.pick.v1' or document['frame'] != 'world':
                raise ValueError('unsupported schema or frame')
            if [phase['name'] for phase in document['phases']] != list(PHASES):
                raise ValueError('approach, close and lift commands are required')
            initial = document['initial_positions_rad']
            if set(initial) != set(NAMES):
                raise ValueError('unexpected initial joint names')
            self.initial = np.array([initial[name] for name in NAMES], dtype=float)
            if not np.isfinite(self.initial).all():
                raise ValueError('invalid initial pose')
            self.phases = []
            previous_end = 0.
            for record, expected in zip(document['phases'], (ARM, (GRIPPER,), ARM)):
                names = record['joint_names']
                if len(names) != len(expected) or set(names) != set(expected):
                    raise ValueError(f'invalid joint names in {record["name"]}')
                start = float(record['gazebo_start_s'])
                points = record['points']
                times = np.array([point['time_s'] for point in points], dtype=float)
                if (len(times) < 2 or times[0] != 0 or not np.isfinite(times).all()
                        or not (np.diff(times) > 0).all() or not math.isfinite(start)
                        or start < previous_end):
                    raise ValueError(f'invalid timing in {record["name"]}')
                arrays = []
                for key in ('positions_rad', 'velocities_rad_s', 'accelerations_rad_s2'):
                    values = np.array([point[key] for point in points], dtype=float)
                    shapes = [(len(times), len(names))]
                    if key != 'positions_rad':
                        shapes.append((len(times), 0))
                    if values.shape not in shapes or not np.isfinite(values).all():
                        raise ValueError(f'invalid {key} in {record["name"]}')
                    arrays.append(values)
                if arrays[2].size and not arrays[1].size:
                    raise ValueError('acceleration requires velocity')
                phase = Phase(record['name'], start, times,
                              [NAMES.index(name) for name in names], *arrays)
                self.phases.append(phase)
                previous_end = phase.end
            self.end = self.phases[-1].end
        except (OSError, KeyError, TypeError, ValueError) as error:
            raise ValueError(f'{path}: {error}') from error

    def target(self, t, close_delay=0.):
        position = self.initial.copy()
        name = 'initial'
        for phase in self.phases:
            start = phase.start + (close_delay if phase.name == 'close' else 0.)
            if t < start:
                break
            position[phase.indexes] = phase.sample(t - start)
            name = phase.name
        return position, name


class Replay:
    def __init__(self, trajectory):
        if __package__:
            from .demo_pick import Pick
        else:
            from demo_pick import Pick

        self.trajectory = trajectory
        self.pick = Pick()
        self.dt = self.pick.model.opt.timestep

    def reset(self):
        pick = self.pick
        pick.reset(self.trajectory.initial[:-1], self.trajectory.initial[-1])
        # Give the free cube the same stationary settling period as the standalone demo.
        for _ in range(round(.5 / self.dt)):
            pick.step(self.trajectory.initial[:-1], self.trajectory.initial[-1])
        self.tick = 0
        self.initial_z = float(pick.data.geom_xpos[pick.cube, 2])
        self.settled = pick.table in pick.cube_contacts()
        self.peak = 0.
        self.min_rise = math.inf
        self.bilateral = 0
        self.samples = []
        self.lift_end_tick = math.ceil(self.trajectory.end / self.dt)
        self.hold_steps = math.ceil(1. / self.dt)
        self.final_tick = self.lift_end_tick + self.hold_steps
        self.observe(self.trajectory.initial, 'initial')

    def observe(self, target, phase):
        pick = self.pick
        data = pick.data
        tcp = data.site('rg2_tcp')
        cube = data.body('cube')
        positions = [*data.qpos[pick.arm_q], data.qpos[pick.finger_q]]
        self.samples.append({
            'time_s': self.tick * self.dt, 'phase': phase,
            'positions_rad': positions, 'target_rad': target.tolist(),
            'tcp_xyz': tcp.xpos.tolist(), 'cube_xyz': cube.xpos.tolist(),
            'contacts': sorted(pick.model.geom(geom).name for geom in pick.cube_contacts()),
        })

    def advance(self, target_tick, close_delay=0.):
        while self.tick < target_tick:
            self.tick += 1
            target, phase = self.trajectory.target(self.tick * self.dt, close_delay)
            pick = self.pick
            pick.step(target[:-1], target[-1])
            rise = pick.data.geom_xpos[pick.cube, 2] - self.initial_z
            self.peak = max(self.peak, rise)
            if self.tick > self.lift_end_tick:
                self.min_rise = min(self.min_rise, rise)
                self.bilateral += pick.cube_contacts() == pick.pads
                phase = 'hold'
            if self.tick % 5 == 0 or self.tick == target_tick:
                self.observe(target, phase)

    def summary(self):
        complete = self.tick == self.final_tick
        return {
            'start_cube_z_m': self.initial_z, 'settled': self.settled,
            'peak_rise_m': float(self.peak),
            'hold_min_rise_m': float(self.min_rise) if complete else None,
            'hold_s': self.hold_steps * self.dt if complete else 0.,
            'bilateral_hold_s': self.bilateral * self.dt,
            'grasp_success': bool(complete and self.settled and self.min_rise >= .05
                                  and self.bilateral == self.hold_steps),
        }


def run_replay(path, episodes, output=None, viewer_factory=None):
    trajectory = Trajectory(path)
    replay = Replay(trajectory)
    results = []
    reference = None
    samples = []
    interrupted = False
    context = viewer_factory(replay.pick) if viewer_factory else nullcontext()
    with context as viewer:
        while True:
            replay.reset()
            data = replay.pick.data
            state = np.r_[data.qpos, data.qvel, data.ctrl]
            if reference is None:
                reference = state
            if viewer is None:
                replay.advance(replay.final_tick)
            elif not viewer.play_replay(replay):
                if viewer.is_running() and viewer.restart_requested:
                    continue
                interrupted = True
                break
            summary = {'episode': len(results) + 1,
                       'reset_matches': bool(np.array_equal(state, reference)), **replay.summary()}
            results.append(summary)
            print(json.dumps(summary), flush=True)
            if len(results) == 1:
                samples = replay.samples
            if len(results) >= episodes and (viewer is None or not viewer.replay_again(replay, summary)):
                break
    if output:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps({
            'simulator': 'MuJoCo', 'trajectory_sha256': trajectory.digest,
            'joint_names': NAMES, 'episodes': results, 'first_episode_samples': samples,
        }, indent=2, allow_nan=False) + '\n')
    passed = all(row['grasp_success'] and row['reset_matches'] for row in results)
    return 0 if not interrupted and len(results) >= episodes and passed else 1
