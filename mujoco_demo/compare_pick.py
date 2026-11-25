"""Compare independent Gazebo and MuJoCo measurements of one command file."""
import argparse
import json
from pathlib import Path

import numpy as np

if __package__:
    from .replay import NAMES, Trajectory
else:
    from replay import NAMES, Trajectory


def tracking(rows, trajectory, clock_origin=None):
    errors = []
    for row in rows:
        measured_at = (row['time_s'] if clock_origin is None
                       else row['joint_time_s'] - clock_origin)
        goal, _ = trajectory.target(measured_at)
        errors.append(np.asarray(row['positions_rad']) - goal)
    errors = np.array(errors)
    return {
        'samples': len(rows),
        'arm_rmse_rad': float(np.sqrt(np.mean(errors[:, :-1]**2))),
        'arm_max_abs_rad': float(np.max(np.abs(errors[:, :-1]))),
        'gripper_rmse_rad': float(np.sqrt(np.mean(errors[:, -1]**2))),
        'gripper_max_abs_rad': float(np.max(np.abs(errors[:, -1]))),
    }


def compare(trajectory, gazebo, mujoco):
    for report in (gazebo, mujoco):
        if (report['trajectory_sha256'] != trajectory.digest
                or report['joint_names'] != list(NAMES)):
            raise ValueError('report does not match the command file and joint ordering')
    gz_rows, mj_rows = gazebo['samples'], mujoco['first_episode_samples']
    terminal_phases = {}
    marks = {'initial': 0., **{phase.name: phase.end for phase in trajectory.phases},
             'hold': trajectory.end + 1.}
    for phase, expected in marks.items():
        gz_matches = [row for row in gz_rows if row['phase'] == phase and row['tcp_pose'] is not None]
        mj_matches = [row for row in mj_rows if row['phase'] == phase]
        if not gz_matches or not mj_matches:
            continue
        gz = min(gz_matches, key=lambda row: abs(row['tcp_time_s'] - gazebo['clock_origin_s'] - expected))
        mj = min(mj_matches, key=lambda row: abs(row['time_s'] - expected))
        gxyz, mxyz = gz['tcp_pose'][:3], mj['tcp_xyz']
        terminal_phases[phase] = {
            'target_time_s': expected,
            'gazebo_tcp_time_s': gz['tcp_time_s'] - gazebo['clock_origin_s'],
            'mujoco_time_s': mj['time_s'],
            'gazebo_tcp_xyz_m': gxyz, 'mujoco_tcp_xyz_m': mxyz,
            'tcp_distance_m': float(np.linalg.norm(np.subtract(gxyz, mxyz))),
            'gazebo_positions_rad': gz['positions_rad'], 'mujoco_positions_rad': mj['positions_rad'],
            'gazebo_cube_xyz_m': gz['cube_pose'][:3], 'mujoco_cube_xyz_m': mj['cube_xyz'],
        }
    return {
        'trajectory_sha256': trajectory.digest, 'joint_names': NAMES,
        'gazebo': {
            **{k: v for k, v in gazebo.items() if k not in ('samples', 'joint_names')},
            'tracking': tracking(gz_rows, trajectory, gazebo['clock_origin_s']),
        },
        'mujoco': {'tracking': tracking(mj_rows, trajectory), 'episodes': mujoco['episodes']},
        'phase_end_samples': terminal_phases,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectory', type=Path)
    parser.add_argument('gazebo', type=Path)
    parser.add_argument('mujoco', type=Path)
    args = parser.parse_args()
    try:
        report = compare(Trajectory(args.trajectory), json.loads(args.gazebo.read_text()),
                         json.loads(args.mujoco.read_text()))
    except (KeyError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
