"""Run the same contact-grasp sequence headlessly or in the MuJoCo viewer."""
import argparse
from contextlib import nullcontext
import math
from pathlib import Path
import time

import mujoco
import numpy as np

if __package__:
    from .model import ARM, CLOSED, GRIPPER, OPEN, load_arm, load_pick
else:
    from model import ARM, CLOSED, GRIPPER, OPEN, load_arm, load_pick

# Offline IK for TCP [0.47, 0, z] with orientation diag(1, -1, -1).
# The two waypoints are on the same elbow branch, at z = .405 and .320 m.
ABOVE = np.array([-.01330349, .50481072, .32346998, .00054814, 2.31311204, -1.58393328])
AT_CUBE = np.array([-.01326659, .31322084, .88905511, .00043296, 1.93911680, -1.58411103])
KP = np.array([1400, 1800, 1500, 500, 500, 250])
KD = np.array([32, 45, 30, 8, 8, 4])


class Viewer:
    def __init__(self, pick):
        self.pick = pick

    def __enter__(self):
        import glfw

        self.glfw = glfw
        if not glfw.init():
            raise RuntimeError("GLFW could not open a desktop display")
        self.window = glfw.create_window(1000, 750, "M0609 + RG2", None, None)
        if not self.window:
            glfw.terminate()
            raise RuntimeError("GLFW could not create an OpenGL window")
        glfw.make_context_current(self.window)
        model = self.pick.model
        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(model, self.camera)
        self.camera.lookat[:] = (.33, 0, .47)
        self.camera.distance = 1.65
        self.scene = mujoco.MjvScene(model, maxgeom=1000)
        self.options = mujoco.MjvOption()
        self.options.geomgroup[0] = 0
        self.options.geomgroup[2] = 1
        self.context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_100)
        return self

    def __exit__(self, *_):
        self.context.free()
        self.glfw.destroy_window(self.window)
        self.glfw.terminate()

    def is_running(self):
        return not self.glfw.window_should_close(self.window)

    def sync(self):
        width, height = self.glfw.get_framebuffer_size(self.window)
        mujoco.mjv_updateScene(self.pick.model, self.pick.data, self.options, None,
                               self.camera, mujoco.mjtCatBit.mjCAT_ALL, self.scene)
        mujoco.mjr_render(mujoco.MjrRect(0, 0, width, height), self.scene, self.context)
        self.glfw.swap_buffers(self.window)
        self.glfw.poll_events()


class Pick:
    def __init__(self):
        self.model = load_pick()
        self.data = mujoco.MjData(self.model)
        self.arm_q = [self.model.joint(name).qposadr.item() for name in ARM]
        self.arm_v = [self.model.joint(name).dofadr.item() for name in ARM]
        self.arm_ctrl = [self.model.actuator(name).id for name in ARM]
        self.grip_ctrl = self.model.actuator(GRIPPER).id
        self.cube = self.model.geom("cube").id
        self.table = self.model.geom("table").id
        self.pads = {self.model.geom(f"rg2_{side}_pad").id for side in ("left", "right")}
        self.mimics = {}
        for index in range(self.model.neq):
            joint = self.model.joint(self.model.eq_obj1id[index])
            offset, ratio = self.model.eq_data[index, :2]
            self.mimics[joint.qposadr.item()] = (offset, ratio)
        self.finger_q = self.model.joint(GRIPPER).qposadr.item()
        self.contact_force = np.zeros(6)

    def reset(self, arm=ABOVE, grip=OPEN):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.arm_q] = arm
        self.data.qpos[self.finger_q] = grip
        for address, (offset, ratio) in self.mimics.items():
            self.data.qpos[address] = offset + ratio * grip
        self.data.ctrl[self.grip_ctrl] = grip
        mujoco.mj_forward(self.model, self.data)
        self.set_control(arm, grip)

    def set_control(self, target, grip):
        data = self.data
        data.ctrl[self.arm_ctrl] = (KP * (target - data.qpos[self.arm_q])
                                    - KD * data.qvel[self.arm_v] + data.qfrc_bias[self.arm_v])
        data.ctrl[self.grip_ctrl] = grip

    def step(self, target, grip):
        self.set_control(target, grip)
        mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def cube_contacts(self):
        contacts = set()
        for index, contact in enumerate(self.data.contact):
            if contact.efc_address < 0 or self.cube not in (contact.geom1, contact.geom2):
                continue
            mujoco.mj_contactForce(self.model, self.data, index, self.contact_force)
            if self.contact_force[0] > 0:
                other = contact.geom2 if contact.geom1 == self.cube else contact.geom1
                contacts.add(other)
        return contacts

    def episode(self, viewer=None):
        self.reset()
        data = self.data
        reset_state = np.r_[data.time, data.qpos, data.qvel, data.ctrl]
        dt = self.model.opt.timestep
        started = time.monotonic()
        for _ in range(round(.5 / dt)):
            self.step(ABOVE, OPEN)
            if not self.view_step(viewer, started):
                return None
        initial = data.geom_xpos[self.cube, 2]
        settled = (self.table in self.cube_contacts()
                   and np.linalg.norm(data.joint("cube_free").qvel) < 1e-4)
        for duration, start, end, grip_start, grip_end in (
            (1.8, ABOVE, AT_CUBE, OPEN, OPEN),
            (.8, AT_CUBE, AT_CUBE, OPEN, CLOSED),
            (.4, AT_CUBE, AT_CUBE, CLOSED, CLOSED),
            (1.8, AT_CUBE, ABOVE, CLOSED, CLOSED),
        ):
            steps = round(duration / dt)
            for index in range(1, steps + 1):
                u = index / steps
                u = u * u * (3 - 2 * u)
                self.step(start + u * (end - start), grip_start + u * (grip_end - grip_start))
                if not self.view_step(viewer, started):
                    return None

        min_rise = math.inf
        bilateral_steps = 0
        steps = math.ceil(1.0 / dt)
        for _ in range(steps):
            self.step(ABOVE, CLOSED)
            min_rise = min(min_rise, data.geom_xpos[self.cube, 2] - initial)
            bilateral_steps += self.cube_contacts() == self.pads
            if not self.view_step(viewer, started):
                return None
        return reset_state, settled, initial, min_rise, bilateral_steps * dt, steps * dt

    def view_step(self, viewer, started):
        if viewer is None:
            return True
        if not viewer.is_running():
            return False
        # Synchronize the window at 50 Hz and pace it by simulation time.
        frame = round(self.data.time / self.model.opt.timestep)
        if frame % 10 == 0:
            viewer.sync()
            time.sleep(max(0.0, started + self.data.time - time.monotonic()))
        return True


def check_arm():
    model = load_arm()
    data = mujoco.MjData(model)
    data.actuator("joint_1").ctrl[0] = 30
    for _ in range(round(2.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    displacement = data.joint("joint_1").qpos.item()
    mujoco.mj_resetData(model, data)
    reset = data.time == 0 and not np.any(np.r_[data.qpos, data.qvel, data.ctrl])
    print(f"arm joints={model.njnt} actuators={model.nu} joint_1={displacement:.6f} rad reset={reset}")
    return model.njnt == model.nu == 6 and displacement > .1 and reset


def check_gripper():
    pick = Pick()
    pick.reset()
    model, data = pick.model, pick.data
    passed = True
    for target in (OPEN, CLOSED):
        for _ in range(round(2.0 / model.opt.timestep)):
            pick.step(ABOVE, target)
        angle = data.qpos[pick.finger_q]
        mimic_error = max(abs(data.qpos[address] - offset - ratio * angle)
                          for address, (offset, ratio) in pick.mimics.items())
        left, right = (data.geom(f"rg2_{side}_pad").xpos for side in ("left", "right"))
        lateral = data.site("rg2_tcp").xmat.reshape(3, 3)[:, 1]
        gap = abs((left - right) @ lateral) - 2 * .003
        print(f"grip target={target:+.2f} actual={angle:+.6f} rad gap={gap*1000:.2f} mm "
              f"mimic_max_error={mimic_error:.6f} rad")
        passed &= abs(angle - target) < .005 and mimic_error < .005
    return passed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--viewer", action="store_true", help="play the sequence in a GUI")
    mode.add_argument("--check-arm", action="store_true", help="check the upstream arm")
    mode.add_argument("--check-gripper", action="store_true", help="check RG2 open/close and mimic")
    mode.add_argument("--trajectory", type=Path, help="replay a recorded MoveIt trajectory")
    parser.add_argument("--report", type=Path, help="write replay results and measured state")
    parser.add_argument("--episodes", type=int, default=1)
    args = parser.parse_args()
    if args.episodes < 1:
        parser.error("--episodes must be positive")
    if args.check_arm:
        return 0 if check_arm() else 1
    if args.check_gripper:
        return 0 if check_gripper() else 1

    if args.trajectory:
        if __package__:
            from .replay import run_replay
        else:
            from replay import run_replay

        try:
            return run_replay(args.trajectory, args.episodes, args.report)
        except ValueError as error:
            parser.error(str(error))
    if args.report:
        parser.error("--report requires --trajectory")

    pick = Pick()
    pick.reset()
    context = Viewer(pick) if args.viewer else nullcontext()
    successes = 0
    reference = None
    with context as viewer:
        for episode in range(1, args.episodes + 1):
            result = pick.episode(viewer)
            if result is None:
                print("Viewer closed before the episode finished.")
                return 1
            reset, settled, start, rise, bilateral, hold = result
            if reference is None:
                reference = reset
            reset_ok = np.array_equal(reset, reference)
            passed = reset_ok and settled and rise >= .05 and bilateral == hold
            successes += passed
            print(f"episode={episode:02d} reset={'OK' if reset_ok else 'FAIL'} "
                  f"settled={bool(settled)} start={start*1000:.3f} mm "
                  f"min_rise={rise*1000:.3f} mm hold={hold:.3f} s "
                  f"bilateral_only={bilateral:.3f} s {'PASS' if passed else 'FAIL'}",
                  flush=True)
    print(f"{successes}/{args.episodes} passed")
    return 0 if successes == args.episodes else 1


if __name__ == "__main__":
    raise SystemExit(main())
