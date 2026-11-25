"""Build the contact scene from the pinned M0609 model and RG2 assembly."""
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco

MODELS = Path(__file__).resolve().parent / "models"
ARM = tuple(f"joint_{index}" for index in range(1, 7))
GRIPPER = "rg2_finger_joint"
OPEN, CLOSED = -0.50, 0.70
CUBE_X, CUBE_Y, TABLE_Z = 0.47, 0.0, 0.30


def load_arm():
    return mujoco.MjModel.from_xml_path(str(MODELS / "m0609/scene.xml"))


def load_pick():
    root = ET.parse(MODELS / "m0609/scene.xml").getroot()
    root.remove(root.find("include"))
    arm = ET.parse(MODELS / "m0609/m0609.xml").getroot()
    rg2 = ET.parse(MODELS / "rg2.xml").getroot()
    asset = root.find("asset")
    for mesh in arm.findall("asset/mesh"):
        mesh.set("file", "m0609/" + mesh.get("file"))
    asset.extend(arm.find("asset"))
    asset.extend(rg2.find("asset"))

    world = root.find("worldbody")
    world.extend(arm.find("worldbody"))
    for geom in arm.findall(".//worldbody//geom"):
        # The official collision meshes overlap on adjacent links. Allow
        # their contacts with the scene using a separate collision mask.
        if geom.get("contype") is None:
            geom.set("contype", "1")
            geom.set("conaffinity", "0")
            geom.set("group", "0")
    for joint, damping in zip(arm.findall(".//joint"), (28, 35, 24, 12, 8, 4)):
        joint.set("damping", str(damping))
        joint.set("frictionloss", "0")
    wrist = world.find(".//body[@name='link_6']")
    wrist.extend(rg2.find("worldbody/body"))
    root.append(rg2.find("equality"))
    actuators = arm.find("actuator")
    ET.SubElement(actuators, "position", name=GRIPPER, joint=GRIPPER,
                  kp="24", kv="1.5", forcerange="-6 6", ctrlrange="-0.558505 0.785398")
    root.append(actuators)
    ET.SubElement(root, "option", timestep=".002", integrator="implicitfast", cone="elliptic")
    floor = world.find("geom[@name='floor']")
    floor.attrib.update(contype="0", group="2")
    ET.SubElement(world, "geom", name="table", type="box",
                  pos=f"{CUBE_X} {CUBE_Y} {TABLE_Z / 2}",
                  size=f".14 .12 {TABLE_Z / 2}", rgba=".28 .38 .44 1",
                  contype="0", conaffinity="7", group="2")
    cube = ET.SubElement(world, "body", name="cube",
                         pos=f"{CUBE_X} {CUBE_Y} {TABLE_Z + .020}")
    ET.SubElement(cube, "freejoint", name="cube_free")
    ET.SubElement(cube, "geom", name="cube", type="box", size=".020 .020 .020",
                  mass=".064", rgba=".9 .34 .1 1", contype="4", conaffinity="1", group="2")
    mesh_files = {mesh.get("file"): (MODELS / mesh.get("file")).read_bytes()
                  for mesh in asset.findall("mesh")}
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"), mesh_files)
