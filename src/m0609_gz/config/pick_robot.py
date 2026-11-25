"""Use the same RG2 pad-only collision geometry as the MuJoCo pick scene."""
import xml.etree.ElementTree as ET


def with_pick_pads(robot_description):
    robot = ET.fromstring(robot_description)
    for link in robot.findall('link'):
        name = link.get('name')
        if name.startswith('rg2_'):
            for collision in link.findall('collision'):
                link.remove(collision)
    for side in ('left', 'right'):
        name = f'rg2_{side}_inner_finger'
        link = robot.find(f'link[@name="{name}"]')
        pad = ET.SubElement(link, 'collision', name=f'rg2_{side}_pad')
        ET.SubElement(pad, 'origin', xyz='0 .016 .042')
        ET.SubElement(ET.SubElement(pad, 'geometry'), 'box', size='.018 .006 .020')
        surface = robot.find(f'gazebo[@reference="{name}"]')
        for coefficient in ('mu1', 'mu2'):
            surface.find(coefficient).text = '1.2'
    return ET.tostring(robot, encoding='unicode')
