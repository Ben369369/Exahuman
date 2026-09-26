#!/usr/bin/env python
"""Build the AlohaMini digital twin (MuJoCo MJCF + URDF) from one parameter set.

Arms are the real SO-101 model from TheRobotStudio/SO-ARM100 (``arms/so101``).
The mobile base, column and lift carriage are simple primitives sized from the
lerobot AlohaMini driver (``src/lerobot/robots/alohamini/model_specs.py``) because
the real AlohaMini base meshes/URDFs are not in either repo (Git LFS stubs only).
Every value marked MEASURE is a placeholder: measure the lab robot and update it.

    python hybrid_twin/tools/build_twin.py                  # alohamini1 (default)
    python hybrid_twin/tools/build_twin.py --robot-model alohamini2

Outputs:
    hybrid_twin/mujoco/alohamini_twin.xml   (self-contained, loads with mujoco>=3.2)
    hybrid_twin/urdf/alohamini_twin.urdf    (for three.js / urdf-loaders on the web)
"""

import argparse
import copy
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco

ROOT = Path(__file__).resolve().parents[1]
SO101_DIR = ROOT / "arms" / "so101"

# Mirrors ROBOT_SPECS in src/lerobot/robots/alohamini/model_specs.py.
ROBOT_SPECS = {
    "alohamini1": {"wheel_radius": 0.05, "base_radius": 0.125},
    "alohamini2": {"wheel_radius": 0.063, "base_radius": 0.195},
    "alohamini2pro": {"wheel_radius": 0.063, "base_radius": 0.195},
}

# Wheel drive directions from AlohaMini._body_to_wheel_raw: angles [240, 0, 120] - 90 deg
# for (left, back, right). Each wheel sits 90 deg clockwise of its drive direction.
WHEELS = {"left": 150.0, "back": -90.0, "right": 30.0}

GEOM = {
    "chassis_height": 0.08,  # MEASURE
    "chassis_clearance": 0.01,  # MEASURE: gap between floor and chassis underside
    "wheel_width": 0.03,  # MEASURE
    "column_size": (0.04, 0.08, 1.05),  # MEASURE: x, y, height; front camera sits at ~1.10 m
    "column_x": -0.06,  # MEASURE: column offset behind the base centre
    "carriage_size": (0.10, 0.34, 0.05),  # MEASURE
    "carriage_z0": 0.15,  # MEASURE: carriage height above chassis top at lift_axis.height_mm = 0
    "lift_travel": 0.60,  # lift_axis.py soft_max_mm = 600
    "arm_mount_x": 0.05,  # MEASURE: arm base forward of the column
    "arm_mount_y": 0.12,  # MEASURE: +/- lateral offset of each arm
}


def rgba(hexstr: str, a: float = 1.0) -> list[float]:
    return [int(hexstr[i : i + 2], 16) / 255 for i in (0, 2, 4)] + [a]


def build_mjcf(spec_params: dict, out_path: Path) -> None:
    wr, br = spec_params["wheel_radius"], spec_params["base_radius"]
    g = GEOM
    chassis_r = br + 0.035
    chassis_z = wr + g["chassis_clearance"] + g["chassis_height"] / 2

    spec = mujoco.MjSpec()
    spec.modelname = "alohamini_twin"
    spec.compiler.degree = False
    spec.meshdir = "../arms/so101/assets"  # relative to mujoco/, forward slashes for Linux/Pi
    spec.option.timestep = 0.002
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST  # stiff lift/base actuators

    tex = spec.add_texture(name="grid", type=mujoco.mjtTexture.mjTEXTURE_2D,
                           builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER, width=300, height=300,
                           rgb1=[0.2, 0.3, 0.4], rgb2=[0.1, 0.2, 0.3], mark=mujoco.mjtMark.mjMARK_EDGE,
                           markrgb=[0.8, 0.8, 0.8])
    mat = spec.add_material(name="grid", texrepeat=[5, 5], texuniform=True, reflectance=0.2)
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = tex.name

    wb = spec.worldbody
    wb.add_light(pos=[0, 0, 3.5], dir=[0, 0, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)
    wb.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0, 0, 0.05], material="grid")

    # Base is driven kinematically in the world frame (x, y, yaw). The twin driver converts the
    # robot's body-frame x.vel / y.vel / theta.vel into these, see tools/check_twin.py.
    base = wb.add_body(name="base_link", pos=[0, 0, chassis_z])
    base.add_joint(name="base_x", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[1, 0, 0], damping=0)
    base.add_joint(name="base_y", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, 1, 0], damping=0)
    base.add_joint(name="base_yaw", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1], damping=0)
    base.add_geom(name="chassis", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                  size=[chassis_r, g["chassis_height"] / 2, 0], rgba=rgba("3a3f47"), mass=4.0, contype=0, conaffinity=0)
    base.add_geom(name="front_marker", type=mujoco.mjtGeom.mjGEOM_BOX, pos=[chassis_r - 0.02, 0, g["chassis_height"] / 2],
                  size=[0.02, 0.01, 0.002], rgba=rgba("e8a33d"), contype=0, conaffinity=0, mass=0)

    wheel_z = wr - chassis_z
    for name, drive_deg in WHEELS.items():
        pos_rad = math.radians(drive_deg - 90)
        ax, ay, hw = math.cos(pos_rad), math.sin(pos_rad), g["wheel_width"] / 2
        w = base.add_body(name=f"wheel_{name}", pos=[br * ax, br * ay, wheel_z])
        w.add_joint(name=f"wheel_{name}_spin", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[ax, ay, 0], damping=0.01)
        # Visual only: the base moves through base_x/base_y/base_yaw, not wheel contact.
        w.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[wr, 0, 0], fromto=[-hw * ax, -hw * ay, 0, hw * ax, hw * ay, 0],
                   rgba=rgba("1b1d21"), contype=0, conaffinity=0, mass=0.2)

    cx, cy, ch = g["column_size"]
    col_bottom = g["chassis_height"] / 2
    base.add_geom(name="column", type=mujoco.mjtGeom.mjGEOM_BOX, pos=[g["column_x"], 0, col_bottom + ch / 2],
                  size=[cx / 2, cy / 2, ch / 2], rgba=rgba("9aa3ad"), mass=1.5, contype=0, conaffinity=0)

    # lift_axis.height_mm maps 1:1 to this slide joint (metres).
    carriage = base.add_body(name="vertical_link",
                             pos=[g["column_x"] + cx / 2 + g["carriage_size"][0] / 2, 0, col_bottom + g["carriage_z0"]])
    carriage.add_joint(name="lift", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, 0, 1],
                       range=[0, g["lift_travel"]], limited=mujoco.mjtLimited.mjLIMITED_TRUE, damping=50)
    sx, sy, sz = g["carriage_size"]
    carriage.add_geom(name="carriage", type=mujoco.mjtGeom.mjGEOM_BOX, size=[sx / 2, sy / 2, sz / 2],
                      rgba=rgba("5b6572"), mass=0.8, contype=0, conaffinity=0)

    for side, sign in (("left", 1), ("right", -1)):
        # attach_body moves the body out of its spec, so load a fresh copy per arm.
        arm = mujoco.MjSpec.from_file(str(SO101_DIR / "so101_new_calib.xml"))
        frame = carriage.add_frame(pos=[g["arm_mount_x"], sign * g["arm_mount_y"], sz / 2])
        frame.attach_body(arm.body("base"), f"{side}_", "")

    for jn in ("base_x", "base_y", "base_yaw"):
        spec.add_actuator(name=f"{jn}_vel", target=jn, trntype=mujoco.mjtTrn.mjTRN_JOINT,
                          gainprm=[500] + [0] * 9, biastype=mujoco.mjtBias.mjBIAS_AFFINE,
                          biasprm=[0, 0, -500] + [0] * 7)
    spec.add_actuator(name="lift_pos", target="lift", trntype=mujoco.mjtTrn.mjTRN_JOINT,
                      gainprm=[20000] + [0] * 9, biastype=mujoco.mjtBias.mjBIAS_AFFINE,
                      biasprm=[0, -20000, -800] + [0] * 7,
                      ctrlrange=[0, g["lift_travel"]], ctrllimited=mujoco.mjtLimited.mjLIMITED_TRUE)

    spec.compile()  # fail here rather than in the viewer
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(spec.to_xml())


def build_urdf(spec_params: dict, out_path: Path) -> None:
    wr, br = spec_params["wheel_radius"], spec_params["base_radius"]
    g = GEOM
    chassis_r = br + 0.035
    chassis_z = wr + g["chassis_clearance"] + g["chassis_height"] / 2
    cx, cy, ch = g["column_size"]
    sx, sy, sz = g["carriage_size"]
    col_bottom = g["chassis_height"] / 2

    robot = ET.Element("robot", name="alohamini_twin")

    def material(name, hexstr):
        m = ET.SubElement(robot, "material", name=name)
        ET.SubElement(m, "color", rgba=" ".join(f"{v:.3f}" for v in rgba(hexstr)))

    material("chassis", "3a3f47")
    material("column", "9aa3ad")
    material("carriage", "5b6572")
    material("wheel", "1b1d21")

    def link(name, shape=None, origin=(0, 0, 0), rpy=(0, 0, 0), mat=None):
        lk = ET.SubElement(robot, "link", name=name)
        if shape is not None:
            vis = ET.SubElement(lk, "visual")
            ET.SubElement(vis, "origin", xyz=" ".join(map(str, origin)), rpy=" ".join(map(str, rpy)))
            geo = ET.SubElement(vis, "geometry")
            ET.SubElement(geo, shape[0], **{k: str(v) for k, v in shape[1].items()})
            if mat:
                ET.SubElement(vis, "material", name=mat)
        return lk

    def joint(name, jtype, parent, child, xyz=(0, 0, 0), rpy=(0, 0, 0), axis=None, limit=None):
        j = ET.SubElement(robot, "joint", name=name, type=jtype)
        ET.SubElement(j, "parent", link=parent)
        ET.SubElement(j, "child", link=child)
        ET.SubElement(j, "origin", xyz=" ".join(f"{v:.6g}" for v in xyz), rpy=" ".join(f"{v:.6g}" for v in rpy))
        if axis:
            ET.SubElement(j, "axis", xyz=" ".join(map(str, axis)))
        if limit:
            ET.SubElement(j, "limit", lower=str(limit[0]), upper=str(limit[1]), effort="100", velocity="1")

    # Planar base as x -> y -> yaw chain so web code can set it like any other joint.
    link("world")
    link("base_x_link")
    link("base_y_link")
    joint("base_x", "prismatic", "world", "base_x_link", axis=(1, 0, 0), limit=(-100, 100))
    joint("base_y", "prismatic", "base_x_link", "base_y_link", axis=(0, 1, 0), limit=(-100, 100))
    link("base_link", ("cylinder", {"radius": chassis_r, "length": g["chassis_height"]}), mat="chassis")
    joint("base_yaw", "continuous", "base_y_link", "base_link", xyz=(0, 0, chassis_z), axis=(0, 0, 1))

    for name, drive_deg in WHEELS.items():
        pos_rad = math.radians(drive_deg - 90)
        # URDF cylinders run along local z; rotate z onto the radial axle direction.
        link(f"wheel_{name}", ("cylinder", {"radius": wr, "length": g["wheel_width"]}),
             rpy=(0, math.pi / 2, 0), mat="wheel")
        joint(f"wheel_{name}_spin", "continuous", "base_link", f"wheel_{name}",
              xyz=(br * math.cos(pos_rad), br * math.sin(pos_rad), wr - chassis_z), rpy=(0, 0, pos_rad),
              axis=(1, 0, 0))

    link("column", ("box", {"size": f"{cx} {cy} {ch}"}), mat="column")
    joint("column_fixed", "fixed", "base_link", "column", xyz=(g["column_x"], 0, col_bottom + ch / 2))

    link("vertical_link", ("box", {"size": f"{sx} {sy} {sz}"}), mat="carriage")
    joint("lift", "prismatic", "base_link", "vertical_link",
          xyz=(g["column_x"] + cx / 2 + sx / 2, 0, col_bottom + g["carriage_z0"]), axis=(0, 0, 1),
          limit=(0, g["lift_travel"]))

    arm_tree = ET.parse(SO101_DIR / "so101_new_calib.urdf").getroot()
    mesh_prefix = "../arms/so101/"
    for side, sign in (("left", 1), ("right", -1)):
        for el in arm_tree:
            if el.tag == "transmission":
                continue
            el = copy.deepcopy(el)
            if el.tag in ("link", "joint"):
                el.set("name", f"{side}_{el.get('name')}")
            for ref in el.iter():
                if ref.tag in ("parent", "child"):
                    ref.set("link", f"{side}_{ref.get('link')}")
                if ref.tag == "mesh" and ref.get("filename", "").startswith("assets/"):
                    ref.set("filename", mesh_prefix + ref.get("filename"))
                if ref.tag == "material" and ref.get("name") and ref.find("color") is not None:
                    ref.set("name", f"{side}_{ref.get('name')}")
            robot.append(el)
        joint(f"{side}_arm_mount", "fixed", "vertical_link", f"{side}_base_link",
              xyz=(g["arm_mount_x"], sign * g["arm_mount_y"], sz / 2))

    ET.indent(robot)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text('<?xml version="1.0"?>\n<!-- Generated by hybrid_twin/tools/build_twin.py -->\n'
                        + ET.tostring(robot, encoding="unicode"))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-model", default="alohamini1", choices=sorted(ROBOT_SPECS))
    args = parser.parse_args()
    params = ROBOT_SPECS[args.robot_model]

    mjcf = ROOT / "mujoco" / "alohamini_twin.xml"
    urdf = ROOT / "urdf" / "alohamini_twin.urdf"
    build_mjcf(params, mjcf)
    build_urdf(params, urdf)
    print(f"[{args.robot_model}] wrote {mjcf.relative_to(ROOT)} and {urdf.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
