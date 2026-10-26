"""Narrow UBT-12HC fit-prototype cartridge for the 2x Microduck envelope.

All dimensions are mm. Importing this module does not build/export a robot.
Public API: ``shapes, report = build_slim_cartridge()`` returns P01/P02/P03 in
the v1 cartridge frame. ``to_xl_frame(shape)`` maps the provisional shaft
(0, 0, 11) to the origin and local +Y to XL330 +X.

This verifies only the constructed CAD and static bounding envelope. It does
not verify real UBT shaft/horn dimensions, assembly fit, strength or motion.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parent

import cadquery as cq


def _portable_path(path: Path) -> str:
    """Keep exported reports independent of the author's machine."""
    try:
        return path.resolve().relative_to(ROOT).as_posix()
    except ValueError:
        return path.name


@dataclass(frozen=True)
class CartridgeParameters:
    outer_width: float = 35.7
    outer_depth: float = 29.2
    outer_height: float = 50.7
    cavity_width: float = 29.7
    cavity_depth: float = 23.2
    cavity_height: float = 44.7
    split_gap: float = 0.24
    shaft_z_assumed: float = 11.0
    output_opening_radius: float = 12.0
    clamp_x: float = 12.5
    clamp_z_top: float = 25.5
    clamp_z_bottom: float = -26.5
    clamp_boss_radius: float = 4.4
    m3_clearance_radius: float = 1.65
    m3_nut_across_flats: float = 5.8
    m3_nut_depth: float = 2.8
    m3_head_radius: float = 3.0
    m3_head_depth: float = 2.4
    top_flange_width: float = 36.0
    top_flange_depth: float = 32.0
    top_flange_height: float = 4.0
    top_flange_z: float = 27.35
    yoke_floor_z: float = -34.0
    yoke_top_z: float = 27.0
    yoke_tool_access_radius: float = 1.8


def _box(x: float, y: float, z: float, center=(0.0, 0.0, 0.0)):
    return cq.Workplane("XY").box(x, y, z).val().translate(center)


def _cylinder(radius: float, length: float, center, axis=(0.0, 1.0, 0.0)):
    direction = cq.Vector(*axis).normalized()
    start = cq.Vector(*center) - direction.multiply(length / 2)
    return cq.Solid.makeCylinder(radius, length, start, direction)


def _hex_pocket(across_flats: float, depth: float, x: float, z: float,
                rear_y: float):
    # XZ cross-section; extrusion runs toward -Y from rear_y + depth.
    diameter = across_flats / math.cos(math.pi / 6)
    return (cq.Workplane("XY").polygon(6, diameter).extrude(depth).val()
            .rotate((0, 0, 0), (1, 0, 0), 90)
            .translate((x, rear_y + depth, z)))


def _bbox(shape):
    bb = shape.BoundingBox()
    return {"min": [bb.xmin, bb.ymin, bb.zmin],
            "max": [bb.xmax, bb.ymax, bb.zmax],
            "size": [bb.xlen, bb.ylen, bb.zlen]}


def _within(shape, low, high, tolerance=1e-6):
    bb = _bbox(shape)
    return all(bb["min"][i] >= low[i] - tolerance and
               bb["max"][i] <= high[i] + tolerance for i in range(3))


def to_xl_frame(shape, shaft_z: float = 11.0):
    """Rigid transform: XL xyz = (v1.y, -v1.x, v1.z - shaft_z)."""
    return (shape.translate((0, 0, -shaft_z))
            .rotate((0, 0, 0), (0, 0, 1), -90))


def build_output_yoke(p: CartridgeParameters | None = None):
    """Matched P03 in v1 local coordinates; fixed shaft and horn interfaces."""
    p = p or CartridgeParameters()
    height = p.yoke_top_z - p.yoke_floor_z
    center_z = (p.yoke_top_z + p.yoke_floor_z) / 2
    cheeks = []
    for y in (-22.0, 22.0):
        cheek = _box(30.0, 4.0, height, (0, y, center_z))
        # Keep analytic square plate edges. A cosmetic 1.2 mm fillet creates
        # tiny sliver faces where the upper Allen-key opening meets the edge.
        # Preserve the v1 side service relief, passive bearing and drive opening.
        cheek = cheek.cut(_box(14, 10, 16, (0, y, -13)))
        cheek = cheek.cut(_cylinder(5.075 if y < 0 else 12.25, 10,
                                   (0, y, p.shaft_z_assumed)))
        if y > 0:
            for x in (-10.0, 10.0):
                for z in (1.0, 21.0):
                    cheek = cheek.cut(_cylinder(1.65, 10, (x, y, z)))
        cheeks.append(cheek)
    yoke = cheeks[0].fuse(cheeks[1]).fuse(_box(30, 48, 4, (0, 0, p.yoke_floor_z)))
    # Lower attachment XY pattern is unchanged; only its Z follows the floor.
    for x in (-10.0, 10.0):
        for y in (-14.0, 14.0):
            yoke = yoke.cut(_cylinder(1.65, 12, (x, y, p.yoke_floor_z), (0, 0, 1)))
    # Straight 2.5 mm Allen-key access to new cartridge clamp screws. Upper
    # holes open into the top edge; these are not structural horn bolt holes.
    for x in (-p.clamp_x, p.clamp_x):
        for z in (p.clamp_z_bottom, p.clamp_z_top):
            yoke = yoke.cut(_cylinder(p.yoke_tool_access_radius, 60, (x, 0, z)))
    return yoke.clean()


def build_slim_cartridge(parameters: CartridgeParameters | None = None):
    """Return P01/P02/P03 shapes and a JSON-serializable geometric report."""
    p = parameters or CartridgeParameters()
    outer = _box(p.outer_width, p.outer_depth, p.outer_height)
    # Cosmetic fillets are confined to the initial housing; screw bosses retain
    # their analytic cylinders and explicitly checked ligament dimensions.
    outer = cq.Workplane(obj=outer).edges().fillet(1.5).val()
    full = outer
    clamp_centers = [(x, 0.0, z) for x in (-p.clamp_x, p.clamp_x)
                     for z in (p.clamp_z_bottom, p.clamp_z_top)]
    for point in clamp_centers:
        full = full.fuse(_cylinder(p.clamp_boss_radius, p.outer_depth, point))
    full = full.fuse(_box(p.top_flange_width, p.top_flange_depth,
                          p.top_flange_height, (0, 0, p.top_flange_z)))

    # Rear M5 passive-axle seat, original v1 datum and recess retained.
    full = full.fuse(_cylinder(8.0, 3.0, (0, -13.1, p.shaft_z_assumed)))
    full = full.fuse(_cylinder(7.5, 5.0, (0, -17.0, p.shaft_z_assumed)))
    cavity = _box(p.cavity_width, p.cavity_depth, p.cavity_height)
    # Cut after adding bosses, so no boss can invade the motor cavity.
    full = full.cut(cavity)

    output_tool = _cylinder(p.output_opening_radius, 20.0,
                            (0, 21.0, p.shaft_z_assumed))
    passive_tool = _cylinder(2.65, 12.0, (0, -15.0, p.shaft_z_assumed))
    nut5_tool = (cq.Workplane("XY").polygon(6, 9.5).extrude(4.0).val()
                 .rotate((0, 0, 0), (1, 0, 0), 90)
                 .translate((0, -15.5, p.shaft_z_assumed)))
    connector_tool = _box(14.0, 40.0, 8.0, (0, 0, -20.0))
    for tool in (output_tool, passive_tool, nut5_tool, connector_tool):
        full = full.cut(tool)

    screw_tools = []
    screw_records = []
    for point in clamp_centers:
        x, _, z = point
        through = _cylinder(p.m3_clearance_radius, 60.0, point)
        # Rear nut recess extends 0.05 mm outside the back surface to guarantee
        # an open insertion path; it does not enter the nominal motor cavity.
        nut = _hex_pocket(p.m3_nut_across_flats, p.m3_nut_depth + 0.05,
                          x, z, -p.outer_depth / 2 - 0.05)
        head = _cylinder(p.m3_head_radius, p.m3_head_depth + 0.05,
                          (x, p.outer_depth / 2 - p.m3_head_depth / 2 + 0.025, z))
        for tool in (through, nut, head):
            full = full.cut(tool)
        screw_tools.append(through)
        screw_records.append({
            "axis_point_v1_mm": list(point), "axis_direction_v1": [0, 1, 0],
            "axis_point_xl_mm": [0, -x, z - p.shaft_z_assumed],
            "axis_direction_xl": [1, 0, 0],
            "front_head_seat_y_mm": p.outer_depth / 2 - p.m3_head_depth,
            "rear_nut_opening_y_mm": -p.outer_depth / 2,
            "bore_radius_mm": p.m3_clearance_radius,
            "bore_to_cavity_ligament_z_mm": abs(z) - p.cavity_height / 2 - p.m3_clearance_radius,
            "bore_intersection_with_cavity_mm3": through.intersect(cavity).Volume(),
            "nut_intersection_with_cavity_mm3": nut.intersect(cavity).Volume(),
            "head_intersection_with_cavity_mm3": head.intersect(cavity).Volume(),
        })
    full = full.clean()

    # Symmetric 0.24 mm split. Full opening is along ±Y; rear shaft datum is
    # preserved. Opening direction itself is geometric, not a real fit claim.
    back = full.intersect(_box(100, 100, 140,
                               (0, -50 - p.split_gap / 2, -4))).clean()
    front = full.intersect(_box(100, 100, 140,
                                (0, 50 + p.split_gap / 2, -4))).clean()
    yoke = build_output_yoke(p)
    shapes = {"P01_cage_back": back, "P02_cage_front": front, "P03_output_yoke": yoke}
    v1_limits = {"min": [-20, -29, -38], "max": [20, 29, 30]}
    xl_limits = {"min": [-29, -20, -49], "max": [29, 20, 19]}
    part_report = {}
    for name, shape in shapes.items():
        xl = to_xl_frame(shape, p.shaft_z_assumed)
        part_report[name] = {
            "brep_valid": bool(shape.isValid()), "solid_count": len(shape.Solids()),
            "volume_mm3": shape.Volume(), "bbox_v1_mm": _bbox(shape),
            "bbox_xl_mm": _bbox(xl),
            "within_v1_target_envelope": _within(shape, **{
                "low": v1_limits["min"], "high": v1_limits["max"]}),
            "within_xl_2x_target_envelope": _within(xl, **{
                "low": xl_limits["min"], "high": xl_limits["max"]}),
            "motor_cavity_intersection_mm3": shape.intersect(cavity).Volume(),
            "clamp_bore_obstruction_mm3": [shape.intersect(t).Volume() for t in screw_tools],
        }
    checks = {
        "all_valid_single_solids": all(v["brep_valid"] and v["solid_count"] == 1 for v in part_report.values()),
        "all_inside_target_envelope": all(v["within_v1_target_envelope"] and v["within_xl_2x_target_envelope"] for v in part_report.values()),
        "cavity_empty": all(abs(v["motor_cavity_intersection_mm3"]) < 1e-6 for v in part_report.values()),
        "clamp_bores_clear": all(abs(x) < 1e-6 for v in part_report.values() for x in v["clamp_bore_obstruction_mm3"]),
        "clamp_hardware_clear_of_cavity": all(abs(r[k]) < 1e-6 for r in screw_records for k in (
            "bore_intersection_with_cavity_mm3", "nut_intersection_with_cavity_mm3", "head_intersection_with_cavity_mm3")),
        "halves_do_not_overlap": abs(back.intersect(front).Volume()) < 1e-6,
        "housing_yoke_static_intersection_zero": all(abs(s.intersect(yoke).Volume()) < 1e-6 for s in (back, front)),
    }
    report = {
        "revision": "v2 slim cartridge fit prototype", "units": "mm",
        "parameters": asdict(p), "parts": part_report, "clamp_screws": screw_records,
        "clamp_holes_v1_mm": [list(point) for point in clamp_centers],
        "transform_v1_to_xl_row_major": [[0, 1, 0, 0], [-1, 0, 0, 0],
                                          [0, 0, 1, -p.shaft_z_assumed], [0, 0, 0, 1]],
        "limits_v1_mm": v1_limits, "limits_xl_mm": xl_limits,
        "top_flange": {"size_mm": [p.top_flange_width, p.top_flange_depth, p.top_flange_height],
                       "center_v1_mm": [0, 0, p.top_flange_z],
                       "mounting_holes": "None: solid pad reserved for the actual v2 interconnect pattern; v1 ±20 mm holes intentionally removed."},
        "rear_m5": {"axis_point_v1_mm": [0, 0, p.shaft_z_assumed],
                    "bore_diameter_mm": 5.3, "nut_hex_circumdiameter_mm": 9.5,
                    "nut_pocket_depth_mm": 4.0, "nut_pocket_y_range_mm": [-19.5, -15.5]},
        "connector_relief": {"size_mm": [14, 40, 8], "center_v1_mm": [0, 0, -20],
                             "status": "Provisional service opening; real plug not measured."},
        "matched_P03": {
            "floor_center_z_mm": p.yoke_floor_z,
            "floor_size_mm": [30, 48, 4],
            "cheek_z_range_mm": [p.yoke_floor_z, p.yoke_top_z],
            "shaft_center_v1_mm": [0, 0, p.shaft_z_assumed],
            "passive_bearing_hole_diameter_mm": 10.15,
            "drive_opening_diameter_mm": 24.5,
            "drive_bolt_centers_v1_mm": [[x, 22, z] for x in (-10, 10) for z in (1, 21)],
            "floor_bolt_centers_v1_mm": [[x, y, p.yoke_floor_z] for x in (-10, 10) for y in (-14, 14)],
            "clamp_tool_access_diameter_mm": 2 * p.yoke_tool_access_radius,
            "clamp_tool_access_note": "Along Y through both cheeks; upper openings meet the top edge. Intended for 2.5 mm Allen key. Hardware/adjacent-part tool access still requires assembly review.",
            "housing_static_intersection_mm3": {"P01_cage_back": back.intersect(yoke).Volume(),
                                                 "P02_cage_front": front.intersect(yoke).Volume()},
            "floor_to_lower_boss_nominal_z_gap_mm": p.clamp_z_bottom - p.clamp_boss_radius - (p.yoke_floor_z + 2),
            "status": "Static modeled pose only; no full-range motion or physical fit verification.",
        },
        "checks": checks,
        "unverified": ["real UBT shaft datum and orientation", "real horn and plug geometry",
                       "physical assembly fit", "tool and screw access with installed adjacent parts",
                       "full-range swept interference", "strength, friction and fatigue", "walking"],
    }
    if not all(checks.values()):
        raise AssertionError(json.dumps(report, indent=2))
    return shapes, report


def export_cartridge(output_dir: Path | str | None = None,
                     parameters: CartridgeParameters | None = None):
    """Export STEP/STL in v1 and XL frames, roundtrip-validate, then write JSON."""
    import trimesh
    output = Path(output_dir) if output_dir is not None else ROOT / "cad" / "slim_cartridge"
    shapes, report = build_slim_cartridge(parameters)
    report["exports"] = {}
    for frame in ("v1", "xl"):
        for kind in ("step", "stl"):
            (output / frame / kind).mkdir(parents=True, exist_ok=True)
        for name, v1_shape in shapes.items():
            shape = v1_shape if frame == "v1" else to_xl_frame(
                v1_shape, report["parameters"]["shaft_z_assumed"])
            step_path = output / frame / "step" / f"{name}.step"
            stl_path = output / frame / "stl" / f"{name}.stl"
            cq.exporters.export(shape, str(step_path))
            cq.exporters.export(shape, str(stl_path), tolerance=0.06, angularTolerance=0.12)
            imported = cq.importers.importStep(str(step_path)).val()
            mesh = trimesh.load(str(stl_path), force="mesh", process=True)
            record = {"step": _portable_path(step_path), "stl": _portable_path(stl_path),
                      "step_reimport_valid": bool(imported.isValid()),
                      "step_reimport_solid_count": len(imported.Solids()),
                      "step_volume_error_mm3": abs(imported.Volume() - shape.Volume()),
                      "stl_watertight": bool(mesh.is_watertight),
                      "stl_winding_consistent": bool(mesh.is_winding_consistent),
                      "stl_positive_volume": bool(mesh.volume > 0),
                      "stl_connected_components": len(mesh.split(only_watertight=False)),
                      "stl_triangles": len(mesh.faces)}
            assert record["step_reimport_valid"] and record["step_reimport_solid_count"] == 1
            assert record["step_volume_error_mm3"] < 1e-3
            assert record["stl_watertight"] and record["stl_winding_consistent"] and record["stl_positive_volume"]
            assert record["stl_connected_components"] == 1
            report["exports"][f"{frame}/{name}"] = record
    legacy_yoke_path = ROOT / "source" / "cartridge-v1" / "P03_output_yoke.step"
    if legacy_yoke_path.exists():
        legacy_yoke = cq.importers.importStep(str(legacy_yoke_path)).val()
        intersections = {}
        for name, shape in shapes.items():
            if name not in ("P01_cage_back", "P02_cage_front"):
                continue
            intersection = shape.intersect(legacy_yoke)
            intersections[name] = {
                "volume_mm3": intersection.Volume(),
                "bbox_v1_mm": _bbox(intersection) if intersection.Volume() > 1e-6 else None,
            }
        report["legacy_v1_P03_compatibility"] = {
            "reference_step": _portable_path(legacy_yoke_path),
            "scope": "Actual CAD Boolean intersection in the same nominal v1 local pose; no motion or real-hardware fit test.",
            "intersections": intersections,
            "may_reuse_unchanged": all(v["volume_mm3"] < 1e-6 for v in intersections.values()),
            "required_change": "Old P03 floor occupies Z[-31.2,-27.2] and intersects the new lower clamp bosses. The separately exported v2 P03 moves its floor center to -34.0 and extends both cheeks. The original v1 file remains unchanged; see matched_P03 for the new static check.",
        }
    report_path = output / "validation.json"
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    if output_dir is None:
        # Assembly integration uses the original v1 frame, never the optional
        # positive-X reference transform exported under slim_cartridge/xl/.
        for kind, folder in (("stl", "adapter-stl"), ("step", "adapter-step")):
            destination = ROOT / "cad" / folder
            destination.mkdir(parents=True, exist_ok=True)
            for name in shapes:
                shutil.copy2(output / "v1" / kind / f"{name}.{kind}",
                             destination / f"{name}.{kind}")
        compatibility_report = dict(report)
        compatibility_report["assembly_export_frame"] = "original v1 local frame; no XL rotation applied"
        compatibility_report["assembly_stl_dir"] = (ROOT / "cad" / "adapter-stl").relative_to(ROOT).as_posix()
        compatibility_report["assembly_step_dir"] = (ROOT / "cad" / "adapter-step").relative_to(ROOT).as_posix()
        (ROOT / "slim-cartridge-report.json").write_text(
            json.dumps(compatibility_report, ensure_ascii=False, indent=2), encoding="utf-8")
    return shapes, report, report_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    _, report, path = export_cartridge(args.output_dir)
    print(json.dumps({"validation_json": _portable_path(path), "checks": report["checks"],
                      "parts": {k: {"bbox_v1_mm": v["bbox_v1_mm"], "volume_mm3": v["volume_mm3"]}
                                for k, v in report["parts"].items()}}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
