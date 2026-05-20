import argparse
import numpy as np
import manifold3d as m3d


def _helix_groove(pitch, num_windings, radius, groove_radius, segments_per_turn=48):
    n = int(num_windings * segments_per_turn)
    spheres = []
    for i in range(n + 1):
        t = i / segments_per_turn
        angle = t * 2 * np.pi
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        z = t * pitch
        spheres.append(m3d.Manifold.sphere(groove_radius, 8).translate((x, y, z)))

    segments = [m3d.Manifold.batch_hull([spheres[i], spheres[i + 1]]) for i in range(n)]
    return m3d.Manifold.batch_boolean(segments, m3d.OpType.Add)


def build_permanent_form(coil_diameter, num_windings, wire_diameter, winding_distance):
    pitch = wire_diameter + winding_distance
    total_length = num_windings * pitch
    form_radius = (coil_diameter - wire_diameter) / 2

    flange_thickness = wire_diameter * 1.5
    flange_radius = form_radius + wire_diameter * 2
    groove_radius = wire_diameter / 2 * 0.95

    # Flat feet outside the helix area (attached to outer faces of flanges)
    foot_thickness = max(wire_diameter * 3, 2.0)  # thin flat pad
    foot_length = max(wire_diameter * 8, 4.0)      # extends outward from flange in Z
    foot_width = flange_radius * 2                 # full flange diameter for stability
    hole_radius = wire_diameter * 0.8              # snug wire guidance hole

    # Body with helical groove (z=0 to z=total_length)
    body = m3d.Manifold.cylinder(total_length, form_radius)
    groove = _helix_groove(pitch, num_windings, form_radius, groove_radius)
    body = body - groove

    # Flanges at each end
    left_flange = m3d.Manifold.cylinder(flange_thickness, flange_radius).translate(
        (0, 0, -flange_thickness)
    )
    right_flange = m3d.Manifold.cylinder(flange_thickness, flange_radius).translate(
        (0, 0, total_length)
    )
    body = body + left_flange + right_flange

    # Flat feet: one per end, attached to outer face of each flange, outside the helix area.
    # Each foot is a flat pad at the bottom (y = -flange_radius) with a vertical wire hole.
    for z_inner, direction in [(-flange_thickness, -1), (total_length + flange_thickness, +1)]:
        z_center = z_inner + direction * foot_length / 2
        # Flat pad sitting at the bottom of the flange
        foot = (
            m3d.Manifold.cube((foot_width, foot_thickness, foot_length), center=True)
            .translate((0, -(flange_radius - foot_thickness / 2), z_center))
        )
        # Vertical wire guidance hole (Y axis) near the inner edge of the foot
        hole_z = z_inner + direction * foot_length * 0.25
        wire_hole = (
            m3d.Manifold.cylinder(foot_thickness + 2, hole_radius, center=True)
            .rotate((90, 0, 0))
            .translate((0, -(flange_radius - foot_thickness / 2), hole_z))
        )
        body = body + (foot - wire_hole)

    return body


def build_removable_form(coil_diameter, num_windings, wire_diameter, winding_distance):
    pitch = wire_diameter + winding_distance
    total_length = num_windings * pitch
    form_radius = (coil_diameter - wire_diameter) / 2

    # Extra length at each end for handling ease
    body_length = total_length + 2 * pitch
    num_windings_total = num_windings + 2
    groove_radius = wire_diameter * 0.55  # slight clearance so coil unscrews off

    body = m3d.Manifold.cylinder(body_length, form_radius)
    groove = _helix_groove(pitch, num_windings_total, form_radius, groove_radius)
    body = body - groove

    return body


def _write_stl(manifold, filename):
    mesh = manifold.to_mesh()
    verts = np.array(mesh.vert_properties, dtype=np.float32)[:, :3]
    tris = np.array(mesh.tri_verts, dtype=np.int64)

    v0, v1, v2 = verts[tris[:, 0]], verts[tris[:, 1]], verts[tris[:, 2]]
    normals = np.cross(v1 - v0, v2 - v0).astype(np.float32)
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    normals /= np.where(norms == 0, 1.0, norms)

    dtype = np.dtype([
        ("normal", np.float32, (3,)),
        ("v0", np.float32, (3,)),
        ("v1", np.float32, (3,)),
        ("v2", np.float32, (3,)),
        ("attr", np.uint16),
    ])
    data = np.zeros(len(tris), dtype=dtype)
    data["normal"] = normals
    data["v0"] = v0
    data["v1"] = v1
    data["v2"] = v2

    with open(filename, "wb") as f:
        f.write(b"\0" * 80)
        f.write(np.uint32(len(tris)).tobytes())
        f.write(data.tobytes())


def main():
    parser = argparse.ArgumentParser(
        description="Generate a 3D-printable coil form (.stl) for Bambu Studio"
    )
    parser.add_argument("--diameter", "-d", type=float, required=True,
                        help="Mean coil diameter in mm (center-to-center of wire across coil)")
    parser.add_argument("--windings", "-n", type=int, required=True,
                        help="Number of windings")
    parser.add_argument("--wire", "-w", type=float, required=True,
                        help="Wire diameter in mm")
    parser.add_argument("--gap", "-g", type=float, required=True,
                        help="Edge-to-edge gap between windings in mm")
    parser.add_argument("--permanent", "-p", action="store_true",
                        help="Permanent form with PCB feet (default: removable mandrel)")
    parser.add_argument("--output", "-o", type=str, default=None,
                        help="Output STL file path")
    args = parser.parse_args()

    form_type = "permanent" if args.permanent else "removable"
    if args.output is None:
        args.output = (
            f"coil_form_d{args.diameter}_n{args.windings}"
            f"_w{args.wire}_{form_type}.stl"
        )

    pitch = args.wire + args.gap
    form_radius = (args.diameter - args.wire) / 2

    if form_radius <= 0:
        raise ValueError("coil_diameter must be greater than wire_diameter")

    print(f"Generating {form_type} coil form...")
    print(f"  Mean diameter : {args.diameter} mm")
    print(f"  Windings      : {args.windings}")
    print(f"  Wire diameter : {args.wire} mm")
    print(f"  Gap           : {args.gap} mm")
    print(f"  Pitch         : {pitch:.3f} mm")
    print(f"  Form body OD  : {form_radius * 2:.3f} mm")
    print(f"  Total length  : {args.windings * pitch:.3f} mm")

    if args.permanent:
        result = build_permanent_form(args.diameter, args.windings, args.wire, args.gap)
    else:
        result = build_removable_form(args.diameter, args.windings, args.wire, args.gap)

    _write_stl(result, args.output)
    print(f"Saved: {args.output}")


if __name__ == "__main__":
    main()
