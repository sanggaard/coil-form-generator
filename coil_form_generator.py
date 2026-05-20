import argparse
import numpy as np
import manifold3d as m3d


def _helix_groove(pitch, num_windings, radius, groove_radius, segments_per_turn=64):
    n = int(num_windings * segments_per_turn)
    spheres = []
    for i in range(n + 1):
        t = i / segments_per_turn
        # -π/2 offset: helix starts and ends at the bottom (y = -radius)
        angle = t * 2 * np.pi - np.pi / 2
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        z = t * pitch
        spheres.append(m3d.Manifold.sphere(groove_radius, 16).translate((x, y, z)))

    segments = [m3d.Manifold.batch_hull([spheres[i], spheres[i + 1]]) for i in range(n)]
    return m3d.Manifold.batch_boolean(segments, m3d.OpType.Add)


def build_permanent_form(coil_diameter, num_windings, wire_diameter, winding_distance,
                         inner_diameter=None, wire_hole_diameter=None,
                         plate_thickness=None, pcb_clearance=None):
    pitch = wire_diameter + winding_distance
    total_length = num_windings * pitch
    form_radius = (coil_diameter - wire_diameter) / 2

    flange_thickness = plate_thickness if plate_thickness is not None else max(wire_diameter * 2, 1.5)
    pcb_gap = pcb_clearance if pcb_clearance is not None else wire_diameter * 2
    flange_radius = form_radius + pcb_gap
    groove_radius = wire_diameter / 2 * 0.95
    hole_radius = (wire_hole_diameter / 2) if wire_hole_diameter is not None else wire_diameter * 0.8

    foot_width = flange_radius * 2

    # Body with helical groove (z=0 to z=total_length)
    body = m3d.Manifold.cylinder(total_length, form_radius)
    groove = _helix_groove(pitch, num_windings, form_radius, groove_radius)
    body = body - groove

    # End plate cross-section: upper semicircle + lower rectangle.
    # The semicircle (y >= 0) contains the windings; the rectangle (y <= 0)
    # is the foot with a flat bottom face at y = -flange_radius for PCB mounting.
    # Plate is rebuilt per end — trim_by_plane result cannot be translated twice.
    def _make_plate():
        upper_semi = (
            m3d.Manifold.cylinder(flange_thickness, flange_radius)
            .trim_by_plane((0, 1, 0), 0)
        )
        lower_rect = (
            m3d.Manifold.cube((foot_width, flange_radius, flange_thickness), center=False)
            .translate((-foot_width / 2, -flange_radius, 0))
        )
        return upper_semi + lower_rect

    y_hole_height = flange_radius - form_radius + hole_radius + 1

    for z_plate_start, z_inner_face, inner_dir in [
        (-flange_thickness, 0, -1),        # left plate: z=-flange_thickness to z=0
        (total_length, total_length, +1),  # right plate: z=total_length to z=total_length+flange_thickness
    ]:
        plate = _make_plate().translate((0, 0, z_plate_start))

        # Z-hole: wire exits the coil groove through the end plate
        z_hole = (
            m3d.Manifold.cylinder(flange_thickness + 2, hole_radius)
            .translate((0, -form_radius, z_plate_start - 1))
        )

        # Y-hole: just inside the inner face, wire drops straight to PCB
        z_y = z_inner_face + inner_dir * hole_radius
        y_hole = (
            m3d.Manifold.cylinder(y_hole_height, hole_radius)
            .rotate((90, 0, 0))
            .translate((0, -flange_radius - 1, z_y))
        )

        body = body + (plate - z_hole - y_hole)

    # Hollow centre through the full form including end plates
    if inner_diameter is not None:
        inner_radius = inner_diameter / 2
    else:
        inner_radius = form_radius - 2.0  # 2 mm wall thickness
    if inner_radius > 0.5:
        full_length = total_length + 2 * flange_thickness + 2
        body = body - m3d.Manifold.cylinder(full_length, inner_radius).translate(
            (0, 0, -flange_thickness - 1)
        )

    return body


def build_removable_form(coil_diameter, num_windings, wire_diameter, winding_distance,
                         inner_diameter=None):
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

    # Hollow centre
    if inner_diameter is not None:
        inner_radius = inner_diameter / 2
    else:
        inner_radius = form_radius - 2.0  # 2 mm wall thickness
    if inner_radius > 0.5:
        body = body - m3d.Manifold.cylinder(body_length + 2, inner_radius).translate((0, 0, -1))

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
        description=(
            "Generate a 3D-printable coil form (.stl) compatible with Bambu Studio.\n\n"
            "Two form types are supported:\n"
            "  Removable  (default)  Grooved mandrel. Wind the coil on it, then unscrew\n"
            "                        the coil off the rod. No PCB mounting features.\n"
            "  Permanent  (--permanent)  Hollow tube with helical groove, end plates and\n"
            "                        flat feet. Mounts directly on a PCB. Wire leads exit\n"
            "                        through holes in the end plates and feet."
        ),
        epilog=(
            "Diameter convention:\n"
            "  --diameter is the MEAN coil diameter (center-to-center of wire across the\n"
            "  coil), matching standard coil calculators (Wheeler formula etc.).\n"
            "  Form body OD  = diameter - wire\n"
            "  Pitch per winding = wire + gap\n\n"
            "Advanced parameters (all optional, shown with their defaults):\n"
            "  --coil-hole      default: form body OD - 2×2 mm  (2 mm wall thickness)\n"
            "  --wire-hole      default: 1.6 × wire diameter     (permanent only)\n"
            "  --plate-thickness default: max(2 × wire, 1.5 mm) (permanent only)\n"
            "  --pcb-clearance  default: 2 × wire diameter       (permanent only)\n\n"
            "Examples:\n"
            "  %(prog)s -d 10 -n 10 -w 0.5 -g 0.1\n"
            "  %(prog)s -d 10 -n 6  -w 1.3 -g 0.5 --permanent\n"
            "  %(prog)s -d 10 -n 6  -w 1.3 -g 0.5 -p --pcb-clearance 3 --plate-thickness 2\n"
            "  %(prog)s -d 8  -n 20 -w 0.3 -g 0.05 -p -o my_coil.stl"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    req = parser.add_argument_group("required arguments")
    req.add_argument("-d", "--diameter", type=float, required=True, metavar="MM",
                     help="Mean coil diameter in mm (center-to-center of wire)")
    req.add_argument("-n", "--windings", type=int, required=True, metavar="N",
                     help="Number of windings")
    req.add_argument("-w", "--wire", type=float, required=True, metavar="MM",
                     help="Wire diameter in mm")
    req.add_argument("-g", "--gap", type=float, required=True, metavar="MM",
                     help="Edge-to-edge gap between adjacent windings in mm")

    parser.add_argument("-p", "--permanent", action="store_true",
                        help="Generate a permanent PCB-mount form (default: removable mandrel)")
    parser.add_argument("-o", "--output", type=str, default=None, metavar="FILE",
                        help="Output STL file path (default: auto-named from parameters)")

    adv = parser.add_argument_group("advanced layout overrides")
    adv.add_argument("--coil-hole", type=float, default=None, metavar="MM",
                     help="Center hole diameter in mm (default: 2 mm wall thickness)")
    adv.add_argument("--wire-hole", type=float, default=None, metavar="MM",
                     help="Wire guidance hole diameter in mm (default: 1.6 × wire, permanent only)")
    adv.add_argument("--plate-thickness", type=float, default=None, metavar="MM",
                     help="End plate thickness in mm (default: max(2 × wire, 1.5 mm), permanent only)")
    adv.add_argument("--pcb-clearance", type=float, default=None, metavar="MM",
                     help="Distance from PCB surface to coil wire centre in mm "
                          "(default: 2 × wire diameter, permanent only)")

    args = parser.parse_args()

    # High-quality circular segments: 2° max angle, 0.2 mm max edge length
    m3d.set_min_circular_angle(2)
    m3d.set_min_circular_edge_length(0.2)

    form_type = "permanent" if args.permanent else "removable"
    if args.output is None:
        args.output = (
            f"coil_form_d{args.diameter}_n{args.windings}"
            f"_w{args.wire}_{form_type}.stl"
        )

    pitch = args.wire + args.gap
    form_radius = (args.diameter - args.wire) / 2

    if form_radius <= 0:
        raise ValueError("--diameter must be greater than --wire")

    # Resolve and display effective parameters
    inner_r = (args.coil_hole / 2) if args.coil_hole is not None else (form_radius - 2.0)
    plate_t = args.plate_thickness if args.plate_thickness is not None else max(args.wire * 2, 1.5)
    pcb_gap = args.pcb_clearance if args.pcb_clearance is not None else args.wire * 2
    wire_h  = args.wire_hole if args.wire_hole is not None else args.wire * 1.6

    print(f"Generating {form_type} coil form...")
    print(f"  Mean diameter   : {args.diameter} mm")
    print(f"  Windings        : {args.windings}")
    print(f"  Wire diameter   : {args.wire} mm")
    print(f"  Gap             : {args.gap} mm")
    print(f"  Pitch           : {pitch:.3f} mm")
    print(f"  Form body OD    : {form_radius * 2:.3f} mm")
    print(f"  Total length    : {args.windings * pitch:.3f} mm")
    print(f"  Centre hole dia : {inner_r * 2:.3f} mm" if inner_r > 0.5 else "  Centre hole     : none (wall too thin)")
    if args.permanent:
        print(f"  Plate thickness : {plate_t:.3f} mm")
        print(f"  PCB clearance   : {pcb_gap:.3f} mm")
        print(f"  Wire hole dia   : {wire_h:.3f} mm")

    if args.permanent:
        result = build_permanent_form(
            args.diameter, args.windings, args.wire, args.gap,
            inner_diameter=args.coil_hole,
            wire_hole_diameter=args.wire_hole,
            plate_thickness=args.plate_thickness,
            pcb_clearance=args.pcb_clearance,
        )
    else:
        result = build_removable_form(
            args.diameter, args.windings, args.wire, args.gap,
            inner_diameter=args.coil_hole,
        )

    _write_stl(result, args.output)
    print(f"Saved: {args.output}")


if __name__ == "__main__":
    main()
