# Coil Form Generator

Python CLI tool that generates 3D-printable STL coil forms from coil parameters.
Output is compatible with Bambu Studio (binary STL).

## Setup

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## Running

```bash
.venv/bin/python coil_form_generator.py --help
.venv/bin/python coil_form_generator.py -d 10 -n 10 -w 0.5 -g 0.1
.venv/bin/python coil_form_generator.py -d 10 -n 6 -w 1.3 -g 0.5 --permanent
```

## Files

- `coil_form_generator.py` — single-file implementation, all geometry and CLI
- `requirements.txt` — `manifold3d`, `numpy`
- `*.stl` — generated output (gitignored)

## Library

Uses **manifold3d** (not CadQuery) because CadQuery's OCP bindings have no wheels for Python 3.14.
manifold3d is a fast CSG library. Boolean ops: `a + b` (union), `a - b` (difference), `a ^ b` (intersection).
STL is written manually via numpy (no extra dependency needed).

## Diameter Convention

`--diameter` is the **mean coil diameter** — center-to-center of wire across the coil,
matching standard coil calculators (Wheeler formula etc.).

- Form body OD = `diameter - wire`
- Form body radius = `form_radius = (diameter - wire) / 2`
- Outer coil radius (top of wire) = `form_radius + wire_diameter`
- Pitch = `wire + gap`
- Total winding length = `windings × pitch`

## Coordinate System

- Coil axis along **Z** (z=0 to z=total\_length for the body)
- **Y** is vertical relative to the PCB: negative Y is downward (toward PCB)
- **X** is lateral
- The helix starts and ends at the bottom of the form: (x=0, y=−form\_radius) — angle offset −π/2

## Form Types

### Removable (default)

Temporary winding mandrel. Wind the coil, then unscrew it off the rod.

- Solid cylinder with helical groove
- Body length = `total_length + 2 × pitch` (extra at each end for grip)
- Groove slightly wider than the wire (`groove_radius = wire × 0.55`) for easy removal
- Hollow centre (2 mm wall) if `form_radius > 2.5 mm`

### Permanent (`--permanent`)

Stays inside the finished coil; mounts on a PCB.

- Hollow cylindrical body with helical groove (`groove_radius = wire/2 × 0.95`, snug fit)
- Two **end plates** at z=−flange\_thickness and z=total\_length, each shaped as:
  - **Upper semicircle** (y ≥ 0): radius = `form_radius + wire + flange_extension`, contains windings
  - **Lower rectangle** (y ≤ 0): width = semicircle diameter, height = `form_radius + pcb_clearance`, flat bottom face rests on PCB
- Each end plate has two wire-routing holes forming an L-channel:
  - **Z-hole** at (x=0, y=−form\_radius): wire exits the groove through the plate (along coil axis)
  - **Y-hole** just inside the inner face: wire drops straight down to the PCB through-hole
- Hollow centre runs through the full form including end plates

## Key Defaults (Permanent Form)

| Parameter | Default |
|---|---|
| Plate thickness | `max(wire × 2, 1.5 mm)` |
| Flange semicircle extension | `wire × 1` beyond outer coil edge |
| PCB clearance (rect height) | `wire × 2` |
| Wire hole diameter | `wire × 1.6` |
| Centre hole | `form_radius − 2.0 mm` inner radius (2 mm wall) |

## Known Gotchas

- **`trim_by_plane` cannot be translated twice** — a Manifold produced by `trim_by_plane` segfaults if `.translate()` is called on it more than once. Fix: recreate the plate with `_make_plate()` per end rather than reusing a translated template.
- **Python 3.14**: CadQuery/OCP has no wheels for 3.14. Use manifold3d instead.
- **Resolution**: call `m3d.set_min_circular_angle(2)` and `m3d.set_min_circular_edge_length(0.2)` before generating or small holes will look square (default gives only 4 segments for a 0.4 mm radius hole).
- **Helix groove** is approximated by sphere-hull segments (64 per turn, sphere resolution 16). The helix starts at angle −π/2 so both ends land at the bottom (y = −form\_radius).

## Advanced CLI Parameters

All optional. Permanent form only unless noted.

| Flag | Controls | Default |
|---|---|---|
| `--coil-hole MM` | Centre hole diameter (both forms) | 2 mm wall thickness |
| `--wire-hole MM` | Wire guidance hole diameter | 1.6 × wire |
| `--plate-thickness MM` | End plate thickness | max(2 × wire, 1.5 mm) |
| `--flange-extension MM` | Semicircle radius beyond outer coil edge | 1 × wire |
| `--pcb-clearance MM` | Rectangle height (PCB surface to coil body bottom) | 2 × wire |
| `-o / --output FILE` | Output STL path | auto-named from parameters |
