#!/usr/bin/env python3
"""
Coil calculator — find single-layer air-core coil configurations for a target inductance.
Outputs Q, DC/AC resistance, impedance, and ready-to-run coil_form_generator.py commands.

Wheeler (1928) formula used throughout:
    L [µH] = r² × n² / (25.4 × (9r + 10l))    r, l in mm
"""

import argparse
import math
import sys

RHO_COPPER = 1.68e-8  # Ω·m at 20 °C


# ── Physics ──────────────────────────────────────────────────────────────────

def wheeler_inductance_uh(radius_mm, n, pitch_mm):
    """Single-layer solenoid inductance via Wheeler (1928). Returns µH."""
    r, l = radius_mm, n * pitch_mm
    denom = 25.4 * (9 * r + 10 * l)
    return r**2 * n**2 / denom if denom else 0.0


def solve_turns(target_uh, radius_mm, pitch_mm):
    """Invert Wheeler formula: solve for n (float). Returns None if no solution."""
    r, p, L = radius_mm, pitch_mm, target_uh
    # r²n² − (L·25.4·10p)n − (L·25.4·9r) = 0
    a = r**2
    b = -(L * 25.4 * 10 * p)
    c = -(L * 25.4 * 9 * r)
    disc = b**2 - 4 * a * c
    if disc < 0:
        return None
    return (-b + math.sqrt(disc)) / (2 * a)


def dc_resistance_ohm(radius_mm, n, wire_diameter_mm):
    """DC winding resistance in Ω."""
    l_wire = n * math.pi * 2 * radius_mm * 1e-3          # m
    a_wire = math.pi * (wire_diameter_mm * 0.5e-3) ** 2  # m²
    return RHO_COPPER * l_wire / a_wire


def skin_depth_mm(frequency_hz):
    """Copper skin depth at given frequency, in mm."""
    if frequency_hz <= 0:
        return float("inf")
    mu0 = 4 * math.pi * 1e-7
    return math.sqrt(RHO_COPPER / (math.pi * frequency_hz * mu0)) * 1e3


def ac_resistance_factor(wire_diameter_mm, frequency_hz):
    """Approximate R_ac / R_dc ratio from skin effect (simplified)."""
    if frequency_hz <= 0:
        return 1.0
    delta = skin_depth_mm(frequency_hz)
    r_wire = wire_diameter_mm / 2
    return 1.0 if delta >= r_wire else r_wire / (2 * delta)


# ── Parsing / formatting ─────────────────────────────────────────────────────

def parse_inductance(s):
    """Parse '10', '10uH', '100nH', '1mH' → µH."""
    s = s.strip().lower().replace("µ", "u").replace("μ", "u").replace(" ", "")
    if s.endswith("nh"):
        return float(s[:-2]) * 1e-3
    if s.endswith("uh"):
        return float(s[:-2])
    if s.endswith("mh"):
        return float(s[:-2]) * 1e3
    return float(s)  # assume µH


def parse_frequency(s):
    """Parse '1e6', '433MHz', '10kHz' → Hz."""
    s = s.strip().lower().replace(" ", "")
    if s.endswith("ghz"):
        return float(s[:-3]) * 1e9
    if s.endswith("mhz"):
        return float(s[:-3]) * 1e6
    if s.endswith("khz"):
        return float(s[:-3]) * 1e3
    if s.endswith("hz"):
        return float(s[:-2])
    return float(s)


def fmt_L(uh):
    if uh >= 1000:
        return f"{uh/1000:.4g} mH"
    if uh >= 1:
        return f"{uh:.4g} µH"
    return f"{uh*1000:.4g} nH"


def fmt_R(ohm):
    if ohm >= 1:
        return f"{ohm:.4f} Ω"
    return f"{ohm*1000:.2f} mΩ"


def fmt_Z(ohm):
    if ohm >= 1e6:
        return f"{ohm/1e6:.3g} MΩ"
    if ohm >= 1000:
        return f"{ohm/1000:.3g} kΩ"
    return f"{ohm:.3g} Ω"


def fmt_freq(hz):
    if hz >= 1e9:
        return f"{hz/1e9:.4g} GHz"
    if hz >= 1e6:
        return f"{hz/1e6:.4g} MHz"
    if hz >= 1e3:
        return f"{hz/1e3:.4g} kHz"
    return f"{hz:.4g} Hz"


# ── Core logic ────────────────────────────────────────────────────────────────

def generate_options(target_uh, wire_mm, freq_hz, diameters, gap_fractions, max_turns=500):
    options = []
    seen = set()
    for diam in diameters:
        radius = diam / 2
        for gf in gap_fractions:
            gap_mm = round(wire_mm * gf, 4)
            pitch_mm = wire_mm + gap_mm
            n_float = solve_turns(target_uh, radius, pitch_mm)
            if n_float is None or n_float < 2 or n_float > max_turns:
                continue
            n = round(n_float)
            if n < 2:
                continue
            key = (diam, n, gap_mm)
            if key in seen:
                continue
            seen.add(key)

            actual_uh = wheeler_inductance_uh(radius, n, pitch_mm)
            error_pct = abs(actual_uh - target_uh) / target_uh * 100
            r_dc = dc_resistance_ohm(radius, n, wire_mm)
            ac_fac = ac_resistance_factor(wire_mm, freq_hz)
            r_ac = r_dc * ac_fac

            Q = Z = xl = None
            if freq_hz > 0:
                omega = 2 * math.pi * freq_hz
                xl = omega * actual_uh * 1e-6
                Q = xl / r_ac if r_ac > 0 else float("inf")
                Z = math.sqrt(r_ac**2 + xl**2)

            options.append(dict(
                diameter=diam, n=n, gap=gap_mm, pitch=pitch_mm,
                actual_uh=actual_uh, error_pct=error_pct,
                r_dc=r_dc, r_ac=r_ac, Q=Q, Z=Z, xl=xl,
                length=n * pitch_mm,
                wire_m=n * math.pi * diam * 1e-3,
            ))

    return options


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description=(
            "Find single-layer air-core coil configurations for a target inductance.\n"
            "Uses the Wheeler (1928) formula. All coils are assumed to be copper wire, air core."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Inductance units: nH, uH/µH, mH  (default: µH)\n"
            "Frequency units : Hz, kHz, MHz, GHz  (or scientific notation, e.g. 1e6)\n\n"
            "Examples:\n"
            "  %(prog)s -L 10 -w 0.5\n"
            "  %(prog)s -L 100nH -w 0.3 -f 433MHz\n"
            "  %(prog)s -L 1mH -w 1.0 -f 10kHz --diameters 10 15 20 30\n"
            "  %(prog)s -L 10uH -w 0.5 -f 1MHz --permanent\n"
        ),
    )

    req = parser.add_argument_group("required arguments")
    req.add_argument("-L", "--inductance", required=True, metavar="VALUE",
                     help="Target inductance (e.g. 10, 10uH, 100nH, 1mH)")
    req.add_argument("-w", "--wire", type=float, required=True, metavar="MM",
                     help="Wire diameter in mm")

    parser.add_argument("-f", "--frequency", default="0", metavar="VALUE",
                        help="Operating frequency for Q and Z (e.g. 1e6, 433MHz; default: DC only)")
    parser.add_argument("--permanent", action="store_true",
                        help="Add --permanent flag to coil_form_generator commands")

    adv = parser.add_argument_group("search options")
    adv.add_argument("--diameters", type=float, nargs="+", default=None, metavar="MM",
                     help="Mean coil diameters to try in mm (default: auto-range)")
    adv.add_argument("--gaps", type=float, nargs="+", default=None, metavar="FRAC",
                     help="Gap as fraction of wire diameter (default: 0 0.1 0.5)")
    adv.add_argument("--max-turns", type=int, default=500, metavar="N",
                     help="Maximum number of turns to consider (default: 500)")

    args = parser.parse_args()

    try:
        target_uh = parse_inductance(args.inductance)
        freq_hz = parse_frequency(args.frequency)
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    wire_mm = args.wire
    if target_uh <= 0:
        print("Error: inductance must be positive", file=sys.stderr)
        sys.exit(1)
    if wire_mm <= 0:
        print("Error: wire diameter must be positive", file=sys.stderr)
        sys.exit(1)

    gap_fractions = args.gaps if args.gaps is not None else [0.0, 0.1, 0.5]

    if args.diameters:
        diameters = args.diameters
    else:
        d_min = max(wire_mm * 4, 3.0)
        d_max = max(d_min * 6, 40.0)
        diameters = [round(d_min + (d_max - d_min) * i / 5, 1) for i in range(6)]

    # ── Print header ──────────────────────────────────────────────────────────
    print(f"\nCoil Calculator")
    print("=" * 62)
    print(f"  Target inductance  : {fmt_L(target_uh)}")
    print(f"  Wire diameter      : {wire_mm} mm")
    print(f"  Formula            : Wheeler (1928), single-layer air-core")
    if freq_hz > 0:
        delta = skin_depth_mm(freq_hz)
        print(f"  Frequency          : {fmt_freq(freq_hz)}")
        print(f"  Cu skin depth      : {delta:.4f} mm"
              + (" (full cross-section active)" if delta >= wire_mm / 2 else " (skin-effect region)"))
    print()

    options = generate_options(target_uh, wire_mm, freq_hz, diameters, gap_fractions, args.max_turns)

    if not options:
        print("No valid configurations found. Try --diameters or a wider range.")
        sys.exit(1)

    if freq_hz > 0:
        options.sort(key=lambda o: -(o["Q"] or 0))
    else:
        options.sort(key=lambda o: o["r_dc"])

    # ── Print table ───────────────────────────────────────────────────────────
    has_freq = freq_hz > 0
    hdr = (
        f"  {'#':>2}  {'Diam':>6}  {'N':>4}  {'Gap':>5}  "
        f"{'Length':>8}  {'Wire':>7}  {'L actual':>10}  {'R_dc':>11}"
    )
    if has_freq:
        hdr += f"  {'R_ac':>11}  {'X_L':>10}  {'Q':>7}  {'|Z|':>10}"
    hdr += f"  {'Err':>6}"
    print(hdr)
    print("  " + "─" * (len(hdr) - 2))

    for i, o in enumerate(options):
        row = (
            f"  {i+1:2d}  {o['diameter']:>6.1f}  {o['n']:>4d}  {o['gap']:>5.2f}  "
            f"{o['length']:>8.2f}  {o['wire_m']:>6.3f}m  {fmt_L(o['actual_uh']):>10}  {fmt_R(o['r_dc']):>11}"
        )
        if has_freq:
            row += (
                f"  {fmt_R(o['r_ac']):>11}"
                f"  {fmt_Z(o['xl']):>10}"
                f"  {o['Q']:>7.1f}"
                f"  {fmt_Z(o['Z']):>10}"
            )
        row += f"  {o['error_pct']:>5.1f}%"
        print(row)

    print()
    print(f"  Units: Diam/Gap/Length in mm, Wire = total wire length")
    if has_freq:
        print(f"  R_ac accounts for skin effect at {fmt_freq(freq_hz)}")
        print(f"  Q = X_L / R_ac,  |Z| = sqrt(R_ac² + X_L²)")
    print()

    # ── Print generator commands ──────────────────────────────────────────────
    print("coil_form_generator.py commands:")
    print("─" * 62)
    flag = " --permanent" if args.permanent else ""
    for i, o in enumerate(options):
        print(
            f"  {i+1:2d}. python coil_form_generator.py"
            f" -d {o['diameter']}"
            f" -n {o['n']}"
            f" -w {wire_mm}"
            f" -g {o['gap']:.2f}"
            f"{flag}"
        )
    print()


if __name__ == "__main__":
    main()
