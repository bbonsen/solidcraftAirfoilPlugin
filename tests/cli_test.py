#!/usr/bin/env python3
"""End-to-end test: runs the built plugin through solidcraft-cli and measures what it made.

    SOLIDCRAFT_CLI=/path/to/solidcraft-cli python3 tests/cli_test.py [package-dir]

The package dir holds plugin.json and plugin.wasm (default: dist/solidcraft-airfoil).
The expected values are worked out here from the textbook formulas, independently of the plugin.
"""
import json, math, os, shutil, subprocess, sys, tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CLI = os.environ.get("SOLIDCRAFT_CLI", "solidcraft-cli")
PKG = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "dist", "solidcraft-airfoil"))
TMP = tempfile.mkdtemp(prefix="airfoil-test-")


def run(steps, save=None, inp=None, allow_fail=False):
    script = os.path.join(TMP, "s.json")
    out = os.path.join(TMP, "o.json")
    if os.path.exists(out):
        os.remove(out)
    json.dump(steps, open(script, "w"))
    cmd = [CLI, "run", script, "--results", "--quiet", "--json-out", out]
    if inp:
        cmd += ["--in", inp]
    if save:
        cmd += ["--save", save]
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode != 0:
        if allow_fail:
            return p.stderr + p.stdout
        sys.exit("cli failed: " + p.stderr[-3000:] + p.stdout[-1000:])
    return [r.get("result", r) if isinstance(r, dict) else r for r in json.load(open(out))]


APPROVAL = run([{"command": "plugin.inspect", "params": {"path": PKG}}])[0]["approval"]


def plugin(entry, params):
    return {"command": "plugin.run", "params": {"path": PKG, "approval": APPROVAL, "entry": entry, "params": params}}


failed = []


def check(name, ok, detail=""):
    print(("PASS " if ok else "FAIL ") + name + (" " + detail if detail else ""))
    if not ok:
        failed.append(name)


def naca00_area(t, te_gap):
    """Section area / chord^2 of a symmetric NACA 4-digit airfoil (closed-TE form) with its
    trailing edge opened linearly to te_gap."""
    c = 0.2969 * 2 / 3 - 0.1260 / 2 - 0.3516 / 3 + 0.2843 / 4 - 0.1036 / 5
    return 10 * t * c + te_gap / 2


def build(entry, params, extra=(), save=None):
    res = run([plugin(entry, params), {"command": "inspect.measure", "params": {}}, *extra], save=save)
    return res[0], res[1]["bodies"], res[2:]


def one_closed(label, bodies):
    check(f"{label}: one closed solid", len(bodies) == 1 and bodies[0].get("closed"), json.dumps([b.get("closed") for b in bodies]))
    return bodies[0]


def near(label, got, want, rel=0.01, abs_=1e-6):
    check(label, abs(got - want) <= max(abs_, rel * abs(want)), f"got {got:.6g}, want {want:.6g}")


def volume_integral(chord, span, area, n=2000):
    return sum(area * chord((i + 0.5) / n) ** 2 for i in range(n)) * span / n


out_dir = os.path.join(TMP, "out")
os.makedirs(out_dir)

# 1. Rectangular, unswept, untwisted NACA 0012 wing: a straight prism.
p = {"root_airfoil": "NACA 0012", "span": 400, "root_chord": 100, "taper_ratio": 1, "sections": 2}
reply, bodies, _ = build("make-wing", p, save=os.path.join(out_dir, "rect.sc"))
b = one_closed("rectangular", bodies)
A = naca00_area(0.12, 0.002)
near("rectangular: volume = section area x span", b["volume_mm3"], A * 100**2 * 400)
bb = b["bbox"]
near("rectangular: chord 0..100 along X", bb["max"][0] - bb["min"][0], 100, rel=1e-4)
near("rectangular: span 0..400 along Y", bb["max"][1], 400, rel=1e-6)
near("rectangular: root at Y = 0", bb["min"][1], 0, abs_=1e-6)
near("rectangular: thickness 12 % of chord", bb["max"][2] - bb["min"][2], 12, rel=0.01)
near("rectangular: reported area", reply["area_mm2"], 40000, rel=1e-6)
reopened = run([{"command": "inspect.measure", "params": {}}], inp=os.path.join(out_dir, "rect.sc"))[0]
check("rectangular: saved design reopens with one closed body", len(reopened["bodies"]) == 1 and reopened["bodies"][0]["closed"])

# 2. Tapered wing swept back 30 degrees at the quarter chord.
cr, lam, span, sw = 200, 0.5, 500, 30
p = {"root_airfoil": "NACA 0012", "span": span, "root_chord": cr, "taper_ratio": lam, "sweep": sw, "sections": 3}
reply, bodies, _ = build("make-wing", p)
b = one_closed("swept taper", bodies)
ct = cr * lam
tip_le = 0.25 * cr + span * math.tan(math.radians(sw)) - 0.25 * ct
near("swept taper: volume", b["volume_mm3"], volume_integral(lambda e: cr + (ct - cr) * e, span, A))
near("swept taper: tip trailing edge is the aftmost point", b["bbox"]["max"][0], tip_le + ct, rel=1e-3)
near("swept taper: root leading edge is the foremost point", b["bbox"]["min"][0], 0, abs_=0.05)
near("swept taper: reported quarter-chord sweep", reply["quarter_chord_sweep"], sw, abs_=1e-3)
mac = 2 / 3 * cr * (1 + lam + lam * lam) / (1 + lam)
near("swept taper: mean aerodynamic chord", reply["mean_aerodynamic_chord"], mac, rel=1e-4)

# 3. Forward sweep at the leading edge: the tip sits ahead of the root.
p = {"root_airfoil": "NACA 0012", "span": 300, "root_chord": 120, "taper_ratio": 0.6, "sweep": -15, "sweep_reference": 0, "sections": 2}
_, bodies, _ = build("make-wing", p)
b = one_closed("forward sweep", bodies)
near("forward sweep: tip leading edge ahead of the root", b["bbox"]["min"][0], 300 * math.tan(math.radians(-15)), rel=1e-3)

# 4. Dihedral and washout, mirrored into a full wing.
p = {"root_airfoil": "NACA 2412", "tip_airfoil": "NACA 0010", "span": 400, "root_chord": 150, "taper_ratio": 0.6,
     "dihedral": 5, "twist": -3, "incidence": 2, "mirror": True, "sections": 5}
reply, bodies, _ = build("make-wing", p, save=os.path.join(out_dir, "full.sc"))
b = one_closed("full wing", bodies)
near("full wing: tip to tip span", b["bbox"]["max"][1] - b["bbox"]["min"][1], 800, rel=1e-6)
near("full wing: symmetric about XZ", b["bbox"]["max"][1] + b["bbox"]["min"][1], 0, abs_=1e-6)
check("full wing: tips raised by dihedral", b["bbox"]["max"][2] > 400 * math.tan(math.radians(5)), f"max z {b['bbox']['max'][2]:.3f}")
near("full wing: reported span", reply["span"], 800, rel=1e-9)
check("full wing: blended airfoils", reply["root_airfoil"] == "NACA 2412" and reply["tip_airfoil"] == "NACA 0010")
half = run([plugin("make-wing", {**p, "mirror": False}), {"command": "inspect.measure", "params": {}}])[1]["bodies"][0]
near("full wing: volume is twice one panel", b["volume_mm3"], 2 * half["volume_mm3"], rel=1e-3)

# 5. Elliptical planform.
cr, ct, span = 180, 20, 450
p = {"root_airfoil": "NACA 0012", "span": span, "root_chord": cr, "taper_ratio": ct / cr, "planform": "elliptical", "sections": 16}
reply, bodies, _ = build("make-wing", p)
b = one_closed("elliptical", bodies)
chord = lambda e: ct + (cr - ct) * math.sqrt(max(0.0, 1 - e * e))
near("elliptical: volume", b["volume_mm3"], volume_integral(chord, span, A), rel=0.02)
area = sum(chord((i + 0.5) / 4000) for i in range(4000)) * span / 4000
near("elliptical: reported area", reply["area_mm2"], area, rel=1e-3)

# 6. One airfoil, extruded: cambered NACA 4412.
p = {"airfoil": "NACA 4412", "chord": 80, "plane": "XZ", "angle": 0, "extrude": 250}
reply, bodies, _ = build("make-airfoil", p)
b = one_closed("airfoil extrude", bodies)
info = run([plugin("airfoil-info", {"airfoil": "NACA 4412", "te_thickness": 0.2, "points_per_side": 30})])[0]
near("airfoil extrude: volume = section area x length", b["volume_mm3"], info["area_per_chord_squared"] * 80**2 * 250, rel=0.005)
near("airfoil-info: NACA 4412 is 12 % thick", info["max_thickness_percent"], 12, rel=0.01)
near("airfoil-info: NACA 4412 has 4 % camber", info["max_camber_percent"], 4, rel=0.01)
near("airfoil-info: max camber at 40 %", info["max_camber_at_percent"], 40, rel=0.03)

# 7. A 5-digit airfoil and a sharp trailing edge.
for label, prm in {"NACA 23012": {"te_thickness": 0.2}, "NACA 0012 sharp": {"te_thickness": 0}}.items():
    prm = {"airfoil": label.split(" sharp")[0], "chord": 100, "plane": "XZ", "angle": 0, "extrude": 50, **prm}
    res = run([plugin("make-airfoil", prm), {"command": "inspect.measure", "params": {}}], allow_fail=True)
    if isinstance(res, str):
        check(f"{label}: built", False, res[-400:])
        continue
    b = one_closed(label, res[1]["bodies"])
    if "sharp" in label:
        near(f"{label}: volume", b["volume_mm3"], naca00_area(0.12, 0) * 100**2 * 50, rel=0.005)

# 8. Custom coordinates (Selig format), here a NACA 0015 written out by this test.
xs = [0.5 * (1 - math.cos(math.pi * i / 60)) for i in range(61)]
yt = lambda x: 5 * 0.15 * (0.2969 * math.sqrt(x) - 0.1260 * x - 0.3516 * x**2 + 0.2843 * x**3 - 0.1036 * x**4)
dat = "MY TEST FOIL\n" + "".join(f"{x:.6f} {yt(x):.6f}\n" for x in reversed(xs)) + "".join(f"{x:.6f} {-yt(x):.6f}\n" for x in xs[1:])
p = {"root_airfoil": "custom", "root_coordinates": dat, "span": 200, "root_chord": 100, "taper_ratio": 1, "sections": 2}
reply, bodies, _ = build("make-wing", p)
b = one_closed("custom coordinates", bodies)
near("custom coordinates: volume", b["volume_mm3"], naca00_area(0.15, 0.002) * 100**2 * 200, rel=0.01)
check("custom coordinates: name from the file", reply["root_airfoil"] == "MY TEST FOIL", reply["root_airfoil"])

# 9. Rounded tip on a rectangular wing. The cap scales the tip section by s = sqrt(1 - (d/R)^2)
# along its length R, so it adds A c^2 * integral(s^2) = A c^2 * 2R/3.
c, span, R = 100, 300, 50
p = {"root_airfoil": "NACA 0012", "span": span, "root_chord": c, "taper_ratio": 1, "sections": 2, "tip": "rounded", "tip_length": 50}
reply, bodies, _ = build("make-wing", p)
b = one_closed("rounded tip", bodies)
near("rounded tip: reaches tip_length past the span", b["bbox"]["max"][1], span + R, rel=2e-3)
near("rounded tip: volume = panel + 2/3 of a prism as long as the tip", b["volume_mm3"], A * c**2 * (span + 2 * R / 3), rel=0.02)
near("rounded tip: reported overall span", reply["overall_span"], span + R, rel=1e-6)

# 10. Vertical winglet: the blend radius r, then the straight part up to r + height.
c, span, h, rad = 100, 300, 60, 0.3
p = {"root_airfoil": "NACA 0012", "span": span, "root_chord": c, "taper_ratio": 1, "sections": 2,
     "winglet_height": h, "winglet_cant": 0, "winglet_radius": rad * 100, "winglet_taper": 0.5, "winglet_sweep": 45}
reply, bodies, _ = build("make-wing", p, save=os.path.join(out_dir, "winglet.sc"))
b = one_closed("winglet", bodies)
r = rad * c
near("winglet: top at blend radius + height", b["bbox"]["max"][2], r + h, rel=1e-3)
# The loft between blend sections bulges a little past the true circle (about 0.2 % of chord).
check("winglet: outboard edge just past the blend", span + r < b["bbox"]["max"][1] < span + r + 0.07 * c, f"max y {b['bbox']['max'][1]:.3f}")
path = r * math.pi / 2 + h  # sweep runs along the whole path, blend included
near("winglet: winglet tip trailing edge is aftmost (sweep)", b["bbox"]["max"][0], 0.25 * c + path * math.tan(math.radians(45)) + 0.75 * 0.5 * c, rel=2e-3)
check("winglet: adds volume", b["volume_mm3"] > A * c**2 * span)

# 11. Canted winglet with a rounded tip, both halves.
p = {"root_airfoil": "NACA 2412", "tip_airfoil": "NACA 0010", "span": 400, "root_chord": 160, "taper_ratio": 0.5, "sweep": 10,
     "dihedral": 3, "twist": -2, "mirror": True, "sections": 3, "winglet_height": 50, "winglet_cant": 20,
     "winglet_airfoil": "NACA 0008", "winglet_toe": 2, "tip": "rounded", "tip_length": 60}
reply, bodies, _ = build("make-wing", p, save=os.path.join(out_dir, "full-winglet.sc"))
b = one_closed("winglet + rounded tip + mirror", bodies)
near("winglet + rounded tip + mirror: symmetric about XZ", b["bbox"]["max"][1] + b["bbox"]["min"][1], 0, abs_=1e-6)
apex = reply["tip"]["apex"]
check("winglet + rounded tip + mirror: apex at the top", abs(b["bbox"]["max"][2] - apex[2]) < 0.02 * 50, f"max z {b['bbox']['max'][2]:.3f}, apex {apex}")
reopened = run([{"command": "inspect.measure", "params": {}}], inp=os.path.join(out_dir, "full-winglet.sc"))[0]
check("winglet + rounded tip + mirror: saved design reopens", len(reopened["bodies"]) == 1 and reopened["bodies"][0]["closed"])

# 12. Downward winglet.
p = {"root_airfoil": "NACA 0012", "span": 300, "root_chord": 100, "taper_ratio": 1, "sections": 2,
     "winglet_height": 40, "winglet_cant": 180, "winglet_radius": 30}
_, bodies, _ = build("make-wing", p)
b = one_closed("downward winglet", bodies)
near("downward winglet: bottom at -(radius + height)", b["bbox"]["min"][2], -(30 + 40), rel=1e-3)

# 13. Bad input is refused with the field named, and leaves the design empty.
err = run([plugin("make-wing", {"root_airfoil": "NACA 9x12", "span": 100, "root_chord": 50})], allow_fail=True)
check("bad airfoil is refused naming the field", isinstance(err, str) and "root_airfoil" in err, (err if isinstance(err, str) else "")[-300:])

shutil.rmtree(TMP)
print(f"\n{'FAILED: ' + ', '.join(failed) if failed else 'all passed'}")
sys.exit(1 if failed else 0)
