# Airfoils & Wings for SolidCraft

A [SolidCraft](https://github.com/tangency-labs/solidcraft) plugin for designing airfoils and wings.
Pick an airfoil for the root and one for the tip, then set the span, chord, taper, sweep (forward
or back), dihedral and twist. The plugin draws one airfoil sketch per span station and lofts a
solid through them. It can also mirror the panel into a full wing.

Everything it makes is made with ordinary SolidCraft commands. Your timeline gets plain section
sketches, a loft and, if you asked for one, a mirror. You can open any of them and edit it, and
the design does not need the plugin installed to open later. The result is a real solid, not a
mesh.

The plugin runs sandboxed as a WebAssembly component (`craft:plugin@0.1.0`, capability
`commands@0.1`). It needs no files, no network and no clock.

## Install

You need a SolidCraft build with the plugin host (0.41 or newer).

Either install the zip: in **Preferences > Plugins** choose **Install plugin…** and pick
`solidcraft-airfoil-<version>.zip` from the [latest release](../../releases/latest).

Or copy it by hand: unzip the release into SolidCraft's plugins folder, so that you end up with a
`solidcraft-airfoil` folder holding `plugin.json` and `plugin.wasm`.

| OS | Plugins folder |
|---|---|
| Windows | `%LOCALAPPDATA%\SolidCraft\plugins\` |
| macOS | `~/Library/Application Support/SolidCraft/plugins/` |
| Linux | `$XDG_DATA_HOME/solidcraft/plugins/` (usually `~/.local/share/solidcraft/plugins/`) |

The entries show up under **Model > Plugins**. Each one opens a dialog built from its parameters.
Each release also has a `.sha256` file you can check the download against.

## Entries

### `make-wing`: loft a wing

The axes are:

- **X**: runs aft along the root chord, from the leading edge to the trailing edge.
- **Y**: runs out along the span, from the root at Y = 0 to the tip.
- **Z**: points up.

The root leading edge sits at the origin. Each section is cut streamwise, in a plane parallel to
XZ. All lengths are in mm and all angles in degrees.

| Parameter | Default | Meaning |
|---|---|---|
| `root_airfoil` | `NACA 2412` | The root section. Use a NACA 4- or 5-digit designation, or `custom` (see below) |
| `tip_airfoil` | *(same as root)* | The tip section. The sections in between blend linearly from root to tip |
| `span` | 500 | Length of one wing panel, root to tip |
| `root_chord` | 200 | Chord at the root |
| `taper_ratio` | 0.5 | Tip chord / root chord. 1 gives a rectangular wing |
| `sweep` | 0 | Sweep of the reference line. Positive sweeps the tip back, negative sweeps it forward |
| `sweep_reference` | 25 | Chord position (%) that the sweep is measured at. 0 is the leading edge, 25 the quarter chord |
| `dihedral` | 0 | Dihedral angle. A negative value gives anhedral |
| `twist` | 0 | Tip angle relative to the root, linear along the span. A negative value is washout (tip nose-down) |
| `incidence` | 0 | Nose-up angle of the root section. The twist is added on top |
| `twist_axis` | 25 | Chord position (%) that the sections rotate about for incidence and twist |
| `planform` | `trapezoidal` | `trapezoidal` gives a straight taper. `elliptical` gives c = c_tip + (c_root - c_tip)·√(1 - η²), with stations clustered toward the tip |
| `mirror` | false | Mirror the panel about the XZ plane and join both halves into one body |
| `sections` | 5 (elliptical: 12) | Number of sections lofted through, root and tip included, 2 to 50 |
| `points_per_side` | 30 | Points per airfoil surface, cosine-spaced, 8 to 150 |
| `te_thickness` | 0.2 | Minimum trailing-edge thickness, in % of chord. A sharp trailing edge is opened to this, and 0 keeps it sharp |
| `root_coordinates`, `tip_coordinates` | | Coordinate text, used when the matching airfoil is `custom` |
| `name` | `Wing` | Name of the loft and its body. The sections are named `<name> section N` |

The reply reports the reference values of the wing as built:

- span and tip chord
- area and aspect ratio
- mean aerodynamic chord, and where it sits
- leading-edge and quarter-chord sweep
- each section's position, chord, angle and airfoil

These are measured over both halves when the wing is mirrored.

### `make-airfoil`: one airfoil sketch

This entry draws the airfoil on the XZ, XY or YZ plane, with the leading edge at the origin and
the chord along the sketch's +x. Set `extrude` above 0 to also extrude it into a straight wing
section. `angle` rotates the airfoil nose-up about the leading edge. The `airfoil`, `coordinates`,
`points_per_side` and `te_thickness` parameters work as they do for wings.

### `airfoil-info`: look before you build

This is a read-only entry and changes nothing. It returns the airfoil's:

- maximum thickness and camber, and where they are
- trailing-edge thickness
- section area per chord²
- normalised coordinates, in Selig order

## Airfoils

**NACA 4-digit (MPXX)**: M % maximum camber at P tenths of chord, XX % thick. For example
`NACA 0012` or `NACA 2412`. The plugin uses the closed-trailing-edge thickness form; `te_thickness`
then gives the trailing edge a thickness you can build.

**NACA 5-digit (LPQXX)**: design lift coefficient 0.15·L, maximum camber position P/20, Q = 1 for a
reflexed camber line, XX % thick. For example `NACA 23012`, or the reflexed `NACA 23112`. Standard
camber lines use P = 1 to 5 and reflexed ones P = 2 to 5, with the usual tabulated constants.

**Custom**: set the airfoil to `custom` and paste the coordinates. Accepted formats:

- Selig (`.dat` files from the [UIUC airfoil database](https://m-selig.ae.illinois.edu/ads/coord_database.html)):
  trailing edge, then over the top to the leading edge, then back underneath.
- Lednicer: a line with the two point counts, then each surface from the leading edge.
- A single line of `x y x y …` numbers.

The first line that is not just numbers is taken as the airfoil's name. Coordinates that are not
in chord units are shifted and scaled to a chord of 1. Each surface is resampled with a monotone
cubic in √x, which keeps a round nose round. This also gives every section the same point layout,
so different airfoils can blend into one another.

## How it builds

1. For each span station η, the plugin blends the root and tip airfoils, then scales the result to
   the local chord. It rotates the section by incidence + twist·η about the twist axis, and moves
   it by the sweep (along X) and the dihedral (along Z).
2. Each section becomes a sketch on its own plane at Y = η·span: a fit-point spline around the
   airfoil, plus a line across the trailing edge when the trailing edge is open.
3. `solid.loft` runs through the sections in order to make a new body.
4. With `mirror` on, `solid.mirror` reflects the body about XZ and joins the two halves.

All of this is one undo step. To change the wing, edit the features, or undo and run the entry
again with new values.

Not modelled: winglets, cranked (multi-panel) planforms, rounded tips, control surfaces and
spars. You can model these afterwards with normal SolidCraft features, or build each panel with
its own run.

## Command line

Use a `solidcraft-cli` that has the plugin host:

```bash
solidcraft-cli run inspect.json --results
```

```bash
solidcraft-cli run wing.json --results --save wing.sc
```

Here `inspect.json` is `[{"command":"plugin.inspect","params":{"path":"<plugins>/solidcraft-airfoil"}}]`,
and `wing.json` is:

```json
[{"command": "plugin.run", "params": {
  "path": "<plugins>/solidcraft-airfoil",
  "approval": "<the approval hash inspect gave you>",
  "entry": "make-wing",
  "params": {"root_airfoil": "NACA 2412", "tip_airfoil": "NACA 0010", "span": 600,
             "root_chord": 220, "taper_ratio": 0.45, "sweep": 8, "dihedral": 3,
             "twist": -2.5, "mirror": true}}}]
```

## Build

You need Rust with the `wasm32-unknown-unknown` target and
[`wasm-tools`](https://github.com/bytecodealliance/wasm-tools):

```bash
rustup target add wasm32-unknown-unknown
```

```bash
cargo install wasm-tools
```

```bash
./build.sh
```

`./build.sh` builds the core module, wraps it as a component and leaves the package in
`dist/solidcraft-airfoil/`. The WIT in `wit/` is a copy of SolidCraft's `craft:plugin@0.1.0`.

## Tests

`cargo test` checks the maths:

- NACA thickness and camber, and the continuity of the 5-digit camber lines
- the Selig and Lednicer parsers, and resampling
- blending between airfoils
- chord, sweep, dihedral and twist placement
- planform area and mean aerodynamic chord
- the command sequence sent to the host

`tests/cli_test.py` runs the built plugin through `solidcraft-cli` and measures the solids it
makes. It works out the expected values from the textbook formulas on its own:

- volumes against the integrated section area
- bounding boxes for chord, span, sweep and dihedral
- mirror symmetry
- the area of an elliptical planform
- custom coordinates
- that a saved design reopens

```bash
./build.sh
```

```bash
SOLIDCRAFT_CLI=/path/to/solidcraft-cli python3 tests/cli_test.py
```

## Releases

Pushing a tag such as `v0.1.0` runs `.github/workflows/release.yml`. It tests and builds the
plugin, then attaches `solidcraft-airfoil-0.1.0.zip` and its `.sha256` to a GitHub release. The
version in `Cargo.toml` and in `plugin.json` must match the tag.

## License

MIT, see [LICENSE](LICENSE).
