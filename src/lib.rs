//! Airfoils and wings for SolidCraft.
//!
//! Three entries:
//! - `make-wing` places airfoil sections along the span (chord, sweep, dihedral, twist) as
//!   ordinary sketches and lofts a solid through them, optionally mirrored into a full wing.
//! - `make-airfoil` draws one airfoil as a sketch on a base plane, optionally extruded.
//! - `airfoil-info` returns an airfoil's coordinates and geometric properties without changing
//!   the design.
//!
//! Every step is a native SolidCraft command, so the result is an editable timeline: the section
//! sketches, the loft and the mirror can all be opened and changed afterwards.

pub mod airfoil;
pub mod tip;
pub mod wing;

use airfoil::Airfoil;
use serde_json::{json, Value};
use tip::{Frame, Winglet};
use wing::{Planform, WingParams};

/// What a build needs from the host: run one command with JSON args, get the JSON reply.
pub trait Runner {
    fn run(&mut self, command: &str, args: Value) -> Result<Value, String>;
}

/// An input problem: the field and why.
pub type FieldError = (String, String);

fn num(v: &Value, k: &str, d: f64) -> f64 {
    v.get(k).and_then(Value::as_f64).unwrap_or(d)
}
fn text<'a>(v: &'a Value, k: &str, d: &'a str) -> &'a str {
    v.get(k).and_then(Value::as_str).unwrap_or(d)
}
fn flag(v: &Value, k: &str, d: bool) -> bool {
    v.get(k).and_then(Value::as_bool).unwrap_or(d)
}

/// Section resolution and trailing edge, shared by every entry.
#[derive(Clone, Debug, PartialEq)]
pub struct Shape {
    pub points_per_side: usize,
    /// Minimum trailing-edge thickness, chord fraction.
    pub te_gap: f64,
}

pub fn parse_shape(v: &Value) -> Result<Shape, FieldError> {
    let n = num(v, "points_per_side", 30.0).round();
    if !(8.0..=150.0).contains(&n) {
        return Err(("points_per_side".into(), "must be 8 to 150".into()));
    }
    let te = num(v, "te_thickness", 0.2);
    if !(0.0..=5.0).contains(&te) {
        return Err(("te_thickness".into(), "must be 0 to 5 % of the chord".into()));
    }
    Ok(Shape { points_per_side: n as usize, te_gap: te / 100.0 })
}

pub fn load_airfoil(designation: &str, coordinates: &str, shape: &Shape, field: &str) -> Result<Airfoil, FieldError> {
    airfoil::resolve(designation, coordinates, shape.points_per_side)
        .map(|a| a.with_min_te_gap(shape.te_gap))
        .map_err(|e| (field.to_owned(), e))
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TipStyle {
    Flat,
    /// Rounded cap, its length a fraction of the chord at the end.
    Rounded(f64),
}

pub struct WingInput {
    pub wing: WingParams,
    pub root: Airfoil,
    pub tip: Airfoil,
    pub name: String,
    pub tip_style: TipStyle,
    pub winglet: Option<(Winglet, Airfoil)>,
}

pub fn parse_wing(v: &Value) -> Result<WingInput, FieldError> {
    let shape = parse_shape(v)?;
    let planform = match text(v, "planform", "trapezoidal") {
        "trapezoidal" => Planform::Trapezoidal,
        "elliptical" => Planform::Elliptical,
        other => return Err(("planform".into(), format!("`{other}` is not trapezoidal or elliptical"))),
    };
    let default_sections = if planform == Planform::Elliptical { 12.0 } else { 5.0 };
    let wing = WingParams {
        span: num(v, "span", 500.0),
        root_chord: num(v, "root_chord", 200.0),
        taper_ratio: num(v, "taper_ratio", 0.5),
        sweep_deg: num(v, "sweep", 0.0),
        sweep_ref: num(v, "sweep_reference", 25.0) / 100.0,
        dihedral_deg: num(v, "dihedral", 0.0),
        incidence_deg: num(v, "incidence", 0.0),
        twist_deg: num(v, "twist", 0.0),
        twist_axis: num(v, "twist_axis", 25.0) / 100.0,
        planform,
        sections: num(v, "sections", default_sections).round().max(0.0) as usize,
        mirror: flag(v, "mirror", false),
    };
    wing::validate(&wing)?;
    let root = load_airfoil(text(v, "root_airfoil", "NACA 2412"), text(v, "root_coordinates", ""), &shape, "root_airfoil")?;
    let tip_name = text(v, "tip_airfoil", "").trim();
    let tip_coords = text(v, "tip_coordinates", "");
    let tip = if tip_name.is_empty() && tip_coords.trim().is_empty() {
        root.clone()
    } else {
        load_airfoil(tip_name, tip_coords, &shape, "tip_airfoil")?
    };
    let tip_style = match text(v, "tip", "flat") {
        "flat" => TipStyle::Flat,
        "rounded" => {
            let l = num(v, "tip_length", 50.0);
            if !(5.0..=300.0).contains(&l) {
                return Err(("tip_length".into(), "must be 5 to 300 % of the chord at the tip".into()));
            }
            TipStyle::Rounded(l / 100.0)
        }
        other => return Err(("tip".into(), format!("`{other}` is not flat or rounded"))),
    };
    let height = num(v, "winglet_height", 0.0);
    let winglet = if height == 0.0 {
        None
    } else {
        let wl = Winglet {
            height,
            cant_deg: num(v, "winglet_cant", 15.0),
            radius: num(v, "winglet_radius", 30.0) / 100.0,
            taper_ratio: num(v, "winglet_taper", 0.5),
            sweep_deg: num(v, "winglet_sweep", 30.0),
            toe_deg: num(v, "winglet_toe", 0.0),
        };
        tip::validate_winglet(&wl)?;
        let wname = text(v, "winglet_airfoil", "").trim();
        let wcoords = text(v, "winglet_coordinates", "");
        let foil = if wname.is_empty() && wcoords.trim().is_empty() {
            tip.clone()
        } else {
            load_airfoil(wname, wcoords, &shape, "winglet_airfoil")?
        };
        // The blend radius check needs the actual tip section.
        tip::winglet_frames(&wing, &wing.section(1.0, &tip), &tip, &foil, &wl)?;
        Some((wl, foil))
    };
    let name = text(v, "name", "Wing").trim();
    let name = if name.is_empty() { "Wing" } else { name }.to_owned();
    Ok(WingInput { wing, root, tip, name, tip_style, winglet })
}

/// Draw a closed outline (sketch coordinates) into the open sketch: one fit-point spline around
/// the airfoil and, for an open trailing edge, a line across it.
fn draw_outline<R: Runner>(r: &mut R, outline: &[[f64; 2]]) -> Result<(), String> {
    let first = outline[0];
    let last = *outline.last().unwrap();
    let round = |p: [f64; 2]| [(p[0] * 1e6).round() / 1e6, (p[1] * 1e6).round() / 1e6];
    let pts: Vec<[f64; 2]> = outline.iter().map(|&p| round(p)).collect();
    r.run("sketch.spline.fit_point", json!({"points": pts}))?;
    if (first[0] - last[0]).hypot(first[1] - last[1]) > 1e-6 {
        r.run("sketch.line", json!({"points": [pts[pts.len() - 1], pts[0]]}))?;
    }
    Ok(())
}

fn sketch_ref(reply: &Value, name: &str) -> Value {
    reply.get("sketch").filter(|s| s.is_u64()).cloned().unwrap_or_else(|| json!(name))
}

fn round3(x: f64) -> f64 {
    (x * 1000.0).round() / 1000.0
}

/// Sketch one frame: an airfoil outline, or the apex point a rounded tip closes on. Returns the
/// loft section that refers to it.
fn draw_frame<R: Runner>(r: &mut R, f: &Frame, name: &str) -> Result<Value, String> {
    let reply = r.run("sketch.create", json!({"plane": {"origin": f.origin, "x_dir": [1.0, 0.0, 0.0], "y_dir": f.y_dir}, "name": name}))?;
    match f.apex {
        Some(p) => {
            r.run("sketch.point", json!({"point": p, "id": "apex"}))?;
        }
        None => draw_outline(r, &f.outline)?,
    }
    r.run("sketch.finish", json!({}))?;
    let mut sec = json!({"sketch": sketch_ref(&reply, name)});
    if f.apex.is_some() {
        sec["point"] = json!("apex");
    }
    Ok(sec)
}

fn frame_summary(f: &Frame) -> Value {
    let le = f.outline.iter().fold([f64::INFINITY, 0.0], |m, p| if p[0] < m[0] { *p } else { m });
    let at = if f.outline.is_empty() { f.world(f.center) } else { f.world(le) };
    json!({
        "position": [round3(at[0]), round3(at[1]), round3(at[2])],
        "chord": round3(f.chord),
        "angle": round3(f.angle_deg),
        "airfoil": f.airfoil,
    })
}

/// Build the wing: one sketch per section and a loft through them; then the winglet and the
/// rounded tip, each a loft from the last section joined onto the body; then the mirror.
pub fn make_wing<R: Runner>(r: &mut R, w: &WingInput) -> Result<Value, String> {
    let p = &w.wing;
    let stations = p.stations();
    let mut refs = Vec::new();
    let mut sections = Vec::new();
    let mut end: Option<(Frame, Value)> = None;
    for (i, &eta) in stations.iter().enumerate() {
        let foil = w.root.lerp(&w.tip, eta);
        let sec = p.section(eta, &foil);
        let frame = tip::panel_frame(p, &sec, &foil);
        let sref = draw_frame(r, &frame, &format!("{} section {}", w.name, i + 1))?;
        refs.push(sref.clone());
        sections.push(json!({
            "y": round3(sec.y),
            "chord": round3(sec.chord),
            "leading_edge": [round3(sec.le_x), round3(sec.y), round3(sec.le_z)],
            "angle": round3(sec.angle_deg),
            "airfoil": foil.name,
        }));
        end = Some((frame, sref));
    }
    let loft = r.run("solid.loft", json!({"sections": refs, "operation": "new", "name": w.name, "body_name": w.name}))?;
    let body = loft
        .get("created_bodies")
        .and_then(Value::as_array)
        .and_then(|b| b.first())
        .and_then(Value::as_str)
        .unwrap_or(&w.name)
        .to_owned();
    let (mut end_frame, mut end_ref) = end.expect("at least two sections");
    let mut extent = end_frame.outline.iter().map(|&q| end_frame.world(q)).collect::<Vec<_>>();
    let mut out = json!({});
    if let Some((wl, wfoil)) = &w.winglet {
        let tip_sec = p.section(1.0, &w.tip);
        let frames = tip::winglet_frames(p, &tip_sec, &w.tip, wfoil, wl).map_err(|(f, e)| format!("{f}: {e}"))?; // checked in parse_wing
        let mut lrefs = vec![end_ref.clone()];
        for (i, f) in frames.iter().enumerate() {
            lrefs.push(draw_frame(r, f, &format!("{} winglet section {}", w.name, i + 1))?);
        }
        r.run("solid.loft", json!({"sections": lrefs, "operation": "join", "targets": [body], "name": format!("{} winglet", w.name)}))?;
        let last = frames.last().unwrap();
        out["winglet"] = json!({"sections": frames.iter().map(frame_summary).collect::<Vec<_>>(), "airfoil": wfoil.name});
        end_ref = lrefs.pop().unwrap();
        end_frame = last.clone();
        extent.extend(end_frame.outline.iter().map(|&q| end_frame.world(q)));
    }
    if let TipStyle::Rounded(len) = w.tip_style {
        let length = len * end_frame.chord;
        let frames = tip::cap_frames(&end_frame, length, 5);
        let mut lrefs = vec![end_ref];
        for (i, f) in frames.iter().enumerate() {
            lrefs.push(draw_frame(r, f, &format!("{} tip section {}", w.name, i + 1))?);
        }
        r.run("solid.loft", json!({"sections": lrefs, "operation": "join", "targets": [body], "name": format!("{} tip", w.name)}))?;
        let apex = frames.last().unwrap();
        out["tip"] = json!({"style": "rounded", "length": round3(length), "apex": apex.world(apex.center).map(round3)});
        extent.push(apex.world(apex.center));
    }
    if p.mirror {
        r.run("solid.mirror", json!({"bodies": [body], "plane": "XZ", "combine": true}))?;
    }
    let rf = wing::reference(p);
    let reach = extent.iter().map(|q| q[1]).fold(p.span, f64::max);
    let halves = if p.mirror { 2.0 } else { 1.0 };
    let mut reply = json!({
        "body": body,
        "mirrored": p.mirror,
        "span": round3(rf.span),
        "overall_span": round3(reach * halves),
        "root_chord": round3(p.root_chord),
        "tip_chord": round3(rf.tip_chord),
        "area_mm2": round3(rf.area),
        "aspect_ratio": round3(rf.aspect_ratio),
        "mean_aerodynamic_chord": round3(rf.mac),
        "mac_y": round3(rf.mac_y),
        "mac_leading_edge_x": round3(rf.mac_le_x),
        "leading_edge_sweep": round3(rf.le_sweep_deg),
        "quarter_chord_sweep": round3(rf.quarter_chord_sweep_deg),
        "root_airfoil": w.root.name,
        "tip_airfoil": w.tip.name,
        "sections": sections,
    });
    for k in ["winglet", "tip"] {
        if let Some(x) = out.get(k) {
            reply[k] = x.clone();
        }
    }
    Ok(reply)
}

pub struct AirfoilInput {
    pub foil: Airfoil,
    pub chord: f64,
    pub angle_deg: f64,
    pub plane: String,
    pub extrude: f64,
    pub name: String,
}

pub fn parse_airfoil(v: &Value) -> Result<AirfoilInput, FieldError> {
    let shape = parse_shape(v)?;
    let chord = num(v, "chord", 100.0);
    if !(chord > 0.0 && chord.is_finite()) {
        return Err(("chord".into(), "must be above 0".into()));
    }
    let angle = num(v, "angle", 0.0);
    if !(angle.abs() <= 90.0) {
        return Err(("angle".into(), "must be between -90 and 90 degrees".into()));
    }
    let plane = text(v, "plane", "XZ").to_ascii_uppercase();
    if !matches!(plane.as_str(), "XY" | "XZ" | "YZ") {
        return Err(("plane".into(), "must be XY, XZ or YZ".into()));
    }
    let extrude = num(v, "extrude", 0.0);
    if !(extrude >= 0.0 && extrude.is_finite()) {
        return Err(("extrude".into(), "must be 0 (sketch only) or a length above 0".into()));
    }
    let foil = load_airfoil(text(v, "airfoil", "NACA 2412"), text(v, "coordinates", ""), &shape, "airfoil")?;
    let name = text(v, "name", "").trim();
    let name = if name.is_empty() { foil.name.clone() } else { name.to_owned() };
    Ok(AirfoilInput { foil, chord, angle_deg: angle, plane, extrude, name })
}

/// The airfoil outline in sketch coordinates: leading edge at the origin, chord along +x, turned
/// nose up by `angle_deg` about the leading edge.
pub fn airfoil_outline(a: &AirfoilInput) -> Vec<[f64; 2]> {
    let (s, k) = a.angle_deg.to_radians().sin_cos();
    a.foil
        .contour()
        .iter()
        .map(|p| {
            let (x, y) = (p[0] * a.chord, p[1] * a.chord);
            [x * k + y * s, -x * s + y * k]
        })
        .collect()
}

pub fn make_airfoil<R: Runner>(r: &mut R, a: &AirfoilInput) -> Result<Value, String> {
    let reply = r.run("sketch.create", json!({"plane": a.plane, "name": a.name}))?;
    draw_outline(r, &airfoil_outline(a))?;
    r.run("sketch.finish", json!({}))?;
    let mut out = json!({
        "sketch": sketch_ref(&reply, &a.name),
        "airfoil": a.foil.name,
        "chord": a.chord,
        "plane": a.plane,
    });
    if a.extrude > 0.0 {
        r.run("solid.extrude", json!({"distance": a.extrude, "profiles": "all", "operation": "new", "name": a.name, "body_name": a.name}))?;
        out["extruded"] = json!(a.extrude);
    }
    Ok(out)
}

pub fn airfoil_info(v: &Value) -> Result<Value, FieldError> {
    let shape = parse_shape(v)?;
    let foil = load_airfoil(text(v, "airfoil", "NACA 2412"), text(v, "coordinates", ""), &shape, "airfoil")?;
    let p = foil.properties();
    let r6 = |x: f64| (x * 1e6).round() / 1e6;
    let pts = |s: &[[f64; 2]]| s.iter().map(|q| [r6(q[0]), r6(q[1])]).collect::<Vec<_>>();
    Ok(json!({
        "airfoil": foil.name,
        "max_thickness_percent": r6(p.max_thickness * 100.0),
        "max_thickness_at_percent": r6(p.max_thickness_x * 100.0),
        "max_camber_percent": r6(p.max_camber * 100.0),
        "max_camber_at_percent": r6(p.max_camber_x * 100.0),
        "te_thickness_percent": r6(p.te_thickness * 100.0),
        "area_per_chord_squared": r6(p.area),
        "coordinates": pts(&foil.contour()),
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Records the commands instead of running them.
    #[derive(Default)]
    struct Record(Vec<(String, Value)>);
    impl Runner for Record {
        fn run(&mut self, command: &str, args: Value) -> Result<Value, String> {
            self.0.push((command.to_owned(), args));
            Ok(match command {
                "sketch.create" => json!({"sketch": self.0.len()}),
                "solid.loft" => json!({"created_bodies": ["Wing"]}),
                _ => json!({}),
            })
        }
    }

    #[test]
    fn wing_command_sequence() {
        let v = json!({"root_airfoil": "NACA 2412", "tip_airfoil": "NACA 0009", "span": 400, "root_chord": 150, "taper_ratio": 0.6, "sections": 3, "mirror": true});
        let w = parse_wing(&v).unwrap();
        let mut r = Record::default();
        let out = make_wing(&mut r, &w).unwrap();
        let names: Vec<&str> = r.0.iter().map(|(c, _)| c.as_str()).collect();
        let per = ["sketch.create", "sketch.spline.fit_point", "sketch.line", "sketch.finish"];
        let expect: Vec<&str> = per.iter().cycle().take(12).copied().chain(["solid.loft", "solid.mirror"]).collect();
        assert_eq!(names, expect);
        // The loft refers to the three sketches by the ids the host returned.
        assert_eq!(r.0[12].1["sections"], json!([{"sketch": 1}, {"sketch": 5}, {"sketch": 9}]));
        // The last section is at the tip, in its own plane.
        assert_eq!(r.0[8].1["plane"]["origin"], json!([0.0, 400.0, 0.0]));
        assert_eq!(out["tip_airfoil"], "NACA 0009");
        assert_eq!(out["sections"][1]["airfoil"], "NACA 2412 / NACA 0009 blend 50 %");
        assert_eq!(out["span"], 800.0);
        // Spline point count: 2n - 1 for the default 30 points per side.
        assert_eq!(r.0[1].1["points"].as_array().unwrap().len(), 59);
    }

    #[test]
    fn winglet_and_rounded_tip_sequence() {
        let v = json!({"root_airfoil": "NACA 2412", "span": 400, "root_chord": 150, "taper_ratio": 0.6, "sections": 2,
                       "winglet_height": 60, "winglet_cant": 0, "tip": "rounded", "mirror": true});
        let w = parse_wing(&v).unwrap();
        let mut r = Record::default();
        let out = make_wing(&mut r, &w).unwrap();
        let lofts: Vec<&Value> = r.0.iter().filter(|(c, _)| c == "solid.loft").map(|(_, a)| a).collect();
        assert_eq!(lofts.len(), 3);
        // The winglet starts from the wing's tip sketch and joins the wing body.
        assert_eq!(lofts[1]["sections"][0], lofts[0]["sections"][1]);
        assert_eq!(lofts[1]["operation"], "join");
        assert_eq!(lofts[1]["targets"], json!(["Wing"]));
        // 4 blend + 3 straight winglet sections after the tip.
        assert_eq!(lofts[1]["sections"].as_array().unwrap().len(), 8);
        // The rounded tip starts where the winglet ends and closes on an apex point.
        let cap = lofts[2]["sections"].as_array().unwrap();
        assert_eq!(cap[0], *lofts[1]["sections"].as_array().unwrap().last().unwrap());
        assert_eq!(cap.last().unwrap()["point"], "apex");
        assert_eq!(r.0.iter().filter(|(c, _)| c == "sketch.point").count(), 1);
        assert_eq!(r.0.last().unwrap().0, "solid.mirror");
        assert!(out["winglet"]["sections"].as_array().unwrap().len() == 7);
        // A vertical winglet adds only its blend radius (0.3 x tip chord 90) and half its
        // thickness (12 % of the 45 mm winglet tip chord, or less) to each side.
        let half = out["overall_span"].as_f64().unwrap() / 2.0;
        assert!(half > 427.0 && half < 427.0 + 0.06 * 90.0, "{half}");
    }

    #[test]
    fn rounded_tip_adds_its_length() {
        let v = json!({"root_airfoil": "NACA 0012", "span": 300, "root_chord": 100, "taper_ratio": 1, "tip": "rounded", "tip_length": 40});
        let out = make_wing(&mut Record::default(), &parse_wing(&v).unwrap()).unwrap();
        assert_eq!(out["tip"]["length"], 40.0);
        assert_eq!(out["overall_span"], 340.0);
    }

    #[test]
    fn winglet_errors() {
        assert_eq!(parse_wing(&json!({"winglet_height": 50, "winglet_radius": 1, "root_airfoil": "NACA 0015"})).err().unwrap().0, "winglet_radius");
        assert_eq!(parse_wing(&json!({"winglet_height": 50, "winglet_cant": 200})).err().unwrap().0, "winglet_cant");
        assert_eq!(parse_wing(&json!({"winglet_height": 50, "winglet_airfoil": "x"})).err().unwrap().0, "winglet_airfoil");
        assert_eq!(parse_wing(&json!({"tip": "pointy"})).err().unwrap().0, "tip");
    }

    #[test]
    fn sharp_trailing_edge_has_no_line() {
        let v = json!({"airfoil": "NACA 0012", "te_thickness": 0});
        let a = parse_airfoil(&v).unwrap();
        let mut r = Record::default();
        make_airfoil(&mut r, &a).unwrap();
        let names: Vec<&str> = r.0.iter().map(|(c, _)| c.as_str()).collect();
        assert_eq!(names, ["sketch.create", "sketch.spline.fit_point", "sketch.finish"]);
    }

    #[test]
    fn airfoil_extrude() {
        let a = parse_airfoil(&json!({"airfoil": "naca 4412", "chord": 80, "extrude": 300, "plane": "xz"})).unwrap();
        let mut r = Record::default();
        let out = make_airfoil(&mut r, &a).unwrap();
        assert_eq!(r.0.last().unwrap().0, "solid.extrude");
        assert_eq!(out["airfoil"], "NACA 4412");
        let o = airfoil_outline(&a);
        let le = o[29];
        assert!(le[0].abs() < 1e-9 && le[1].abs() < 1e-9);
        assert!((o[0][0] - 80.0).abs() < 1e-6);
    }

    #[test]
    fn errors_name_the_field() {
        assert_eq!(parse_wing(&json!({"span": -1})).err().unwrap().0, "span");
        assert_eq!(parse_wing(&json!({"root_airfoil": "NACA 9x"})).err().unwrap().0, "root_airfoil");
        assert_eq!(parse_wing(&json!({"tip_airfoil": "custom"})).err().unwrap().0, "tip_airfoil");
        assert_eq!(parse_wing(&json!({"planform": "delta"})).err().unwrap().0, "planform");
        assert_eq!(parse_wing(&json!({"points_per_side": 2})).err().unwrap().0, "points_per_side");
        assert_eq!(parse_airfoil(&json!({"plane": "AB"})).err().unwrap().0, "plane");
    }

    #[test]
    fn info() {
        let v = airfoil_info(&json!({"airfoil": "NACA 0012", "te_thickness": 0})).unwrap();
        assert!((v["max_thickness_percent"].as_f64().unwrap() - 12.0).abs() < 0.02);
        assert_eq!(v["coordinates"].as_array().unwrap().len(), 59);
    }
}

#[cfg(target_arch = "wasm32")]
mod guest {
    wit_bindgen::generate!({ path: "wit", world: "command-plugin" });

    use super::*;
    use craft::plugin::{
        commands, log,
        types::{FieldFailure, PluginError},
    };

    struct Host;
    impl Runner for Host {
        fn run(&mut self, command: &str, args: Value) -> Result<Value, String> {
            match commands::run(command, &args.to_string()) {
                Ok(s) => Ok(serde_json::from_str(&s).unwrap_or(Value::Null)),
                Err(e) => Err(format!("{command}: {e:?}")),
            }
        }
    }

    fn invalid((field, reason): FieldError) -> PluginError {
        PluginError::InvalidInput(FieldFailure { field, reason })
    }

    struct Airfoils;
    impl exports::craft::plugin::entry::Guest for Airfoils {
        fn execute(entry: String, params: String) -> Result<String, PluginError> {
            let v: Value = serde_json::from_str(&params).unwrap_or(json!({}));
            match entry.as_str() {
                "make-wing" => {
                    let w = parse_wing(&v).map_err(invalid)?;
                    log::log(log::Level::Info, &format!("lofting {} through {} sections", w.name, w.wing.sections));
                    make_wing(&mut Host, &w).map(|v| v.to_string()).map_err(PluginError::Trap)
                }
                "make-airfoil" => {
                    let a = parse_airfoil(&v).map_err(invalid)?;
                    make_airfoil(&mut Host, &a).map(|v| v.to_string()).map_err(PluginError::Trap)
                }
                "airfoil-info" => airfoil_info(&v).map(|v| v.to_string()).map_err(invalid),
                other => Err(invalid(("entry".into(), format!("unknown entry {other}")))),
            }
        }
    }
    export!(Airfoils);
}
