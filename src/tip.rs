//! What goes beyond the wing panel's tip section: a winglet and a rounded tip cap.
//!
//! Both are built from [`Frame`]s: an airfoil outline in a section plane whose x axis is world X
//! (streamwise) and whose y axis is `y_dir`. The plane's normal, x cross y, points back along the
//! path, so the path runs along `tangent` = -normal. The wing panel's sections are frames with
//! `y_dir` = +Z and `tangent` = +Y.

use crate::airfoil::Airfoil;
use crate::wing::{Section, WingParams};

/// Where an airfoil (chord units) goes in a section plane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Placement {
    pub chord: f64,
    /// Leading edge before rotation, plane coordinates.
    pub le: [f64; 2],
    /// Nose-up rotation toward +y of the plane, degrees.
    pub angle_deg: f64,
    /// Chord fraction the rotation is about.
    pub pivot: f64,
}

impl Placement {
    pub fn map(&self, p: [f64; 2]) -> [f64; 2] {
        let (s, k) = self.angle_deg.to_radians().sin_cos();
        let c = self.chord;
        let (dx, dy) = ((p[0] - self.pivot) * c, p[1] * c);
        [self.le[0] + self.pivot * c + dx * k + dy * s, self.le[1] - dx * s + dy * k]
    }

    pub fn outline(&self, foil: &Airfoil) -> Vec<[f64; 2]> {
        foil.contour().iter().map(|&p| self.map(p)).collect()
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    /// Plane origin, world mm.
    pub origin: [f64; 3],
    /// Plane y axis, world; the x axis is always world X.
    pub y_dir: [f64; 3],
    /// Direction the path runs on through this plane (the plane normal, reversed).
    pub tangent: [f64; 3],
    /// The section in plane coordinates; empty for an apex.
    pub outline: Vec<[f64; 2]>,
    /// The apex point for the last frame of a rounded tip.
    pub apex: Option<[f64; 2]>,
    /// The point a rounded tip shrinks toward, plane coordinates.
    pub center: [f64; 2],
    pub chord: f64,
    /// Name of the airfoil drawn, for the reply.
    pub airfoil: String,
    pub angle_deg: f64,
}

impl Frame {
    /// World position of a point given in this frame's plane coordinates.
    pub fn world(&self, p: [f64; 2]) -> [f64; 3] {
        let o = self.origin;
        let v = self.y_dir;
        [o[0] + p[0] + p[1] * v[0], o[1] + p[1] * v[1], o[2] + p[1] * v[2]]
    }
}

/// Point a rounded tip closes toward, as a chord fraction on the chord line.
pub const CAP_CENTER: f64 = 0.4;

/// The wing panel's tip section as a frame.
pub fn panel_frame(p: &WingParams, s: &Section, foil: &Airfoil) -> Frame {
    let place = Placement { chord: s.chord, le: [s.le_x, s.le_z], angle_deg: s.angle_deg, pivot: p.twist_axis };
    Frame {
        origin: [0.0, s.y, 0.0],
        y_dir: [0.0, 0.0, 1.0],
        tangent: [0.0, 1.0, 0.0],
        outline: s.outline.clone(),
        apex: None,
        center: place.map([CAP_CENTER, 0.0]),
        chord: s.chord,
        airfoil: foil.name.clone(),
        angle_deg: s.angle_deg,
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Winglet {
    /// Length of the straight part, after the blend, mm.
    pub height: f64,
    /// Degrees from vertical: 0 straight up, 90 in the wing's plane, 180 straight down.
    pub cant_deg: f64,
    /// Blend radius between wing and winglet, as a fraction of the wing tip chord.
    pub radius: f64,
    /// Winglet tip chord / wing tip chord.
    pub taper_ratio: f64,
    /// Sweep of the winglet's twist-axis line, degrees, positive aft.
    pub sweep_deg: f64,
    /// Toe angle at the winglet tip, degrees; positive turns the leading edge outboard.
    pub toe_deg: f64,
}

pub fn validate_winglet(w: &Winglet) -> Result<(), (String, String)> {
    let bad = |f: &str, r: &str| Err((f.to_owned(), r.to_owned()));
    if !(w.height > 0.0 && w.height.is_finite()) {
        return bad("winglet_height", "must be above 0 (or 0 for no winglet)");
    }
    if !(0.0..=180.0).contains(&w.cant_deg) {
        return bad("winglet_cant", "must be 0 (up) to 180 (down) degrees");
    }
    if !(0.05..=5.0).contains(&w.radius) {
        return bad("winglet_radius", "must be 5 to 500 % of the tip chord");
    }
    if !(w.taper_ratio > 0.0 && w.taper_ratio <= 2.0) {
        return bad("winglet_taper", "must be above 0 and at most 2");
    }
    if !(w.sweep_deg.abs() <= 75.0) {
        return bad("winglet_sweep", "must be between -75 and 75 degrees");
    }
    if !(w.toe_deg.abs() <= 20.0) {
        return bad("winglet_toe", "must be between -20 and 20 degrees");
    }
    Ok(())
}

/// The winglet's sections after the wing tip (which is not repeated): around a circular blend
/// in the YZ plane, then straight out along the cant direction. Chord, sweep and toe change
/// evenly along the whole path from the wing tip; the airfoil blends from the wing tip's to the
/// winglet's through the blend.
pub fn winglet_frames(p: &WingParams, tip: &Section, tip_foil: &Airfoil, foil: &Airfoil, w: &Winglet) -> Result<Vec<Frame>, (String, String)> {
    let elev = (90.0 - w.cant_deg).to_radians(); // angle above the horizontal
    let sign = if elev < 0.0 { -1.0 } else { 1.0 };
    let r = w.radius * tip.chord;
    let ax = p.twist_axis;
    let x_axis = tip.le_x + ax * tip.chord;
    let (y0, z0) = (tip.y, tip.le_z);
    // The wing tip's twist carries on into the winglet; toe is added along the straight part.
    let toe = |f: f64| -w.toe_deg * f * sign;
    let mut frames = Vec::new();
    let bend_steps = ((elev.abs() / 22.5f64.to_radians()).ceil() as usize).max(if elev == 0.0 { 0 } else { 2 });
    let frame_at = |y: f64, z: f64, a: f64, chord: f64, x_ax: f64, angle: f64, foil: &Airfoil| {
        let (sa, ca) = a.sin_cos();
        let place = Placement { chord, le: [x_ax - ax * chord, 0.0], angle_deg: angle, pivot: ax };
        Frame {
            origin: [0.0, y, z],
            y_dir: [0.0, -sa, ca],
            tangent: [0.0, ca, sa],
            outline: place.outline(foil),
            apex: None,
            center: place.map([CAP_CENTER, 0.0]),
            chord,
            airfoil: foil.name.clone(),
            angle_deg: angle,
        }
    };
    // Taper, sweep and toe run along the whole path from the wing tip, blend included, so the
    // edges carry on smoothly instead of kinking where the blend ends.
    let bend_len = r * elev.abs();
    let total = bend_len + w.height;
    let tan_sweep = w.sweep_deg.to_radians().tan();
    let chord_at = |len: f64| tip.chord * (1.0 + (w.taper_ratio - 1.0) * len / total);
    let axis_at = |len: f64| x_axis + len * tan_sweep;
    for i in 1..=bend_steps {
        let t = i as f64 / bend_steps as f64;
        let a = elev * t;
        let len = bend_len * t;
        let (y, z) = (y0 + r * a.abs().sin(), z0 + sign * r * (1.0 - a.cos()));
        let f = tip_foil.lerp(foil, t);
        let fr = frame_at(y, z, a, chord_at(len), axis_at(len), tip.angle_deg + toe(len / total), &f);
        // The inside of the bend must clear the section, or the loft folds over itself.
        let inside = fr.outline.iter().map(|q| sign * q[1]).fold(f64::NEG_INFINITY, f64::max);
        if inside >= 0.9 * r {
            return Err((
                "winglet_radius".into(),
                format!("the blend radius ({r:.1} mm) is too tight for the tip section, which reaches {inside:.1} mm toward the inside of the bend; use at least {:.0} %", 100.0 * inside / 0.9 / tip.chord + 1.0),
            ));
        }
        frames.push(fr);
    }
    let (y1, z1) = (y0 + r * elev.abs().sin(), z0 + sign * r * (1.0 - elev.cos()));
    let (se, ce) = elev.sin_cos();
    let steps = 3;
    for j in 1..=steps {
        let d = w.height * j as f64 / steps as f64;
        let len = bend_len + d;
        let fr = frame_at(y1 + d * ce, z1 + d * se, elev, chord_at(len), axis_at(len), tip.angle_deg + toe(len / total), foil);
        frames.push(fr);
    }
    Ok(frames)
}

/// How far round the quarter ellipse the last section of a rounded tip is, as a fraction of the
/// quarter turn. SolidCraft lofts to a point only from a single section, so the cap's main loft
/// stops at this small section (8 % of the end chord) and a short second loft closes it on the
/// apex.
pub const CAP_LAST: f64 = 0.95;

/// A rounded cap after `end`: `steps` copies of its section scaled down toward its `center`
/// along a quarter ellipse of length `length`, then the apex point. The last frame is the apex.
pub fn cap_frames(end: &Frame, length: f64, steps: usize) -> Vec<Frame> {
    let q = end.center;
    (1..=steps + 1)
        .map(|i| {
            let apex = i == steps + 1;
            let phi = std::f64::consts::FRAC_PI_2 * if apex { 1.0 } else { CAP_LAST * i as f64 / steps as f64 };
            let d = length * phi.sin();
            let s = if apex { 0.0 } else { phi.cos() };
            let t = end.tangent;
            let origin = [end.origin[0] + d * t[0], end.origin[1] + d * t[1], end.origin[2] + d * t[2]];
            let shrink = |p: [f64; 2]| [q[0] + s * (p[0] - q[0]), q[1] + s * (p[1] - q[1])];
            Frame {
                origin,
                y_dir: end.y_dir,
                tangent: end.tangent,
                outline: if apex { Vec::new() } else { end.outline.iter().map(|&p| shrink(p)).collect() },
                apex: if apex { Some(q) } else { None },
                center: q,
                chord: end.chord * s,
                airfoil: end.airfoil.clone(),
                angle_deg: end.angle_deg,
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::airfoil::resolve;
    use crate::wing::Planform;

    fn wing() -> WingParams {
        WingParams {
            span: 500.0,
            root_chord: 200.0,
            taper_ratio: 0.5,
            sweep_deg: 0.0,
            sweep_ref: 0.25,
            dihedral_deg: 0.0,
            incidence_deg: 0.0,
            twist_deg: 0.0,
            twist_axis: 0.25,
            planform: Planform::Trapezoidal,
            sections: 3,
            mirror: false,
        }
    }

    fn winglet() -> Winglet {
        Winglet { height: 80.0, cant_deg: 0.0, radius: 0.3, taper_ratio: 0.5, sweep_deg: 30.0, toe_deg: 0.0 }
    }

    fn close(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn placement_matches_wing_sections() {
        let p = WingParams { twist_deg: -5.0, incidence_deg: 2.0, sweep_deg: 20.0, dihedral_deg: 4.0, ..wing() };
        let foil = resolve("NACA 2412", "", 20).unwrap();
        let s = p.section(1.0, &foil);
        let place = Placement { chord: s.chord, le: [s.le_x, s.le_z], angle_deg: s.angle_deg, pivot: p.twist_axis };
        for (a, b) in place.outline(&foil).iter().zip(&s.outline) {
            assert!(close(a[0], b[0], 1e-9) && close(a[1], b[1], 1e-9));
        }
    }

    #[test]
    fn vertical_winglet_geometry() {
        let p = wing();
        let foil = resolve("NACA 0012", "", 20).unwrap();
        let tip = p.section(1.0, &foil);
        let w = winglet();
        let fr = winglet_frames(&p, &tip, &foil, &foil, &w).unwrap();
        // 90 degrees of bend in 4 steps, then 3 straight.
        assert_eq!(fr.len(), 7);
        let r = 0.3 * 100.0;
        let last = fr.last().unwrap();
        assert!(close(last.origin[1], 500.0 + r, 1e-9), "{:?}", last.origin);
        assert!(close(last.origin[2], r + 80.0, 1e-9));
        assert!(close(last.tangent[2], 1.0, 1e-12) && close(last.y_dir[1], -1.0, 1e-12));
        assert!(close(last.chord, 50.0, 1e-9));
        // Swept back: the twist axis point moves aft by height * tan(sweep).
        let axis = |f: &Frame| f.outline[19][0] + 0.25 * f.chord; // leading edge + pivot
        assert!(close(axis(last) - axis(&fr[3]), 80.0 * 30f64.to_radians().tan(), 1e-9));
        // Frames along the blend are on the circle about (500, r).
        for f in &fr[..4] {
            assert!(close((f.origin[1] - 500.0).hypot(f.origin[2] - r), r, 1e-9));
        }
    }

    #[test]
    fn down_and_planar_winglets() {
        let p = wing();
        let foil = resolve("NACA 0012", "", 20).unwrap();
        let tip = p.section(1.0, &foil);
        let down = winglet_frames(&p, &tip, &foil, &foil, &Winglet { cant_deg: 180.0, ..winglet() }).unwrap();
        assert!(down.last().unwrap().origin[2] < -80.0);
        assert!(close(down.last().unwrap().tangent[2], -1.0, 1e-12));
        let flat = winglet_frames(&p, &tip, &foil, &foil, &Winglet { cant_deg: 90.0, ..winglet() }).unwrap();
        assert_eq!(flat.len(), 3);
        assert!(close(flat[2].origin[1], 580.0, 1e-9) && close(flat[2].origin[2], 0.0, 1e-12));
    }

    #[test]
    fn tight_radius_refused() {
        let p = wing();
        let foil = resolve("NACA 0018", "", 20).unwrap();
        let tip = p.section(1.0, &foil);
        let e = winglet_frames(&p, &tip, &foil, &foil, &Winglet { radius: 0.05, ..winglet() }).unwrap_err();
        assert_eq!(e.0, "winglet_radius");
    }

    #[test]
    fn toe_out_turns_the_nose_outboard() {
        let p = wing();
        let foil = resolve("NACA 0012", "", 20).unwrap();
        let tip = p.section(1.0, &foil);
        let fr = winglet_frames(&p, &tip, &foil, &foil, &Winglet { toe_deg: 4.0, ..winglet() }).unwrap();
        let last = fr.last().unwrap();
        let le = last.world(last.outline[19]);
        let te = last.world(last.outline[0]);
        assert!(le[1] > te[1], "leading edge {le:?} trailing edge {te:?}");
    }

    #[test]
    fn cap_shrinks_to_an_apex() {
        let p = wing();
        let foil = resolve("NACA 2412", "", 20).unwrap();
        let s = p.section(1.0, &foil);
        let end = panel_frame(&p, &s, &foil);
        let cap = cap_frames(&end, 50.0, 5);
        assert_eq!(cap.len(), 6);
        let apex = cap.last().unwrap();
        assert!(apex.outline.is_empty());
        assert_eq!(apex.apex, Some(end.center));
        assert!(close(apex.origin[1], 550.0, 1e-9));
        assert!(close(end.center[0], 0.4 * 100.0 + s.le_x, 1e-9));
        // Each section is smaller than the last.
        let width = |f: &Frame| f.outline.iter().map(|q| q[0]).fold(f64::NEG_INFINITY, f64::max) - f.outline.iter().map(|q| q[0]).fold(f64::INFINITY, f64::min);
        let mut prev = width(&end);
        for f in &cap[..5] {
            let w = width(f);
            assert!(w < prev);
            prev = w;
        }
    }
}
