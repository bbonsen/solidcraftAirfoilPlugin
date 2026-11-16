//! Wing planform: where each section sits, how big it is and how it is turned, plus the
//! planform's reference values (area, aspect ratio, mean aerodynamic chord).
//!
//! Axes: X runs aft along the root chord (leading edge to trailing edge), Y runs out along the
//! span from the root (Y = 0) to the tip, Z is up. Every section lies in a plane parallel to XZ,
//! so the airfoil is cut streamwise, as is usual for wings.

use crate::airfoil::Airfoil;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Planform {
    /// Chord changes linearly from root to tip.
    Trapezoidal,
    /// Chord follows an ellipse, c = c_tip + (c_root - c_tip) sqrt(1 - eta^2).
    Elliptical,
}

#[derive(Clone, Debug, PartialEq)]
pub struct WingParams {
    /// Root-to-tip length of one wing panel, mm.
    pub span: f64,
    pub root_chord: f64,
    /// Tip chord / root chord.
    pub taper_ratio: f64,
    /// Sweep angle of the reference line, degrees. Positive sweeps the tip back, negative forward.
    pub sweep_deg: f64,
    /// Chord fraction the sweep is measured along: 0 = leading edge, 0.25 = quarter chord.
    pub sweep_ref: f64,
    /// Dihedral angle, degrees; negative is anhedral.
    pub dihedral_deg: f64,
    /// Root incidence angle, degrees, nose up positive.
    pub incidence_deg: f64,
    /// Tip angle relative to the root, degrees. Negative is washout (tip nose down).
    pub twist_deg: f64,
    /// Chord fraction the sections turn about for incidence and twist.
    pub twist_axis: f64,
    pub planform: Planform,
    /// Number of sections lofted through, root and tip included.
    pub sections: usize,
    /// Mirror the panel about the XZ plane to build both halves.
    pub mirror: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Section {
    /// Span fraction, 0 at the root, 1 at the tip.
    pub eta: f64,
    /// Span position (world Y), mm.
    pub y: f64,
    pub chord: f64,
    /// Leading edge before twist (world X and Z), mm.
    pub le_x: f64,
    pub le_z: f64,
    /// Section angle (incidence + twist), degrees.
    pub angle_deg: f64,
    /// The section's outline in the section plane: (world X, world Z), mm, in Selig order.
    pub outline: Vec<[f64; 2]>,
}

pub fn validate(p: &WingParams) -> Result<(), (String, String)> {
    let bad = |f: &str, r: String| Err((f.to_owned(), r));
    if !(p.span > 0.0 && p.span.is_finite()) {
        return bad("span", "must be above 0".into());
    }
    if !(p.root_chord > 0.0 && p.root_chord.is_finite()) {
        return bad("root_chord", "must be above 0".into());
    }
    if !(p.taper_ratio > 0.0 && p.taper_ratio <= 10.0) {
        return bad("taper_ratio", "must be above 0 and at most 10 (tip chord / root chord)".into());
    }
    if !(p.sweep_deg.abs() < 80.0) {
        return bad("sweep", "must be between -80 and 80 degrees".into());
    }
    if !(p.dihedral_deg.abs() < 60.0) {
        return bad("dihedral", "must be between -60 and 60 degrees".into());
    }
    if !(p.twist_deg.abs() <= 45.0) {
        return bad("twist", "must be between -45 and 45 degrees".into());
    }
    if !(p.incidence_deg.abs() <= 45.0) {
        return bad("incidence", "must be between -45 and 45 degrees".into());
    }
    if !(0.0..=1.0).contains(&p.sweep_ref) {
        return bad("sweep_reference", "must be a chord fraction from 0 to 1".into());
    }
    if !(0.0..=1.0).contains(&p.twist_axis) {
        return bad("twist_axis", "must be a chord fraction from 0 to 1".into());
    }
    if !(2..=50).contains(&p.sections) {
        return bad("sections", "must be 2 to 50".into());
    }
    // With dihedral the panel is sheared upward; a sweep and a dihedral this large would fold the
    // sections over each other only at absurd values, which the checks above already exclude.
    Ok(())
}

impl WingParams {
    pub fn tip_chord(&self) -> f64 {
        self.root_chord * self.taper_ratio
    }

    pub fn chord_at(&self, eta: f64) -> f64 {
        let (cr, ct) = (self.root_chord, self.tip_chord());
        match self.planform {
            Planform::Trapezoidal => cr + (ct - cr) * eta,
            Planform::Elliptical => ct + (cr - ct) * (1.0 - eta * eta).max(0.0).sqrt(),
        }
    }

    /// Leading edge (x, z) of the untwisted section at `eta`.
    pub fn le_at(&self, eta: f64) -> (f64, f64) {
        let y = eta * self.span;
        let c = self.chord_at(eta);
        let x_ref = self.sweep_ref * self.root_chord + y * self.sweep_deg.to_radians().tan();
        (x_ref - self.sweep_ref * c, y * self.dihedral_deg.to_radians().tan())
    }

    pub fn angle_at(&self, eta: f64) -> f64 {
        self.incidence_deg + self.twist_deg * eta
    }

    /// Span stations: even for a trapezoidal wing, cosine-clustered toward the tip for an
    /// elliptical one, where the chord changes fastest.
    pub fn stations(&self) -> Vec<f64> {
        let n = self.sections;
        (0..n)
            .map(|i| {
                let s = i as f64 / (n - 1) as f64;
                match self.planform {
                    Planform::Trapezoidal => s,
                    Planform::Elliptical => (s * std::f64::consts::FRAC_PI_2).sin(),
                }
            })
            .collect()
    }

    /// Place an airfoil (chord units) as the section at `eta`.
    pub fn section(&self, eta: f64, foil: &Airfoil) -> Section {
        let c = self.chord_at(eta);
        let (le_x, le_z) = self.le_at(eta);
        let angle = self.angle_at(eta);
        let (s, k) = angle.to_radians().sin_cos();
        let ax = self.twist_axis;
        let outline = foil
            .contour()
            .iter()
            .map(|p| {
                // About the twist axis point, nose up for a positive angle.
                let (dx, dz) = ((p[0] - ax) * c, p[1] * c);
                let (rx, rz) = (dx * k + dz * s, -dx * s + dz * k);
                [le_x + ax * c + rx, le_z + rz]
            })
            .collect();
        Section { eta, y: eta * self.span, chord: c, le_x, le_z, angle_deg: angle, outline }
    }
}

/// Reference values of the planform as built (both halves when mirrored).
#[derive(Clone, Debug, PartialEq)]
pub struct Reference {
    /// Tip-to-tip span when mirrored, else the panel's span.
    pub span: f64,
    pub area: f64,
    pub aspect_ratio: f64,
    /// Mean aerodynamic chord and where it sits on one panel.
    pub mac: f64,
    pub mac_y: f64,
    pub mac_le_x: f64,
    pub tip_chord: f64,
    /// Sweep of the leading edge and the quarter-chord line of a trapezoidal panel, degrees
    /// (for an elliptical panel, root to tip).
    pub le_sweep_deg: f64,
    pub quarter_chord_sweep_deg: f64,
}

pub fn reference(p: &WingParams) -> Reference {
    // Simpson's rule over one panel.
    let n = 400;
    let h = 1.0 / n as f64;
    let (mut s, mut s_c2, mut s_y, mut s_x) = (0.0, 0.0, 0.0, 0.0);
    for i in 0..=n {
        let eta = i as f64 * h;
        let w = if i == 0 || i == n { 1.0 } else if i % 2 == 1 { 4.0 } else { 2.0 };
        let c = p.chord_at(eta);
        let (le_x, _) = p.le_at(eta);
        s += w * c;
        s_c2 += w * c * c;
        s_y += w * c * eta * p.span;
        s_x += w * c * le_x;
    }
    let k = h * p.span / 3.0;
    let half_area = s * k;
    let mac = s_c2 * k / half_area;
    let mac_y = s_y * k / half_area;
    let mac_le_x = s_x * k / half_area;
    let halves = if p.mirror { 2.0 } else { 1.0 };
    let span = p.span * halves;
    let area = half_area * halves;
    let (tip_le, _) = p.le_at(1.0);
    let ct = p.tip_chord();
    let le_sweep = (tip_le / p.span).atan().to_degrees();
    let qc = ((tip_le + 0.25 * ct - 0.25 * p.root_chord) / p.span).atan().to_degrees();
    Reference {
        span,
        area,
        aspect_ratio: span * span / area,
        mac,
        mac_y,
        mac_le_x,
        tip_chord: ct,
        le_sweep_deg: le_sweep,
        quarter_chord_sweep_deg: qc,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::airfoil::resolve;

    fn base() -> WingParams {
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
            sections: 5,
            mirror: false,
        }
    }

    fn close(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn trapezoid_reference_values() {
        let p = base();
        let r = reference(&p);
        assert!(close(r.area, 0.5 * (200.0 + 100.0) * 500.0, 1e-6));
        // MAC of a trapezoid: 2/3 cr (1 + l + l^2) / (1 + l).
        let l: f64 = 0.5;
        assert!(close(r.mac, 2.0 / 3.0 * 200.0 * (1.0 + l + l * l) / (1.0 + l), 1e-6));
        assert!(close(r.mac_y, 500.0 / 3.0 * (1.0 + 2.0 * l) / (1.0 + l), 1e-6));
        // Unswept quarter chord: the quarter chord is straight along Y.
        assert!(close(r.quarter_chord_sweep_deg, 0.0, 1e-9));
        let m = reference(&WingParams { mirror: true, ..p });
        assert!(close(m.area, 2.0 * r.area, 1e-6) && close(m.span, 1000.0, 1e-9));
        assert!(close(m.aspect_ratio, 1000.0 * 1000.0 / m.area, 1e-9));
    }

    #[test]
    fn elliptical_area() {
        // With a zero tip chord the area is a quarter ellipse: pi/4 cr b.
        let p = WingParams { planform: Planform::Elliptical, taper_ratio: 1e-9, ..base() };
        let r = reference(&p);
        assert!(close(r.area, std::f64::consts::PI / 4.0 * 200.0 * 500.0, 50.0), "{}", r.area);
        let st = p.stations();
        assert_eq!(st.first(), Some(&0.0));
        assert!(close(*st.last().unwrap(), 1.0, 1e-12));
        assert!(st.windows(2).all(|w| w[1] > w[0]));
    }

    #[test]
    fn sweep_moves_the_reference_line() {
        let p = WingParams { sweep_deg: 30.0, ..base() };
        let (x_tip, _) = p.le_at(1.0);
        // The quarter-chord point of the tip sits span * tan(30) behind the root's.
        let qc_tip = x_tip + 0.25 * 100.0;
        assert!(close(qc_tip - 0.25 * 200.0, 500.0 * 30f64.to_radians().tan(), 1e-9));
        assert!(close(reference(&p).quarter_chord_sweep_deg, 30.0, 1e-9));
        let fwd = WingParams { sweep_deg: -20.0, sweep_ref: 0.0, ..base() };
        assert!(fwd.le_at(1.0).0 < 0.0);
        assert!(close(reference(&fwd).le_sweep_deg, -20.0, 1e-9));
    }

    #[test]
    fn dihedral_lifts_the_tip() {
        let p = WingParams { dihedral_deg: 5.0, ..base() };
        assert!(close(p.le_at(1.0).1, 500.0 * 5f64.to_radians().tan(), 1e-9));
        assert_eq!(p.le_at(0.0).1, 0.0);
    }

    #[test]
    fn twist_turns_about_the_axis() {
        let foil = resolve("NACA 0012", "", 30).unwrap();
        let p = WingParams { twist_deg: -4.0, incidence_deg: 2.0, ..base() };
        let root = p.section(0.0, &foil);
        let tip = p.section(1.0, &foil);
        assert!(close(root.angle_deg, 2.0, 1e-12) && close(tip.angle_deg, -2.0, 1e-12));
        // The leading edge (contour index 29) goes up with positive incidence, down at the tip.
        let le_r = root.outline[29];
        let le_t = tip.outline[29];
        assert!(le_r[1] > 0.0 && le_t[1] < 0.0);
        // The twist axis point (quarter chord on the chord line) does not move.
        let axis = |s: &Section| s.le_x + 0.25 * s.chord;
        let r = (le_r[0] - axis(&root)).hypot(le_r[1]);
        assert!(close(r, 0.25 * 200.0, 1e-9));
        // Chord length is kept: leading edge to trailing-edge midpoint.
        let te = [(tip.outline[0][0] + tip.outline[58][0]) / 2.0, (tip.outline[0][1] + tip.outline[58][1]) / 2.0];
        assert!(close((te[0] - le_t[0]).hypot(te[1] - le_t[1]), 100.0, 1e-9));
    }

    #[test]
    fn validation() {
        assert!(validate(&base()).is_ok());
        assert_eq!(validate(&WingParams { span: 0.0, ..base() }).unwrap_err().0, "span");
        assert_eq!(validate(&WingParams { sections: 1, ..base() }).unwrap_err().0, "sections");
        assert_eq!(validate(&WingParams { sweep_deg: 85.0, ..base() }).unwrap_err().0, "sweep");
        assert_eq!(validate(&WingParams { taper_ratio: 0.0, ..base() }).unwrap_err().0, "taper_ratio");
    }
}
