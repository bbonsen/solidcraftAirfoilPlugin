//! Airfoil sections: NACA 4- and 5-digit generators, a parser for Selig and Lednicer coordinate
//! files, resampling onto a common cosine-spaced grid, blending between two profiles and the
//! geometric properties of a section.
//!
//! All coordinates here are in chord units: the leading edge at (0, 0) and the trailing edge at
//! x = 1. Both surfaces run from the leading edge to the trailing edge and share the leading-edge
//! point, so `upper[0] == lower[0]`.

use std::f64::consts::PI;

/// One airfoil section, in chord units.
#[derive(Clone, Debug, PartialEq)]
pub struct Airfoil {
    pub name: String,
    /// Upper surface, leading edge to trailing edge.
    pub upper: Vec<[f64; 2]>,
    /// Lower surface, leading edge to trailing edge. Same length as `upper`.
    pub lower: Vec<[f64; 2]>,
}

/// Where an airfoil comes from.
#[derive(Clone, Debug, PartialEq)]
pub enum Source {
    /// NACA 4-digit series MPXX: max camber M %, at P tenths of chord, thickness XX %.
    Naca4 { m: f64, p: f64, t: f64 },
    /// NACA 5-digit series LPQXX: design lift coefficient 0.15 L, max camber position P/20,
    /// Q = 1 for a reflexed camber line, thickness XX %.
    Naca5 { cl: f64, p_index: u32, reflex: bool, t: f64 },
    /// Coordinates read from a Selig or Lednicer file.
    Points(Vec<[f64; 2]>),
}

/// Parse an airfoil designation such as `NACA 2412`, `naca0012`, `23012`, `NACA 23112`.
pub fn parse_designation(s: &str) -> Result<Source, String> {
    let cleaned: String = s.trim().to_ascii_lowercase().replace("naca", "").chars().filter(|c| !matches!(c, ' ' | '-' | '_')).collect();
    if cleaned.is_empty() {
        return Err("empty airfoil designation; use for example NACA 2412, NACA 23012 or custom".into());
    }
    if !cleaned.chars().all(|c| c.is_ascii_digit()) {
        return Err(format!("`{s}` is not a NACA 4- or 5-digit designation (for example NACA 2412 or NACA 23012); use `custom` with coordinates for other airfoils"));
    }
    let d: Vec<u32> = cleaned.chars().map(|c| c.to_digit(10).unwrap()).collect();
    match d.len() {
        4 => {
            let m = d[0] as f64 / 100.0;
            let p = d[1] as f64 / 10.0;
            let t = (d[2] * 10 + d[3]) as f64 / 100.0;
            if t <= 0.0 {
                return Err(format!("`{s}`: thickness must be above 0 %"));
            }
            if (m == 0.0) != (p == 0.0) {
                return Err(format!("`{s}`: camber and its position must both be zero (symmetric) or both non-zero"));
            }
            Ok(Source::Naca4 { m, p, t })
        }
        5 => {
            let cl = d[0] as f64 * 0.15;
            let p_index = d[1];
            let reflex = match d[2] {
                0 => false,
                1 => true,
                _ => return Err(format!("`{s}`: the third digit of a 5-digit NACA airfoil is 0 (normal) or 1 (reflexed)")),
            };
            let t = (d[3] * 10 + d[4]) as f64 / 100.0;
            if t <= 0.0 {
                return Err(format!("`{s}`: thickness must be above 0 %"));
            }
            if d[0] == 0 {
                return Err(format!("`{s}`: the first digit (design lift coefficient) must be 1 to 9"));
            }
            let ok = if reflex { (2..=5).contains(&p_index) } else { (1..=5).contains(&p_index) };
            if !ok {
                return Err(format!(
                    "`{s}`: the second digit (max camber position) must be {} for this series",
                    if reflex { "2 to 5" } else { "1 to 5" }
                ));
            }
            Ok(Source::Naca5 { cl, p_index, reflex, t })
        }
        _ => Err(format!("`{s}` is not a NACA 4- or 5-digit designation; use `custom` with coordinates for other airfoils")),
    }
}

/// `n` cosine-spaced stations from 0 to 1, clustered at the leading and trailing edges.
pub fn cosine_spacing(n: usize) -> Vec<f64> {
    (0..n).map(|i| 0.5 * (1.0 - (PI * i as f64 / (n - 1) as f64).cos())).collect()
}

/// NACA half-thickness with the closed trailing edge coefficient (-0.1036).
fn naca_thickness(t: f64, x: f64) -> f64 {
    5.0 * t * (0.2969 * x.sqrt() - 0.1260 * x - 0.3516 * x * x + 0.2843 * x.powi(3) - 0.1036 * x.powi(4))
}

/// Camber line ordinate and slope for a source with an analytic camber line.
fn camber(src: &Source, x: f64) -> (f64, f64) {
    match *src {
        Source::Naca4 { m, p, .. } => {
            if m == 0.0 {
                (0.0, 0.0)
            } else if x < p {
                (m / (p * p) * (2.0 * p * x - x * x), 2.0 * m / (p * p) * (p - x))
            } else {
                let q = (1.0 - p) * (1.0 - p);
                (m / q * ((1.0 - 2.0 * p) + 2.0 * p * x - x * x), 2.0 * m / q * (p - x))
            }
        }
        Source::Naca5 { cl, p_index, reflex, .. } => {
            let i = p_index as usize - 1;
            let scale = cl / 0.3;
            if !reflex {
                // Standard NACA 5-digit camber line constants for the design CL of 0.3.
                const M: [f64; 5] = [0.0580, 0.1260, 0.2025, 0.2900, 0.3910];
                const K1: [f64; 5] = [361.400, 51.640, 15.957, 6.643, 3.230];
                let (m, k1) = (M[i], K1[i] * scale);
                if x < m {
                    (k1 / 6.0 * (x.powi(3) - 3.0 * m * x * x + m * m * (3.0 - m) * x), k1 / 6.0 * (3.0 * x * x - 6.0 * m * x + m * m * (3.0 - m)))
                } else {
                    (k1 / 6.0 * m.powi(3) * (1.0 - x), -k1 / 6.0 * m.powi(3))
                }
            } else {
                // Reflexed camber line constants (index 0 unused: P = 1 has no reflexed form).
                const M: [f64; 5] = [0.0, 0.1300, 0.2170, 0.3180, 0.4410];
                const K1: [f64; 5] = [0.0, 51.990, 15.793, 6.520, 3.191];
                const K21: [f64; 5] = [0.0, 0.000764, 0.00677, 0.0303, 0.1355];
                let (m, k1, r) = (M[i], K1[i] * scale, K21[i]);
                let m3 = m.powi(3);
                let c = (1.0 - m).powi(3);
                if x < m {
                    (
                        k1 / 6.0 * ((x - m).powi(3) - r * c * x - m3 * x + m3),
                        k1 / 6.0 * (3.0 * (x - m).powi(2) - r * c - m3),
                    )
                } else {
                    (
                        k1 / 6.0 * (r * (x - m).powi(3) - r * c * x - m3 * x + m3),
                        k1 / 6.0 * (3.0 * r * (x - m).powi(2) - r * c - m3),
                    )
                }
            }
        }
        Source::Points(_) => (0.0, 0.0),
    }
}

/// Build an airfoil from its source, sampled at `n` cosine-spaced stations per surface.
pub fn generate(src: &Source, name: &str, n: usize) -> Result<Airfoil, String> {
    let xs = cosine_spacing(n);
    match src {
        Source::Naca4 { t, .. } | Source::Naca5 { t, .. } => {
            let mut upper = Vec::with_capacity(n);
            let mut lower = Vec::with_capacity(n);
            for &x in &xs {
                let yt = naca_thickness(*t, x);
                let (yc, dyc) = camber(src, x);
                let th = dyc.atan();
                upper.push([x - yt * th.sin(), yc + yt * th.cos()]);
                lower.push([x + yt * th.sin(), yc - yt * th.cos()]);
            }
            // Pin the shared leading edge and the closed trailing edge exactly.
            upper[0] = [0.0, 0.0];
            lower[0] = [0.0, 0.0];
            let te = upper[n - 1];
            lower[n - 1] = te;
            Ok(normalise(Airfoil { name: name.to_owned(), upper, lower }))
        }
        Source::Points(pts) => from_points(pts, name, &xs),
    }
}

/// Parse a Selig (trailing edge, upper surface, leading edge, lower surface, trailing edge) or
/// Lednicer (counts line, then upper and lower surfaces each from the leading edge) coordinate
/// list. Header lines and lines that are not exactly two numbers are skipped. A text with all
/// coordinates on one line is read as x y pairs.
pub fn parse_coordinates(text: &str) -> Result<Vec<[f64; 2]>, String> {
    let nums = |line: &str| -> Option<Vec<f64>> {
        let v: Vec<&str> = line.split(|c: char| c.is_whitespace() || c == ',' || c == ';').filter(|s| !s.is_empty()).collect();
        if v.is_empty() {
            return None;
        }
        v.iter().map(|s| s.parse::<f64>().ok().filter(|f| f.is_finite())).collect()
    };
    let lines: Vec<&str> = text.lines().filter(|l| !l.trim().is_empty()).collect();
    let mut pairs: Vec<[f64; 2]> = Vec::new();
    if lines.len() <= 2 {
        // One line of numbers (a pasted single-line list), possibly after a title line.
        if let Some(v) = lines.iter().rev().find_map(|l| nums(l).filter(|v| v.len() >= 6)) {
            if v.len() % 2 != 0 {
                return Err("coordinates must come in x y pairs".into());
            }
            pairs = v.chunks(2).map(|c| [c[0], c[1]]).collect();
        }
    } else {
        for l in &lines {
            if let Some(v) = nums(l) {
                if v.len() == 2 {
                    pairs.push([v[0], v[1]]);
                }
            }
        }
    }
    if pairs.is_empty() {
        return Err("no coordinates found; paste a Selig or Lednicer .dat file (x y per line)".into());
    }
    // Lednicer: the first pair holds the point counts of the two surfaces.
    let first = pairs[0];
    if first[0] >= 2.0 && first[1] >= 2.0 && first[0].fract() == 0.0 && first[1].fract() == 0.0 {
        let (nu, nl) = (first[0] as usize, first[1] as usize);
        let rest = &pairs[1..];
        if rest.len() != nu + nl {
            return Err(format!("Lednicer file says {nu} + {nl} points but has {}", rest.len()));
        }
        // Upper from the leading edge to the trailing edge, then lower likewise. Return it in
        // Selig order: trailing edge over the top to the leading edge and back underneath.
        let mut out: Vec<[f64; 2]> = rest[..nu].iter().rev().copied().collect();
        let lower = &rest[nu..];
        let skip = usize::from(lower.first() == out.last());
        out.extend_from_slice(&lower[skip..]);
        pairs = out;
    }
    if pairs.len() < 7 {
        return Err(format!("only {} coordinate points; an airfoil needs at least 7", pairs.len()));
    }
    Ok(pairs)
}

/// Monotone (Fritsch-Carlson) cubic interpolation through (xs, ys), xs strictly increasing.
struct Monotone {
    xs: Vec<f64>,
    ys: Vec<f64>,
    ms: Vec<f64>,
}

impl Monotone {
    fn new(xs: Vec<f64>, ys: Vec<f64>) -> Monotone {
        let n = xs.len();
        let d: Vec<f64> = (0..n - 1).map(|i| (ys[i + 1] - ys[i]) / (xs[i + 1] - xs[i])).collect();
        let mut ms = vec![0.0; n];
        ms[0] = d[0];
        ms[n - 1] = d[n - 2];
        for i in 1..n - 1 {
            ms[i] = if d[i - 1] * d[i] <= 0.0 { 0.0 } else { (d[i - 1] + d[i]) / 2.0 };
        }
        for i in 0..n - 1 {
            if d[i] == 0.0 {
                ms[i] = 0.0;
                ms[i + 1] = 0.0;
                continue;
            }
            let (a, b) = (ms[i] / d[i], ms[i + 1] / d[i]);
            let h = a * a + b * b;
            if h > 9.0 {
                let t = 3.0 / h.sqrt();
                ms[i] = t * a * d[i];
                ms[i + 1] = t * b * d[i];
            }
        }
        Monotone { xs, ys, ms }
    }

    fn eval(&self, x: f64) -> f64 {
        let n = self.xs.len();
        let i = match self.xs.iter().position(|&xi| xi > x) {
            Some(0) => 0,
            Some(i) => i - 1,
            None => n - 2,
        };
        let h = self.xs[i + 1] - self.xs[i];
        let t = ((x - self.xs[i]) / h).clamp(0.0, 1.0);
        let (t2, t3) = (t * t, t * t * t);
        (2.0 * t3 - 3.0 * t2 + 1.0) * self.ys[i] + (t3 - 2.0 * t2 + t) * h * self.ms[i] + (-2.0 * t3 + 3.0 * t2) * self.ys[i + 1] + (t3 - t2) * h * self.ms[i + 1]
    }
}

/// Interpolate one surface (leading edge to trailing edge) at the given x stations. The
/// interpolation runs in u = sqrt(x), in which a round nose's y(u) is smooth, so the leading edge
/// keeps its shape.
fn resample_surface(surface: &[[f64; 2]], xs: &[f64]) -> Result<Vec<[f64; 2]>, String> {
    let mut us: Vec<f64> = Vec::new();
    let mut ys: Vec<f64> = Vec::new();
    for p in surface {
        let u = p[0].max(0.0).sqrt();
        match us.last() {
            Some(&last) if u <= last + 1e-12 => {
                // Not increasing in x: a duplicate or a point folding back at the nose. Keep the
                // first one.
                continue;
            }
            _ => {
                us.push(u);
                ys.push(p[1]);
            }
        }
    }
    if us.len() < 3 {
        return Err("a surface has fewer than 3 distinct points".into());
    }
    let f = Monotone::new(us, ys);
    Ok(xs.iter().map(|&x| [x, f.eval(x.sqrt())]).collect())
}

fn from_points(pts: &[[f64; 2]], name: &str, xs: &[f64]) -> Result<Airfoil, String> {
    // Data that is already in chord units (as in the UIUC database) is kept as it is: a cambered
    // airfoil's nose reaches slightly ahead of its nominal leading edge (0, 0), and moving the
    // smallest-x point to the origin would shift the whole section. Anything else is moved and
    // scaled so its smallest-x point is at the origin and its chord is 1.
    let (imin, _) = pts.iter().enumerate().fold((0, f64::INFINITY), |acc, (i, p)| if p[0] < acc.1 { (i, p[0]) } else { acc });
    let x_min = pts[imin][0];
    let x_max = pts.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max);
    let chord = x_max - x_min;
    if chord <= 0.0 {
        return Err("coordinates have no chord length".into());
    }
    let in_chord_units = x_min.abs() < 0.02 && (x_max - 1.0).abs() < 0.02;
    let norm: Vec<[f64; 2]> = if in_chord_units {
        pts.to_vec()
    } else {
        let (x0, y0) = (pts[imin][0], pts[imin][1]);
        pts.iter().map(|p| [(p[0] - x0) / chord, (p[1] - y0) / chord]).collect()
    };
    // The leading edge is the point nearest the origin.
    let le = (0..norm.len()).min_by(|&i, &j| norm[i][0].hypot(norm[i][1]).total_cmp(&norm[j][0].hypot(norm[j][1]))).unwrap();
    if le == 0 || le == norm.len() - 1 {
        return Err("coordinates must run around the airfoil (Selig order: trailing edge, upper surface, leading edge, lower surface, trailing edge)".into());
    }
    let mut a: Vec<[f64; 2]> = norm[..=le].iter().rev().copied().collect();
    let mut b: Vec<[f64; 2]> = norm[le..].to_vec();
    let mean = |s: &[[f64; 2]]| s.iter().map(|p| p[1]).sum::<f64>() / s.len() as f64;
    if mean(&a) < mean(&b) {
        std::mem::swap(&mut a, &mut b);
    }
    let mut upper = resample_surface(&a, xs)?;
    let mut lower = resample_surface(&b, xs)?;
    upper[0] = [0.0, 0.0];
    lower[0] = [0.0, 0.0];
    let n = xs.len();
    if upper[n - 1][1] < lower[n - 1][1] {
        return Err("the surfaces cross at the trailing edge; check the coordinate order".into());
    }
    Ok(normalise(Airfoil { name: name.to_owned(), upper, lower }))
}

/// Scale and shift so the leading edge sits at (0, 0) and the trailing edge midpoint at x = 1.
fn normalise(mut a: Airfoil) -> Airfoil {
    let n = a.upper.len();
    let x_te = 0.5 * (a.upper[n - 1][0] + a.lower[n - 1][0]);
    if (x_te - 1.0).abs() > 1e-12 && x_te > 0.0 {
        for p in a.upper.iter_mut().chain(a.lower.iter_mut()) {
            p[0] /= x_te;
            p[1] /= x_te;
        }
    }
    a
}

impl Airfoil {
    /// Trailing-edge thickness, in chord units.
    pub fn te_thickness(&self) -> f64 {
        let n = self.upper.len();
        self.upper[n - 1][1] - self.lower[n - 1][1]
    }

    /// Open the trailing edge to at least `gap` (chord units). Each surface moves away from the
    /// camber line in proportion to x, so the leading edge is untouched.
    pub fn with_min_te_gap(mut self, gap: f64) -> Airfoil {
        let extra = gap - self.te_thickness();
        if extra > 0.0 {
            for p in self.upper.iter_mut() {
                p[1] += 0.5 * extra * p[0];
            }
            for p in self.lower.iter_mut() {
                p[1] -= 0.5 * extra * p[0];
            }
        }
        self
    }

    /// Point-by-point blend: `t` = 0 gives `self`, 1 gives `other`. Both must have the same
    /// number of points, which holds for airfoils generated with the same `n`.
    pub fn lerp(&self, other: &Airfoil, t: f64) -> Airfoil {
        let mix = |a: &[[f64; 2]], b: &[[f64; 2]]| a.iter().zip(b).map(|(p, q)| [p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])]).collect();
        let name = if t <= 0.0 {
            self.name.clone()
        } else if t >= 1.0 {
            other.name.clone()
        } else {
            format!("{} / {} blend {:.0} %", self.name, other.name, t * 100.0)
        };
        Airfoil { name, upper: mix(&self.upper, &other.upper), lower: mix(&self.lower, &other.lower) }
    }

    /// Closed contour in Selig order: trailing edge over the upper surface to the leading edge,
    /// then along the lower surface back to the trailing edge. With an open trailing edge the
    /// first and last points differ by the trailing-edge thickness.
    pub fn contour(&self) -> Vec<[f64; 2]> {
        let mut c: Vec<[f64; 2]> = self.upper.iter().rev().copied().collect();
        c.extend_from_slice(&self.lower[1..]);
        c
    }

    /// Thickness and camber distribution, by interpolating both surfaces on a common grid.
    pub fn properties(&self) -> Properties {
        let xs = cosine_spacing(201);
        let up = resample_surface(&self.upper, &xs).unwrap_or_else(|_| self.upper.clone());
        let lo = resample_surface(&self.lower, &xs).unwrap_or_else(|_| self.lower.clone());
        let mut p = Properties { max_thickness: 0.0, max_thickness_x: 0.0, max_camber: 0.0, max_camber_x: 0.0, te_thickness: self.te_thickness(), area: 0.0 };
        for (i, &x) in xs.iter().enumerate() {
            let t = up[i][1] - lo[i][1];
            let c = 0.5 * (up[i][1] + lo[i][1]);
            if t > p.max_thickness {
                p.max_thickness = t;
                p.max_thickness_x = x;
            }
            if c.abs() > p.max_camber.abs() {
                p.max_camber = c;
                p.max_camber_x = x;
            }
            if i > 0 {
                let t0 = up[i - 1][1] - lo[i - 1][1];
                p.area += 0.5 * (t + t0) * (x - xs[i - 1]);
            }
        }
        p
    }
}

/// Geometric properties of a section, in chord units.
#[derive(Clone, Debug, PartialEq)]
pub struct Properties {
    pub max_thickness: f64,
    pub max_thickness_x: f64,
    pub max_camber: f64,
    pub max_camber_x: f64,
    pub te_thickness: f64,
    /// Cross-section area divided by chord squared.
    pub area: f64,
}

/// Resolve an airfoil parameter: a NACA designation, or `custom` with coordinates text.
pub fn resolve(designation: &str, coordinates: &str, n: usize) -> Result<Airfoil, String> {
    let d = designation.trim();
    if d.eq_ignore_ascii_case("custom") || (d.is_empty() && !coordinates.trim().is_empty()) {
        let pts = parse_coordinates(coordinates)?;
        let name = coordinates.lines().map(str::trim).find(|l| !l.is_empty() && l.split_whitespace().any(|w| w.parse::<f64>().is_err())).unwrap_or("custom").to_owned();
        return generate(&Source::Points(pts), &name, n);
    }
    let src = parse_designation(d)?;
    let digits: String = d.chars().filter(char::is_ascii_digit).collect();
    generate(&src, &format!("NACA {digits}"), n)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn designations() {
        assert_eq!(parse_designation("NACA 2412").unwrap(), Source::Naca4 { m: 0.02, p: 0.4, t: 0.12 });
        assert_eq!(parse_designation("naca0012").unwrap(), Source::Naca4 { m: 0.0, p: 0.0, t: 0.12 });
        assert!(matches!(parse_designation("NACA 23012").unwrap(), Source::Naca5 { p_index: 3, reflex: false, .. }));
        assert!(matches!(parse_designation("23112").unwrap(), Source::Naca5 { p_index: 3, reflex: true, .. }));
        assert!(parse_designation("NACA 2012x").is_err());
        assert!(parse_designation("NACA 2000").is_err());
        assert!(parse_designation("NACA 2012").is_err()); // camber without a position
        assert!(parse_designation("NACA 0412").is_err());
        assert!(parse_designation("NACA 21112").is_err());
        assert!(parse_designation("123").is_err());
    }

    #[test]
    fn naca0012_thickness() {
        let a = resolve("NACA 0012", "", 60).unwrap();
        let p = a.properties();
        assert!(close(p.max_thickness, 0.12, 2e-4), "{}", p.max_thickness);
        assert!(close(p.max_thickness_x, 0.30, 0.01), "{}", p.max_thickness_x);
        assert!(close(p.max_camber, 0.0, 1e-12));
        assert!(close(p.te_thickness, 0.0, 1e-12));
        // Symmetric: the surfaces mirror each other.
        for (u, l) in a.upper.iter().zip(&a.lower) {
            assert!(close(u[0], l[0], 1e-12) && close(u[1], -l[1], 1e-12));
        }
    }

    #[test]
    fn naca2412_camber() {
        let p = resolve("NACA 2412", "", 80).unwrap().properties();
        assert!(close(p.max_camber, 0.02, 2e-4), "{}", p.max_camber);
        assert!(close(p.max_camber_x, 0.4, 0.02), "{}", p.max_camber_x);
        assert!(close(p.max_thickness, 0.12, 1e-3), "{}", p.max_thickness);
    }

    #[test]
    fn naca5_camber_is_continuous_and_peaks_near_p() {
        for (d, xm) in [("23012", 0.15), ("23112", 0.15), ("24012", 0.20), ("21012", 0.05)] {
            let src = parse_designation(d).unwrap();
            let mut prev = camber(&src, 0.0).0;
            assert!(close(prev, 0.0, 1e-12));
            let mut best = (0.0, 0.0);
            for i in 1..=2000 {
                let x = i as f64 / 2000.0;
                let (y, _) = camber(&src, x);
                assert!((y - prev).abs() < 1e-3, "{d}: jump at {x}");
                if y > best.0 {
                    best = (y, x);
                }
                prev = y;
            }
            assert!(close(camber(&src, 1.0).0, 0.0, 1e-4), "{d}: camber at TE {}", camber(&src, 1.0).0);
            assert!(close(best.1, xm, 0.02), "{d}: max camber at {}", best.1);
        }
    }

    #[test]
    fn reflexed_camber_turns_up_at_te() {
        let src = parse_designation("23112").unwrap();
        // A reflexed line dips below the chord and comes back up to it at the trailing edge.
        let (_, slope) = camber(&src, 0.99);
        let (_, plain) = camber(&parse_designation("23012").unwrap(), 0.99);
        assert!(slope > plain, "{slope} vs {plain}");
    }

    #[test]
    fn contour_order() {
        let a = resolve("NACA 4412", "", 30).unwrap();
        let c = a.contour();
        assert_eq!(c.len(), 59);
        assert_eq!(c[29], [0.0, 0.0]);
        assert!(c[0][0] > 0.99 && c[58][0] > 0.99);
        assert!(c[10][1] > c[48][1]);
    }

    #[test]
    fn te_gap() {
        let a = resolve("NACA 0012", "", 40).unwrap().with_min_te_gap(0.005);
        assert!(close(a.te_thickness(), 0.005, 1e-12));
        assert_eq!(a.upper[0], [0.0, 0.0]);
        // Already thicker: unchanged.
        let b = a.clone().with_min_te_gap(0.001);
        assert_eq!(a, b);
    }

    fn selig_from(a: &Airfoil) -> String {
        let mut s = String::from("TEST AIRFOIL 123\n");
        for p in a.contour() {
            s += &format!("  {:.7}  {:.7}\n", p[0], p[1]);
        }
        s
    }

    #[test]
    fn selig_roundtrip_matches_generator() {
        let fine = resolve("NACA 4415", "", 120).unwrap();
        let back = resolve("custom", &selig_from(&fine), 40).unwrap();
        assert_eq!(back.name, "TEST AIRFOIL 123");
        let direct = resolve("NACA 4415", "", 40).unwrap();
        let (pb, pd) = (back.properties(), direct.properties());
        assert!(close(pb.max_thickness, pd.max_thickness, 5e-4));
        assert!(close(pb.max_camber, pd.max_camber, 5e-4), "{pb:?} {pd:?}");
        // Every resampled point lies on the original surfaces.
        let fu = resample_surface(&fine.upper, &back.upper.iter().map(|p| p[0]).collect::<Vec<_>>()).unwrap();
        for (p, q) in back.upper.iter().zip(&fu) {
            assert!(close(p[1], q[1], 2e-4), "{p:?} {q:?}");
        }
    }

    #[test]
    fn lednicer_and_single_line() {
        let a = resolve("NACA 2412", "", 25).unwrap();
        let mut s = String::from("NACA 2412 LEDNICER\n\n25. 25.\n\n");
        for p in &a.upper {
            s += &format!("{} {}\n", p[0], p[1]);
        }
        s += "\n";
        for p in &a.lower {
            s += &format!("{} {}\n", p[0], p[1]);
        }
        let b = resolve("custom", &s, 25).unwrap();
        assert!(close(b.properties().max_camber, a.properties().max_camber, 1e-4));
        let one: String = a.contour().iter().map(|p| format!("{} {} ", p[0], p[1])).collect();
        let c = resolve("custom", &one, 25).unwrap();
        assert!(close(c.properties().max_thickness, a.properties().max_thickness, 1e-4));
    }

    #[test]
    fn custom_scaled_and_shifted_is_normalised() {
        let a = resolve("NACA 0012", "", 40).unwrap();
        let s: String = a.contour().iter().map(|p| format!("{} {}\n", 100.0 * p[0] + 5.0, 100.0 * p[1] - 2.0)).collect();
        let b = resolve("custom", &format!("scaled\n{s}"), 40).unwrap();
        assert!(close(b.properties().max_thickness, 0.12, 1e-3));
        assert!(close(b.upper[39][0], 1.0, 1e-9));
    }

    #[test]
    fn bad_coordinates() {
        assert!(resolve("custom", "", 30).is_err());
        assert!(resolve("custom", "hello\n1 2 3\n", 30).is_err());
        assert!(resolve("custom", "0 0 1 0 0.5", 30).is_err());
    }

    #[test]
    fn blend() {
        let a = resolve("NACA 0012", "", 30).unwrap();
        let b = resolve("NACA 0006", "", 30).unwrap();
        let m = a.lerp(&b, 0.5);
        assert!(close(m.properties().max_thickness, 0.09, 1e-3));
        assert_eq!(a.lerp(&b, 0.0), a);
    }
}
