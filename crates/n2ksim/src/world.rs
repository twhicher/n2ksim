//! The boat and its surroundings: what the sensors measure.
//!
//! A deliberately simple model so the bus has believable, coupled data:
//! first-order (Nomoto) yaw, surge towards a placeholder polar, roll as a
//! damped oscillator towards a heel set by apparent wind, waves as three
//! sinusoids, gusts and shifts as filtered noise, a tidal stream, and a
//! seabed with a slope. It is not the autopilot's four-dof model; every
//! coefficient is illustrative, not data for a real boat.
//!
//! Conventions follow the autopilot's `docs/conventions.md`: SI and radians
//! inside, knots and degrees only in config keys that say so; NED world
//! frame; heel positive starboard down; wind angle positive from starboard;
//! rudder positive turns the boat to starboard.

use crate::rng::{Ou, Rng};
use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};

pub const KN: f64 = 1852.0 / 3600.0;
const EARTH_R: f64 = 6_371_000.0;

pub fn wrap_pi(a: f64) -> f64 {
    let mut x = (a + PI).rem_euclid(TAU) - PI;
    if x <= -PI {
        x += TAU;
    }
    x
}

pub fn wrap_2pi(a: f64) -> f64 {
    a.rem_euclid(TAU)
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct EnvCfg {
    pub tws_kn: f64,
    /// Direction the wind blows from, true.
    pub twd_deg: f64,
    /// Gust strength: standard deviation as a percentage of wind speed.
    pub gust_pct: f64,
    /// Standard deviation of direction wander, degrees.
    pub shift_deg: f64,
    pub current_kn: f64,
    /// Direction the stream flows towards, true.
    pub current_set_deg: f64,
    pub wave_height_m: f64,
    pub wave_period_s: f64,
    /// Direction the waves come from, true.
    pub wave_dir_deg: f64,
    /// Depth at the start position.
    pub depth_m: f64,
    /// How fast it shoals, metres per kilometre, and towards which bearing.
    pub shoal_m_per_km: f64,
    pub shoal_dir_deg: f64,
    /// Magnetic variation, east positive.
    pub variation_deg: f64,
    pub water_temp_c: f64,
    pub air_temp_c: f64,
    pub humidity_pct: f64,
    pub pressure_hpa: f64,
    pub start_lat: f64,
    pub start_lon: f64,
    /// Unix time at the start of the run.
    pub start_utc: f64,
    pub seed: u64,
}

impl Default for EnvCfg {
    fn default() -> Self {
        EnvCfg {
            tws_kn: 14.0,
            twd_deg: 225.0,
            gust_pct: 10.0,
            shift_deg: 4.0,
            current_kn: 0.8,
            current_set_deg: 90.0,
            wave_height_m: 0.8,
            wave_period_s: 5.0,
            wave_dir_deg: 230.0,
            depth_m: 18.0,
            shoal_m_per_km: 2.0,
            shoal_dir_deg: 0.0,
            variation_deg: -1.0,
            water_temp_c: 15.5,
            air_temp_c: 13.0,
            humidity_pct: 78.0,
            pressure_hpa: 1015.0,
            // The Solent, off Cowes.
            start_lat: 50.775,
            start_lon: -1.300,
            start_utc: 1_793_534_400.0, // 1 November 2026, 10:00 UTC
            seed: 1,
        }
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum HelmMode {
    /// The rudder follows the manual setting.
    Manual,
    /// A built-in helmsman holds `course_deg` (true), so the boat sails
    /// while nothing else is steering.
    Hold,
    /// The rudder follows angle orders received by the rudder drive on
    /// PGN 127245, from the autopilot through the gateway.
    External,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct HelmCfg {
    pub mode: HelmMode,
    pub course_deg: f64,
    pub rudder_deg: f64,
    /// With no angle order for this long, the drive holds its last order
    /// and raises an alarm.
    pub order_timeout_s: f64,
}

impl Default for HelmCfg {
    fn default() -> Self {
        HelmCfg {
            mode: HelmMode::Hold,
            course_deg: 160.0,
            rudder_deg: 0.0,
            order_timeout_s: 1.0,
        }
    }
}

/// Fixed boat constants. Illustrative values for a 30 to 36 ft yacht.
pub struct Boat {
    pub hull_speed_kn: f64,
    pub rudder_max: f64,
    pub ram_rate: f64,
    /// Nomoto gain per unit speed (1/s per rad per m/s) and time constant.
    pub nomoto_k: f64,
    pub nomoto_t: f64,
    pub weather_helm: f64,
    pub surge_tau: f64,
    pub roll_period: f64,
    pub roll_damping: f64,
    pub max_heel: f64,
    pub mast_height: f64,
}

pub const BOAT: Boat = Boat {
    hull_speed_kn: 7.6,
    rudder_max: 0.52, // 30 degrees
    ram_rate: 0.105,  // 6 degrees per second
    nomoto_k: 0.12,
    nomoto_t: 2.0,
    weather_helm: 0.02,
    surge_tau: 6.0,
    roll_period: 4.0,
    roll_damping: 0.3,
    max_heel: 0.49, // 28 degrees
    mast_height: 15.0,
};

/// What is really happening. Only the sensors read this; the bus carries
/// their view of it. SI and radians.
#[derive(Clone, Debug, Default)]
pub struct State {
    pub north: f64,
    pub east: f64,
    pub heading: f64,
    pub yaw_rate: f64,
    pub heel: f64,
    pub roll_rate: f64,
    pub pitch: f64,
    pub pitch_rate: f64,
    pub heave: f64,
    pub bsp: f64,
    pub leeway: f64,
    pub sog: f64,
    pub cog: f64,
    pub tws: f64,
    pub twd: f64,
    pub twa: f64,
    pub aws: f64,
    pub awa: f64,
    pub depth: f64,
    pub rudder: f64,
    pub rudder_order: f64,
    pub log_m: f64,
    pub ram_current: f64,
}

pub struct World {
    pub env: EnvCfg,
    pub helm: HelmCfg,
    pub s: State,
    pub t: f64,
    rng: Rng,
    gust: Ou,
    shift: Ou,
    waves: [(f64, f64); 3], // (period factor, phase)
    /// Last angle order from the bus and when it arrived.
    pub external_order: Option<(f64, f64)>,
    pub external_timed_out: bool,
    hold_int: f64,
}

impl World {
    pub fn new(env: EnvCfg, helm: HelmCfg) -> Self {
        let mut rng = Rng::new(env.seed);
        let waves = [0.8, 1.0, 1.25].map(|k| (k, rng.range(0.0, TAU)));
        let mut w = World {
            s: State {
                heading: helm.course_deg.to_radians(),
                bsp: 5.5 * KN,
                ..Default::default()
            },
            env,
            helm,
            t: 0.0,
            rng,
            gust: Ou::default(),
            shift: Ou::default(),
            waves,
            external_order: None,
            external_timed_out: false,
            hold_int: 0.0,
        };
        w.update_wind(0.0);
        w.s.depth = w.depth_at(0.0, 0.0);
        w
    }

    pub fn lat_lon(&self) -> (f64, f64) {
        let lat0 = self.env.start_lat.to_radians();
        (
            self.env.start_lat + (self.s.north / EARTH_R).to_degrees(),
            self.env.start_lon + (self.s.east / (EARTH_R * lat0.cos())).to_degrees(),
        )
    }

    pub fn utc(&self) -> f64 {
        self.env.start_utc + self.t
    }

    /// Seabed depth below the surface at a position, before waves.
    pub fn depth_at(&self, north: f64, east: f64) -> f64 {
        let d = self.env.shoal_dir_deg.to_radians();
        let along = north * d.cos() + east * d.sin();
        let bumps = 1.5 * (north / 310.0).sin() * (east / 420.0).cos();
        (self.env.depth_m - self.env.shoal_m_per_km * along / 1000.0 + bumps).max(0.3)
    }

    fn update_wind(&mut self, dt: f64) {
        let g = self.gust.step(dt, 20.0, self.env.gust_pct / 100.0, &mut self.rng);
        let sh = self.shift.step(dt, 90.0, self.env.shift_deg.to_radians(), &mut self.rng);
        self.s.tws = (self.env.tws_kn * KN * (1.0 + g)).max(0.0);
        self.s.twd = wrap_2pi(self.env.twd_deg.to_radians() + sh);
        // Ground velocity of the boat.
        let vn = self.s.sog * self.s.cog.cos();
        let ve = self.s.sog * self.s.cog.sin();
        // Wind velocity (towards), ground referenced.
        let wn = -self.s.tws * self.s.twd.cos();
        let we = -self.s.tws * self.s.twd.sin();
        let (an, ae) = (wn - vn, we - ve);
        // Into body axes, then the masthead swinging with roll: the top of
        // the mast moves to starboard at h * p, so the air seems to come
        // from starboard by that much less.
        let (sh_, ch) = self.s.heading.sin_cos();
        let ax = an * ch + ae * sh_;
        let ay = -an * sh_ + ae * ch - BOAT.mast_height * self.s.roll_rate;
        self.s.aws = (ax * ax + ay * ay).sqrt();
        self.s.awa = if self.s.aws > 1e-6 { wrap_pi((-ay).atan2(-ax)) } else { 0.0 };
        self.s.twa = wrap_pi(self.s.twd - self.s.heading);
    }

    /// Placeholder polar: knots for a true wind speed and angle.
    pub fn polar_kn(tws_kn: f64, twa: f64) -> f64 {
        let a = twa.abs().to_degrees();
        let vmax = BOAT.hull_speed_kn * (1.0 - (-tws_kn / 7.0).exp()) * 1.05;
        let shape = if a < 30.0 {
            0.1
        } else if a < 45.0 {
            0.1 + 0.65 * (a - 30.0) / 15.0
        } else if a < 110.0 {
            0.75 + 0.25 * (a - 45.0) / 65.0
        } else {
            1.0 - 0.2 * (a - 110.0) / 70.0
        };
        vmax * shape
    }

    fn wave_sum(&self, t: f64, deriv: bool) -> f64 {
        let a = 1.0 / 3f64.sqrt();
        let period = self.env.wave_period_s.max(1.0);
        self.waves
            .iter()
            .map(|(k, ph)| {
                let w = TAU / (period * k);
                if deriv {
                    a * w * (w * t + ph).cos()
                } else {
                    a * (w * t + ph).sin()
                }
            })
            .sum()
    }

    /// One physics step.
    pub fn step(&mut self, dt: f64) {
        self.t += dt;
        let t = self.t;
        self.update_wind(dt);
        let w0 = self.wave_sum(t, false);
        let w1 = self.wave_sum(t, true);
        let s = &mut self.s;

        // Rudder order from whoever steers, then the ram's rate limit.
        let order = match self.helm.mode {
            HelmMode::Manual => self.helm.rudder_deg.to_radians(),
            HelmMode::Hold => {
                let err = wrap_pi(self.helm.course_deg.to_radians() - s.heading);
                self.hold_int = (self.hold_int + 0.15 * err * dt).clamp(-0.2, 0.2);
                1.2 * err + self.hold_int - 4.0 * s.yaw_rate
            }
            HelmMode::External => match self.external_order {
                Some((at, a)) => {
                    self.external_timed_out = t - at > self.helm.order_timeout_s;
                    a
                }
                None => {
                    self.external_timed_out = true;
                    s.rudder_order
                }
            },
        };
        s.rudder_order = order.clamp(-BOAT.rudder_max, BOAT.rudder_max);
        let before = s.rudder;
        let step = (s.rudder_order - s.rudder).clamp(-BOAT.ram_rate * dt, BOAT.ram_rate * dt);
        s.rudder += step;
        let rate = (s.rudder - before).abs() / dt;
        s.ram_current = 0.3 + 3.0 * rate / BOAT.ram_rate + 0.15 * s.bsp * s.bsp * s.rudder.abs();

        // Waves: elevation, and slopes that roll, pitch and yaw the boat.
        let hs = self.env.wave_height_m;
        let sigma = hs / 4.0;
        let rel = wrap_pi(self.env.wave_dir_deg.to_radians() - s.heading);
        s.heave = -sigma * 2f64.sqrt() * w0;
        let roll_wave = 0.06 * hs * rel.sin() * w0;
        let pitch_wave = 0.04 * hs * rel.cos() * w0;
        let pitch_rate = 0.04 * hs * rel.cos() * w1;
        let yaw_wave = 0.025 * hs * (2.0 * rel).sin() * w1;

        // Yaw (Nomoto first order) with weather helm from heel.
        let u = s.bsp.max(0.3);
        let r_ss = BOAT.nomoto_k * u * s.rudder - BOAT.weather_helm * u * s.heel + yaw_wave;
        s.yaw_rate += (r_ss - s.yaw_rate) * dt / BOAT.nomoto_t;
        s.heading = wrap_2pi(s.heading + s.yaw_rate * dt);

        // Surge towards the polar, less rudder drag.
        let target = World::polar_kn(s.tws / KN, s.twa) * KN * (1.0 - 0.6 * s.rudder * s.rudder);
        s.bsp += (target - s.bsp) * dt / BOAT.surge_tau;
        s.bsp = s.bsp.max(0.0);

        // Heel towards the apparent wind's heeling force, as a damped oscillator.
        let aws_kn = s.aws / KN;
        let force = (aws_kn / 16.0).powi(2) * s.awa.sin().abs() * 1.2;
        let heel_ss = -s.awa.signum() * BOAT.max_heel * force.tanh() + roll_wave;
        let wn = TAU / BOAT.roll_period;
        let p_dot = wn * wn * (heel_ss - s.heel) - 2.0 * BOAT.roll_damping * wn * s.roll_rate;
        s.roll_rate += p_dot * dt;
        s.heel += s.roll_rate * dt;
        s.pitch = pitch_wave;
        s.pitch_rate = pitch_rate;

        // Leeway follows heel: sliding to leeward.
        s.leeway = 0.2 * s.heel;

        // Ground track: water track plus the stream.
        let ct = s.heading + s.leeway;
        let vw = s.bsp / s.leeway.cos();
        let cur = self.env.current_kn * KN;
        let cs = self.env.current_set_deg.to_radians();
        let vn = vw * ct.cos() + cur * cs.cos();
        let ve = vw * ct.sin() + cur * cs.sin();
        s.north += vn * dt;
        s.east += ve * dt;
        s.sog = (vn * vn + ve * ve).sqrt();
        s.cog = wrap_2pi(ve.atan2(vn));
        s.log_m += s.bsp * dt;
        let (n, e) = (s.north, s.east);
        let heave = s.heave;
        self.s.depth = self.depth_at(n, e) + heave;
    }

    pub fn reset(&mut self) {
        *self = World::new(self.env.clone(), self.helm.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrapping() {
        assert!((wrap_pi(3.0 * PI / 2.0) + PI / 2.0).abs() < 1e-12);
        assert!((wrap_2pi(-0.1) - (TAU - 0.1)).abs() < 1e-12);
    }

    #[test]
    fn apparent_wind_sign_on_starboard_tack() {
        // Wind from 225, heading 180: wind from the starboard side.
        let mut w = World::new(
            EnvCfg {
                gust_pct: 0.0,
                shift_deg: 0.0,
                wave_height_m: 0.0,
                current_kn: 0.0,
                ..Default::default()
            },
            HelmCfg {
                course_deg: 180.0,
                ..Default::default()
            },
        );
        for _ in 0..6000 {
            w.step(0.01);
        }
        assert!(w.s.twa > 0.0 && w.s.awa > 0.0, "twa {} awa {}", w.s.twa, w.s.awa);
        assert!(w.s.awa < w.s.twa, "apparent is forward of true when sailing");
        assert!(w.s.heel < 0.0, "starboard tack heels to port");
        assert!((wrap_pi(w.s.heading - PI)).abs() < 0.05, "holds course");
        assert!(w.s.bsp / KN > 4.5, "{} kn at twa {}", w.s.bsp / KN, w.s.twa.to_degrees());
    }
}
