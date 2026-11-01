//! Voltage along the backbone.
//!
//! Each device draws LEN × 50 mA at its tee, through its drop cable (Micro).
//! Current flows from the power tap out to each side. The voltage lost
//! between the tap and a tee at distance `x` is the sum, over every load on
//! the same side, of its current times the resistance of the stretch of
//! backbone the two share: `2 ρ min(x, x_j)` (out and back). Devices that are
//! off draw nothing, which is what makes brownouts cycle.

use crate::network::{Backbone, Cable};

pub const LEN_AMPS: f64 = 0.05;
/// Below this a device resets (NMEA 2000 devices run on 9 to 16 V).
pub const MIN_VOLTS: f64 = 9.0;

pub struct Load {
    pub tee_m: f64,
    pub drop_m: f64,
    pub amps: f64,
}

/// Voltage at each load's connector. All zero if the network is unpowered.
pub fn voltages(bb: &Backbone, loads: &[Load]) -> Vec<f64> {
    if !bb.powered || bb.power_tap_m < 0.0 || bb.power_tap_m > bb.length_m {
        return vec![0.0; loads.len()];
    }
    let rho = bb.cable.ohm_per_m();
    let rho_drop = Cable::Micro.ohm_per_m();
    let side = |x: f64| (x - bb.power_tap_m).signum();
    loads
        .iter()
        .map(|li| {
            let xi = (li.tee_m - bb.power_tap_m).abs();
            let si = side(li.tee_m);
            let backbone_drop: f64 = loads
                .iter()
                .filter(|lj| side(lj.tee_m) == si)
                .map(|lj| lj.amps * 2.0 * rho * xi.min((lj.tee_m - bb.power_tap_m).abs()))
                .sum();
            let drop = li.amps * 2.0 * rho_drop * li.drop_m;
            (bb.supply_v - backbone_drop - drop).max(0.0)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shared_segments_and_sides() {
        let bb = Backbone {
            length_m: 20.0,
            cable: Cable::Micro,
            power_tap_m: 0.0,
            supply_v: 12.0,
            ..Default::default()
        };
        let loads = [
            Load { tee_m: 10.0, drop_m: 0.0, amps: 1.0 },
            Load { tee_m: 20.0, drop_m: 0.0, amps: 1.0 },
        ];
        let v = voltages(&bb, &loads);
        let r = 2.0 * Cable::Micro.ohm_per_m();
        // First 10 m carries 2 A, the next 10 m carries 1 A.
        assert!((v[0] - (12.0 - 2.0 * r * 10.0)).abs() < 1e-9);
        assert!((v[1] - (12.0 - 2.0 * r * 10.0 - 1.0 * r * 10.0)).abs() < 1e-9);
        // Moving the tap to the middle splits the load.
        let mid = Backbone { power_tap_m: 15.0, ..bb };
        let v = voltages(&mid, &loads);
        assert!((v[0] - (12.0 - r * 5.0)).abs() < 1e-9);
        assert!((v[1] - (12.0 - r * 5.0)).abs() < 1e-9);
    }
}
