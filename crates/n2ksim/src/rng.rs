//! Seeded random numbers: the same seed gives the same run.

#[derive(Clone)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng(seed ^ 0x9E37_79B9_7F4A_7C15)
    }

    /// SplitMix64.
    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in [0, 1).
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    pub fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.uniform()
    }

    /// Standard normal (Box-Muller).
    pub fn gauss(&mut self) -> f64 {
        let u1 = self.uniform().max(1e-12);
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
    }

    pub fn chance(&mut self, p: f64) -> bool {
        self.uniform() < p
    }
}

/// Ornstein-Uhlenbeck process: zero mean, standard deviation `sigma`,
/// correlation time `tau` seconds.
#[derive(Clone, Default)]
pub struct Ou {
    pub x: f64,
}

impl Ou {
    pub fn step(&mut self, dt: f64, tau: f64, sigma: f64, rng: &mut Rng) -> f64 {
        if tau <= 0.0 || sigma <= 0.0 {
            self.x = 0.0;
            return 0.0;
        }
        let a = (-dt / tau).exp();
        self.x = a * self.x + sigma * (1.0 - a * a).sqrt() * rng.gauss();
        self.x
    }
}
