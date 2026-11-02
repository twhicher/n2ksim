# Working rules

What this is and how to run it: `README.md`. Conventions: SI and radians inside, knots and degrees only where a name says so, NED world frame, heel positive starboard down, wind angle positive from starboard, rudder positive turns the boat to starboard.

## Never break these

- `n2k` has no dependencies and no simulator types; other projects use it on its own.
- PGN layouts follow canboat's definitions. Change one only with a canboat reference, and keep the round-trip tests.
- Everything the UI shows as "on the bus" comes from the analyser, which sees only frames. Simulator truth is labelled as truth.
- Each health finding says its evidence: `bus`, `design` or `sim`.
- Do not invent sourced figures. Illustrative numbers say so in a comment; sourced ones name the source.
- Seeded randomness: the same seed gives the same run.

## How to work

- Tests first for codec and protocol behaviour (address claim, fast packet, arbitration).
- A new fault or drill comes with a check that finds it; `scenarios::tests` fails if a drill raises nothing.
- `cargo test --release` and `cargo clippy --release --all-targets` clean before committing.
- Ask before adding dependencies. British English, no em dashes in docs.
