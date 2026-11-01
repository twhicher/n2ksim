# Hosted n2ksim: a simulation per visitor, no gateway, no saving to disk.
FROM rust:1.95-slim-bookworm AS build
WORKDIR /src
COPY Cargo.toml Cargo.lock ./
COPY crates ./crates
RUN cargo build --release --locked -p n2ksim

FROM debian:bookworm-slim
RUN useradd --system --uid 10001 n2ksim
COPY --from=build /src/target/release/n2ksim /usr/local/bin/n2ksim
USER n2ksim
ENV N2KSIM_HOSTED=1
EXPOSE 8090
CMD ["n2ksim", "--hosted"]
