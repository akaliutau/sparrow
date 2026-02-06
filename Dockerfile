# Stage 1: Build
FROM rust:latest AS builder
WORKDIR /usr/src/app
COPY . .
# Build release binary
RUN cargo build --release

# Stage 2: Runtime (Lightweight)
FROM debian:bookworm-slim
WORKDIR /app

# Install ca-certificates (needed for GCP interactions) and clean up
RUN apt-get update && apt-get install -y ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Copy the binary from builder (replace 'my_app_name' with your actual binary name from Cargo.toml)
COPY --from=builder /usr/src/app/target/release/sparrow /app/sparrow
COPY --from=builder /usr/src/app/target/release/libsparrow.rlib /app/libsparrow.rlib
RUN chmod +x /app/sparrow
RUN chmod +x /app/libsparrow.rlib

# Create a wrapper script to handle dynamic logic
COPY wrapper.sh /app/wrapper.sh
RUN chmod +x /app/wrapper.sh

RUN mkdir -p /mnt/share/inputs /mnt/share/outputs

ENTRYPOINT ["/app/wrapper.sh"]
