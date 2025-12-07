# isaac-sim-headless.dockerfile
FROM nvcr.io/nvidia/isaac-sim:2024.2.1

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Copy assets and scenes
COPY assets/scenes/ /assets/scenes/

# Set working directory
WORKDIR /workspace

# Expose necessary ports
EXPOSE 55555 55556 11345 11346

# Isaac Sim headless entrypoint
ENTRYPOINT ["/isaac-sim/python.sh", "-m", "omni.isaac.kit", "--noWindow", "--/renderer/enabled=false"]