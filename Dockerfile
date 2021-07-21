FROM ros:galactic

RUN apt-get update && apt-get install -y \
  libopencv-dev \
  libgpiod-dev \
  && rm -rf /var/lib/apt/lists/*
