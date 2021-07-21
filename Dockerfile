FROM ros:galactic

RUN apt-get update && apt-get install -y \
  libopencv-dev \
  libpcl-dev \
  libgpiod-dev \
  && rm -rf /var/lib/apt/lists/*
