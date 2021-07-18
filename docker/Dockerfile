FROM zhuoqiw/ros-opencv:latest AS opencv

FROM zhuoqiw/ros-pcl:latest AS pcl

FROM ros:galactic

RUN apt-get update && apt-get install -y \
  libgpiod-dev \
  && rm -rf /var/lib/apt/lists/*
  
COPY --from=opencv /opt/opencv /opt/opencv
