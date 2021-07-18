FROM ros:galactic AS build

RUN apt-get update && apt-get install -y \
  unzip \
  wget \
  && rm -rf /var/lib/apt/lists/*

RUN wget -O opencv-4.5.2.zip https://github.com/opencv/opencv/archive/refs/tags/4.5.2.zip
#COPY ./opencv-4.5.2.zip .

RUN unzip -q opencv-4.5.2.zip

RUN cmake \
  -D CMAKE_BUILD_TYPE:STRING=Release \
  -D CMAKE_INSTALL_PREFIX:STRING=/opt/opencv \
  -D BUILD_LIST:STRING=core \
  -D BUILD_TESTS:BOOL=OFF \
  -D BUILD_PERF_TESTS:BOOL=OFF \
  -D BUILD_EXAMPLES:BOOL=OFF \
  -D WITH_OPENCL:BOOL=OFF \
  -D WITH_V4L:BOOL=OFF \
  -D WITH_FFMPEG:BOOL=OFF \
  -D WITH_GSTREAMER:BOOL=OFF \
  -D WITH_GTK:BOOL=OFF \
  -S opencv-4.5.2/ \
  -B build/

RUN cmake --build build/ --target install

FROM ubuntu:20.04

COPY --from=build /opt/opencv /opt/opencv

#ENV LD_LIBRARY_PATH="/opt/opencv/lib:${LD_LIBRARY_PATH}"
