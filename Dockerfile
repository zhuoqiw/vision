FROM ros:galactic

# Install dependencies
RUN apt-get update && apt-get install -y \
  wget \
  libopencv-dev \
  libgpiod-dev \
  && rm -rf /var/lib/apt/lists/*

# Install daheng galaxy
RUN wget -O galaxy.tar.gz http://gb.daheng-imaging.com/CN/Software/Cameras/Linux/Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.2106.9091.tar.gz \
  && tar -xzf galaxy.tar.gz \
  && rm galaxy.tar.gz \
  && (printf "\nY\nEn\n" && cat) | ./Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.2106.9091/Galaxy_camera.run \
  && rm -r Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.2106.9091 \
  && mv Galaxy_camera/ /opt/

# Install paho mqtt c
RUN wget -O paho.mqtt.c.tar.gz https://github.com/eclipse/paho.mqtt.c/archive/refs/tags/v1.3.9.tar.gz \
  && tar -xzf paho.mqtt.c.tar.gz \
  && rm paho.mqtt.c.tar.gz \
  && cmake \
    -D CMAKE_BUILD_TYPE=Release \
    -D PAHO_BUILD_STATIC=ON \
    -D PAHO_ENABLE_TESTING=OFF \
    -D PAHO_WITH_SSL=OFF \
    -D PAHO_HIGH_PERFORMANCE=ON \
    -S paho.mqtt.c-1.3.9/ \
    -B build/ \
  && cmake --build build/ --target install \
  && rm -r paho.mqtt.c-1.3.9 build

# Install paho mqtt cpp
RUN wget -O paho.mqtt.cpp.tar.gz https://github.com/eclipse/paho.mqtt.cpp/archive/refs/tags/v1.2.0.tar.gz \
  && tar -xzf paho.mqtt.cpp.tar.gz \
  && rm paho.mqtt.cpp.tar.gz \
  && cmake \
    -D CMAKE_BUILD_TYPE=Release \
    -D PAHO_BUILD_STATIC=ON \
    -D PAHO_WITH_SSL=OFF \
    -S paho.mqtt.cpp-1.2.0/ \
    -B build/ \
  && cmake --build build/ --target install \
  && rm -r paho.mqtt.cpp-1.2.0 build

# Install nlohmann json
RUN wget https://github.com/nlohmann/json/releases/download/v3.9.1/json.hpp \
  && mkdir -p /usr/local/include/nlohmann \
  && mv json.hpp /usr/local/include/nlohmann/json.hpp

# Install pcl
RUN wget -O pcl.tar.gz https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.12.0.tar.gz \
  && tar -xzf pcl.tar.gz \
  && rm pcl.tar.gz \
  && cmake \
    -D CMAKE_BUILD_TYPE:STRING=Release \
    -D BUILD_CUDA:BOOL=OFF \
    -D BUILD_GPU:BOOL=OFF \
    -D BUILD_apps:BOOL=OFF \
    -D BUILD_benchmarks:BOOL=OFF \
    -D BUILD_examples:BOOL=OFF \
    -D BUILD_global_tests:BOOL=OFF \
    -D BUILD_io:BOOL=OFF \
    -D BUILD_keypoints:BOOL=OFF \
    -D BUILD_outofcore:BOOL=OFF \
    -D BUILD_people:BOOL=OFF \
    -D BUILD_recognition:BOOL=OFF \
    -D BUILD_registration:BOOL=OFF \
    -D BUILD_simulation:BOOL=OFF \
    -D BUILD_stereo:BOOL=OFF \
    -D BUILD_surface:BOOL=OFF \
    -D BUILD_tools:BOOL=OFF \
    -D BUILD_tracking:BOOL=OFF \
    -D BUILD_visualization:BOOL=OFF \
    -D WITH_CUDA:BOOL=OFF \
    -D WITH_DAVIDSDK:BOOL=OFF \
    -D WITH_DOCS:BOOL=OFF \
    -D WITH_DSSDK:BOOL=OFF \
    -D WITH_ENSENSO:BOOL=OFF \
    -D WITH_LIBUSB:BOOL=OFF \
    -D WITH_OPENGL:BOOL=OFF \
    -D WITH_OPENMP:BOOL=OFF \
    -D WITH_OPENNI:BOOL=OFF \
    -D WITH_OPENNI2:BOOL=OFF \
    -D WITH_PCAP:BOOL=OFF \
    -D WITH_PNG:BOOL=OFF \
    -D WITH_QHULL:BOOL=OFF \
    -D WITH_QT:BOOL=OFF \
    -D WITH_RSSDK:BOOL=OFF \
    -D WITH_RSSDK2:BOOL=OFF \
    -D WITH_VTK:BOOL=OFF \
    -S pcl-pcl-1.12.0/ \
    -B build/ \
  && cmake --build build/ --target install \
  && rm -r pcl-pcl-1.12.0 build
