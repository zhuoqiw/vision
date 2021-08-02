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
  && mv json.hpp /usr/local/include/nlohmann/json.hpp
