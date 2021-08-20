# Install galaxy on ROS
FROM ros:galactic

# linux/amd64 or linux/arm64
ARG TARGETPLATFORM

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  wget \
  && rm -rf /var/lib/apt/lists/*

# Install daheng galaxy
RUN if [ "$TARGETPLATFORM" = "linux/amd64" ]; then \
  wget -O galaxy.tar.gz http://gb.daheng-imaging.com/EN/Software/Cameras/Linux/Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.2107.9261.tar.gz \
  && tar -xzf galaxy.tar.gz \
  && rm galaxy.tar.gz \
  && (printf "\nY\nEn\n" && cat) | ./Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.2107.9261/Galaxy_camera.run \
  && rm -r Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.2107.9261 \
  && mv Galaxy_camera/ /opt/; fi

RUN if [ "$TARGETPLATFORM" = "linux/arm64" ]; then \
  wget -O galaxy.tar.gz http://gb.daheng-imaging.com/EN/Software/Cameras/Linux/Galaxy_Linux-armhf_Gige-U3_32bits-64bits_1.3.2107.9261.tar.gz \
  && tar -xzf galaxy.tar.gz \
  && rm galaxy.tar.gz \
  && (printf "\nY\nEn\n" && cat) | ./Galaxy_Linux-armhf_Gige-U3_32bits-64bits_1.3.2107.9261/Galaxy_camera.run \
  && rm -r Galaxy_Linux-armhf_Gige-U3_32bits-64bits_1.3.2107.9261 \
  && mv Galaxy_camera/ /opt/; fi
