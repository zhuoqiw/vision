FROM ros:galactic

#RUN apt-get update && apt-get install -y \
#  wget \
#  && rm -rf /var/lib/apt/lists/*

#RUN wget -O galaxy.tar.gz http://gb.daheng-imaging.com/CN/Software/Cameras/Linux/Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.2106.9091.tar.gz \
#  && tar -xzf galaxy.tar.gz \
#  && rm galaxy.tar.gz \
#  && (printf "\nY\nEn\n" && cat) | ./Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.2106.9091/Galaxy_camera.run \
#  && rm -r ./Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.2106.9091 \
#  && mv Galaxy_camera/ /opt/

WORKDIR /ros
COPY . camera_galaxy/
#RUN colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
#RUN sed -i 's+opt/ros/$ROS_DISTRO+ros/install+g' /ros_entrypoint.sh
