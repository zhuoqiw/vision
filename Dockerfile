FROM ros:galactic

RUN apt-get update && apt-get install -y \
  wget \
  libopencv-dev \
  libgpiod-dev \
  && rm -rf /var/lib/apt/lists/*
  
RUN wget http://gb.daheng-imaging.com/CN/Software/Cameras/Linux/Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.2106.9091.tar.gz -O - | tar -xz \

RUN ./Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.2106.9091/Galaxy_camera.run << EOF \
  \n\
  Y\n\
  En\n\
  EOF
