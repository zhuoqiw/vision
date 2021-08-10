FROM ros:galactic

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  wget \
  # GPIO
  libgpiod-dev \
  # PCL dependencies
  libboost-dev \
  libboost-filesystem-dev \
  libboost-date-time-dev \
  libboost-iostreams-dev \
  libboost-system-dev \
  libflann-dev \
  && rm -rf /var/lib/apt/lists/*

# Install daheng galaxy
RUN wget -O galaxy.tar.gz http://gb.daheng-imaging.com/CN/Software/Cameras/Linux/Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.2106.9091.tar.gz \
  && tar -xzf galaxy.tar.gz \
  && rm galaxy.tar.gz \
  && (printf "\nY\nEn\n" && cat) | ./Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.2106.9091/Galaxy_camera.run \
  && rm -r Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.2106.9091 \
  && mv Galaxy_camera/ /opt/

# Install nlohmann json
RUN wget https://github.com/nlohmann/json/releases/download/v3.9.1/json.hpp \
  && mkdir -p /usr/local/include/nlohmann \
  && mv json.hpp /usr/local/include/nlohmann/json.hpp

# Build paho mqtt c
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

# Build paho mqtt cpp
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

# Build opencv
RUN wget -O opencv.tar.gz https://github.com/opencv/opencv/archive/refs/tags/4.5.2.tar.gz \
  && tar -xzf opencv.tar.gz \
  && rm opencv.tar.gz \
  && cmake \
    -D CMAKE_BUILD_TYPE:STRING=Release \
    #-D CMAKE_INSTALL_PREFIX:STRING=/opt/opencv \
    -D BUILD_LIST:STRING=core,imgproc,calib3d \
    -D BUILD_TESTS:BOOL=OFF \
    -D BUILD_PERF_TESTS:BOOL=OFF \
    -D BUILD_EXAMPLES:BOOL=OFF \
    -D BUILD_opencv_apps=OFF \
    -D WITH_1394:BOOL=OFF \
    -D WITH_ADE:BOOL=OFF \
    -D WITH_ARAVIS:BOOL=OFF \
    -D WITH_CLP:BOOL=OFF \
    -D WITH_CUDA:BOOL=OFF \
    -D WITH_EIGEN:BOOL=OFF \
    -D WITH_FFMPEG:BOOL=OFF \
    -D WITH_FREETYPE:BOOL=OFF \
    -D WITH_GDAL:BOOL=OFF \
    -D WITH_GDCM:BOOL=OFF \
    -D WITH_GPHOTO2:BOOL=OFF \
    -D WITH_GSTREAMER:BOOL=OFF \
    -D WITH_GTK:BOOL=OFF \
    -D WITH_GTK_2_X:BOOL=OFF \
    -D WITH_HALIDE:BOOL=OFF \
    -D WITH_HPX:BOOL=OFF \
    -D WITH_IMGCODEC_HDR:BOOL=OFF \
    -D WITH_IMGCODEC_PFM:BOOL=OFF \
    -D WITH_IMGCODEC_PXM:BOOL=OFF \
    -D WITH_IMGCODEC_SUNRASTER:BOOL=OFF \
    -D WITH_INF_ENGINE:BOOL=OFF \
    -D WITH_IPP:BOOL=OFF \
    -D WITH_ITT:BOOL=OFF \
    -D WITH_JASPER:BOOL=OFF \
    -D WITH_JPEG:BOOL=OFF \
    -D WITH_LAPACK:BOOL=OFF \
    -D WITH_LIBREALSENSE:BOOL=OFF \
    -D WITH_MFX:BOOL=OFF \
    -D WITH_NGRAPH:BOOL=OFF \
    -D WITH_ONNX:BOOL=OFF \
    -D WITH_OPENCL:BOOL=OFF \
    -D WITH_OPENCLAMDBLAS:BOOL=OFF \
    -D WITH_OPENCLAMDFFT:BOOL=OFF \
    -D WITH_OPENCL_SVM:BOOL=OFF \
    -D WITH_OPENEXR:BOOL=OFF \
    -D WITH_OPENGL:BOOL=OFF \
    -D WITH_OPENJPEG:BOOL=OFF \
    -D WITH_OPENMP:BOOL=OFF \
    -D WITH_OPENNI:BOOL=OFF \
    -D WITH_OPENNI2:BOOL=OFF \
    -D WITH_OPENVX:BOOL=OFF \
    -D WITH_PLAIDML:BOOL=OFF \
    -D WITH_PNG:BOOL=OFF \
    -D WITH_PROTOBUF:BOOL=OFF \
    -D WITH_PTHREADS_PF:BOOL=OFF \
    -D WITH_PVAPI:BOOL=OFF \
    -D WITH_QT:BOOL=OFF \
    -D WITH_QUIRC:BOOL=OFF \
    -D WITH_TBB:BOOL=OFF \
    -D WITH_TIFF:BOOL=OFF \
    -D WITH_UEYE:BOOL=OFF \
    -D WITH_V4L:BOOL=OFF \
    -D WITH_VA:BOOL=OFF \
    -D WITH_VA_INTEL:BOOL=OFF \
    -D WITH_VTK:BOOL=OFF \
    -D WITH_VULKAN:BOOL=OFF \
    -D WITH_WEBP:BOOL=OFF \
    -D WITH_XIMEA:BOOL=OFF \
    -D WITH_XINE:BOOL=OFF \
    -S opencv-4.5.2/ \
    -B build/ \
  && cmake --build build/ --target install \
  && rm -r opencv-4.5.2 build

# Build pcl
RUN wget -O pcl.tar.gz https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.12.0.tar.gz \
  && tar -xzf pcl.tar.gz \
  && rm pcl.tar.gz \
  && cmake \
    -D CMAKE_BUILD_TYPE:STRING=Release \
    #-D CMAKE_INSTALL_PREFIX:STRING=/opt/pcl \
    -D BUILD_2d:BOOL=ON \
    -D BUILD_CUDA:BOOL=OFF \
    -D BUILD_GPU:BOOL=OFF \
    -D BUILD_apps:BOOL=OFF \
    -D BUILD_benchmarks:BOOL=OFF \
    -D BUILD_common:BOOL=ON \
    -D BUILD_examples:BOOL=OFF \
    -D BUILD_features:BOOL=ON \
    -D BUILD_filters:BOOL=ON \
    -D BUILD_geometry:BOOL=ON \
    -D BUILD_global_tests:BOOL=OFF \
    -D BUILD_io:BOOL=OFF \
    -D BUILD_kdtree:BOOL=ON \
    -D BUILD_keypoints:BOOL=OFF \
    -D BUILD_ml:BOOL=ON \
    -D BUILD_octree:BOOL=ON \
    -D BUILD_outofcore:BOOL=OFF \
    -D BUILD_people:BOOL=OFF \
    -D BUILD_recognition:BOOL=OFF \
    -D BUILD_registration:BOOL=OFF \
    -D BUILD_sample_consensus:BOOL=ON \
    -D BUILD_search:BOOL=ON \
    -D BUILD_segmentation:BOOL=ON \
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
