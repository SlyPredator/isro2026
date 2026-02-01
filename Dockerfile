FROM osrf/ros:noetic-desktop-full

# Install system dependencies
RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
    apt-get install -y \
    git wget autoconf automake nano \
    python3-dev python3-pip python3-scipy python3-matplotlib \
    ipython3 python3-wxgtk4.0 python3-tk python3-igraph python3-pyx \
    libeigen3-dev libboost-all-dev libsuitesparse-dev \
    doxygen cmake libfreeimage-dev libglew-dev freeglut3-dev\
    curl gnupg2 lsb-release libopencv-dev \
    libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev \
    python3-catkin-tools python3-osrf-pycommon \
    && rm -rf /var/lib/apt/lists/

# Install Intel RealSense SDK
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/librealsense.list && \
    apt-get update && apt-get install -y \
    librealsense2-utils librealsense2-dev ros-noetic-realsense2-camera \
    && rm -rf /var/lib/apt/lists/

ENV WORKSPACE /catkin_ws
RUN mkdir -p $WORKSPACE/src

RUN cd $WORKSPACE && \
    catkin init && \
    catkin config --extend /opt/ros/noetic && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY . $WORKSPACE/src/isro_rovio

RUN sed -i 's/CV_GRAY2RGB/cv::COLOR_GRAY2RGB/g' $WORKSPACE/src/isro_rovio/rovio/include/rovio/ImgUpdate.hpp
RUN sed -i 's/${catkin_LIBRARIES}/${catkin_LIBRARIES} GLEW GL/g' $WORKSPACE/src/isro_rovio/rovio/CMakeLists.txt
RUN sed -i 's/set(ROVIO_CHECK_JACOBIANS 1)/set(ROVIO_CHECK_JACOBIANS 0)/g' $WORKSPACE/src/isro_rovio/rovio/CMakeLists.txt

RUN apt-get update && apt-get install -y ros-noetic-mavros ros-noetic-mavros-extras && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    bash ./install_geographiclib_datasets.sh && \
    rm ./install_geographiclib_datasets.sh

# Build the workspace
RUN cd $WORKSPACE && \
    catkin build rovio kindr rovio_tools -j6 --cmake-args -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=ON

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source $WORKSPACE/devel/setup.bash" >> ~/.bashrc

WORKDIR $WORKSPACE
ENTRYPOINT ["/bin/bash", "-c", "source $WORKSPACE/devel/setup.bash && /bin/bash"]
