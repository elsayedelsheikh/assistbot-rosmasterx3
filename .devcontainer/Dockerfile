FROM althack/ros:humble-gazebo-nvidia

ENV DEBIAN_FRONTEND=noninteractive
#################################################################################################
### Install WSLg dependencies and vGPU acceleration
# RUN apt-get update && apt install -y software-properties-common
# RUN add-apt-repository ppa:oibaf/graphics-drivers -y
RUN apt-get update \
   && apt-get -y install \
          vainfo \
          mesa-va-drivers \
          mesa-utils \
   # Clean up
        && apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*
ENV LIBVA_DRIVER_NAME=d3d12
ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib
CMD vainfo --display drm --device /dev/dri/card0
#################################################################################################
ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
# ARG WORKSPACE
# RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# WORKDIR $WORKSPACE
