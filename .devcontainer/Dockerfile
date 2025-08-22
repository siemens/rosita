FROM ros:jazzy-ros-base@sha256:9749355a0760334fa5ea3c660f2de1ecc5e8a5af8047e4bbaa9afa8a7843da80

# specify to accept the default answer for all questions
ENV DEBIAN_FRONTEND=noninteractive

# Configure apt to allow unauthenticated packages and ignore specific repository
RUN echo 'Acquire::AllowInsecureRepositories "true";' > /etc/apt/apt.conf.d/99allow-insecure && \
    echo 'APT::Get::AllowUnauthenticated "true";' >> /etc/apt/apt.conf.d/99allow-insecure && \
    echo 'Acquire::Check-Valid-Until "false";' >> /etc/apt/apt.conf.d/99allow-insecure


# using bash as shell
SHELL ["/bin/bash", "-c"]

# Create non-root user 'ros'
ENV USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN echo "Renaming ubuntu user to ros" \
    && usermod -l ${USERNAME} ubuntu \
    && groupmod -n ${USERNAME} ubuntu \
    && usermod -d /home/${USERNAME} -m ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME};

# install packages
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    # Install python build dependencies
    build-essential \
    zlib1g-dev \
    libncurses5-dev \
    libgdbm-dev \
    libnss3-dev \
    libssl-dev \
    libreadline-dev \
    libffi-dev \
    libsqlite3-dev \
    wget \
    libbz2-dev \
    #### 
    nlohmann-json3-dev=3.11.3-1 \
    clangd-15 \
    clang-tidy \
    # Tools
    git \
    openssh-client \
    net-tools \
    iputils-ping \
    # ROS2 packages
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-turtlesim

# Build and install Python 3.12.4
RUN cd /tmp \
    && wget https://www.python.org/ftp/python/3.12.4/Python-3.12.4.tgz \
    && tar xzf Python-3.12.4.tgz \
    && cd Python-3.12.4 \
    && ./configure --enable-shared --enable-optimizations --prefix=/usr/local \
    && make -j $(nproc) \
    && make altinstall \
    && cd .. \
    && rm -rf Python-3.12.4* \
    && ln -sf /usr/local/bin/python3.12 /usr/local/bin/python3 \
    && ln -sf /usr/local/bin/pip3.12 /usr/local/bin/pip3 \
    && ldconfig

# Install required Python packages
RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && python3 -m pip install --no-cache-dir \
    PyYAML==6.0.1 \
    empy \
    catkin_pkg \
    numpy \
    lark \
    setuptools \
    wheel

# Set up auto-source of workspace and autocomplete functionality for user 'ros'
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/${USERNAME}/.bashrc \
    && echo "source /usr/share/vcstool-completion/vcs.bash" >> /home/${USERNAME}/.bashrc \
    && echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc

# define environment variable for the used ROS2 workspace
ENV ROS_WS=/home/${USERNAME}/ros2_ws

# create package workspace directory
RUN mkdir -p ${ROS_WS}/src

WORKDIR ${ROS_WS}

# switch to interactive user mode
ENV DEBIAN_FRONTEND=dialog

# set the used DDS implementation:
# rmw_fastrtps_cpp
# rmw_cyclonedds_cpp
# rmw_connextdds
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# unique ID to separate ROS2 networks from each other
# selectable between 0 and 101 (default: 0)
ENV ROS_DOMAIN_ID=0

# Add library path for Python shared libraries
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

USER ros