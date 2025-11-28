FROM ros:humble-ros-base AS base

ARG GID
ARG UID
ARG USERNAME
ARG DOCKER_HOME_DIR
ARG DOCKER_WORKDIR
ARG ROS_DISTRO=humble
# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Install Python and pip
RUN apt-get update && apt-get install -y \
    python-is-python3 \
    python3-pip \
    vim \
    git \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
####################
# Deployment image #
####################
FROM base as deploy

RUN addgroup --gid $GID $USERNAME
RUN adduser --disabled-password --gecos '' --uid $UID --gid $GID $USERNAME

WORKDIR ${DOCKER_WORKDIR}
RUN chown -R ${UID}:${GID} ${DOCKER_HOME_DIR}
USER ${USERNAME}

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
 && echo "PS1='${debian_chroot:+($debian_chroot)}\[\033[01;33m\]\u@docker-\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> ~/.bashrc