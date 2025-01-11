FROM osrf/ros:jazzy-desktop

ARG DEBIAN_FRONTEND=noninteractive

ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user

WORKDIR /workspaces

RUN (getent group $USER_GID || groupadd --gid $USER_GID $USERNAME) \
    && (getent passwd $USER_UID || useradd --uid $USER_UID --gid $USER_GID -m $USERNAME) \
    && mkdir -p -m 0700 /run/user/"${USER_UID}" \
    && mkdir -p -m 0700 /run/user/"${USER_UID}"/gdm \
    && chown $USER_UID:$USER_GID /run/user/"${USER_UID}" \
    && chown $USER_UID:$USER_GID /workspaces \
    && chown $USER_UID:$USER_GID /run/user/"${USER_UID}"/gdm 

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*

ENV XDG_RUNTIME_DIR=/run/user/"${USER_UID}"

# Specify user by UID & GID so this script doesn't
# depend on the exact username
USER $USER_UID:$USER_GID

RUN echo "source /ros_entrypoint.sh" >>~/.bashrc
ARG MAX_ROS_DOMAIN_ID=232
RUN echo "export ROS_DOMAIN_ID=100" >>~/.bashrc
RUN echo "set +e" >>~/.bashrc
