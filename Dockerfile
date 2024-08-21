ARG ROS_DISTRO=humble

###############################
# Base Image for modular_slam #
###############################
FROM osrf/ros:${ROS_DISTRO}-desktop as base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

RUN mkdir -p /modular_slam_ws/src
WORKDIR /modular_slam_ws
COPY ./modules ./src/modular_slam/

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
  && apt-get update -y \
  && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
  && apt-get install software-properties-common -y \
  && add-apt-repository ppa:joseluisblancoc/mrpt-stable \
  && apt install libmrpt-dev mrpt-apps python3-pymrpt -y \
  && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]

##############################
# Image for modular_slam dev #
##############################
From base as modular_slam

ARG USERNAME=devuser
ARG UID=1000
ARG GID=${UID}

# Create new user and home directory
RUN groupadd --gid $GID $USERNAME \
  && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
  && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
  && chmod 0440 /etc/sudoers.d/${USERNAME} \
  && mkdir -p /home/${USERNAME} \
  && chown -R ${UID}:${GID} /home/${USERNAME}

# Set the ownership of the modular_slam workspace to the new user
RUN chown -R ${UID}:${GID} /modular_slam_ws/

# Set the user and source entrypoint in the user's .bashrc file
USER ${USERNAME}
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc

