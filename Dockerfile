ARG ROS_DISTRO=humble

#######################
# Base Image for Mola #
#######################
FROM osrf/ros:${ROS_DISTRO}-desktop as base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

RUN mkdir -p /mola_ws/src
WORKDIR /mola_ws
COPY ./mola/ ./src/mola/
COPY ./mola_common/ ./src/mola_common/
COPY ./mola_lidar_odometry/ ./src/mola_lidar_odometry/
COPY ./mola_test_datasets/ ./src/mola_test_datasets/
COPY ./mp2p_icp/ ./src/mp2p_icp/

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
  && apt-get update -y \
  && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
  # && colcon  build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]

##########################
# Image for mola example #
##########################
From base as mola

ARG USERNAME=devuser
ARG UID=1000
ARG GID=${UID}

# RUN apt-get update && apt-get install -y --no-install-recommends \
#   ros-${ROS_DISTRO}-mola ros-${ROS_DISTRO}-mola-test-datasets

# Create new user and home directory
RUN groupadd --gid $GID $USERNAME \
  && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
  && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
  && chmod 0440 /etc/sudoers.d/${USERNAME} \
  && mkdir -p /home/${USERNAME} \
  && chown -R ${UID}:${GID} /home/${USERNAME}

# Set the ownership of the mola workspace to the new user
RUN sudo chown -R ${UID}:${GID} /mola_ws/

# Set the user and source entrypoint in the user's .bashrc file
USER ${USERNAME}
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc

