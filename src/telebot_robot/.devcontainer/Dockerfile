# FROM ros:humble
# ARG USERNAME=USERNAME
# ARG USER_UID=1000
# ARG USER_GID=$USER_UID

# # Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
# RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# # Create the user
# RUN groupadd --gid $USER_GID $USERNAME \
#     && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
#     #
#     # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
#     && apt-get update \
#     && apt-get install -y sudo \
#     && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
#     && chmod 0440 /etc/sudoers.d/$USERNAME
# RUN apt-get update && apt-get upgrade -y
# RUN apt-get install -y python3-pip
# ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
# USER $USERNAME
# CMD ["/bin/bash"]


FROM ros:humble-ros-base

WORKDIR /telebot_ws

COPY src ./src
# Enable universe repo
RUN apt-get update && apt-get install -y \
    software-properties-common \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

# Ensure ROS apt repo is available


RUN apt-get update


RUN rosdep init || true && rosdep update

RUN . /opt/ros/humble/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/humble/setup.sh && \
    colcon build

CMD ["bash"]

