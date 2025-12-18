# Base ROS2 Humble image
FROM ros:humble-ros-core

# Install OS dependencies
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-pip python3-venv build-essential git curl wget vim ca-certificates \
    x11-apps libgl1 libglu1-mesa libsm6 libxrender1 libfontconfig1 libx11-6 \
    libxrandr2 libxinerama1 libxcursor1 libxcomposite1 libasound2 mesa-utils \
    libxkbcommon-x11-0 libxcb-xinerama0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 \
    libxcb-randr0 libxcb-render-util0 libxcb-shape0 libxcb-shm0 libxcb-sync1 \
    libxcb-xfixes0 libxcb-xinput0 libxcb-xkb1 libxcb-cursor0 libxcb-glx0 libxcb-present0 \
    libxkbcommon0 libx11-xcb1 libqt5gui5 libqt5widgets5 libqt5core5a libqt5x11extras5 qtbase5-dev \
  && rm -rf /var/lib/apt/lists/*

# Python tooling
RUN pip3 install -U colcon-common-extensions setuptools

# Create non-root user
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
 && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
 && apt-get update && apt-get install -y sudo \
 && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
 && chmod 0440 /etc/sudoers.d/$USERNAME \
 && rm -rf /var/lib/apt/lists/*

USER $USERNAME
WORKDIR /home/$USERNAME

# Create container-internal workspace
RUN mkdir -p /home/$USERNAME/ros2_ws/src

# The folder where your host project will be mounted
RUN mkdir -p /home/$USERNAME/project

# Add entrypoint
COPY entrypoint.sh /home/$USERNAME/entrypoint.sh
RUN sudo chmod +x /home/$USERNAME/entrypoint.sh

# Environment variables for Qt/X11
ENV QT_QPA_PLATFORM=xcb
ENV QT_XCB_FORCE_SOFTWARE_OPENGL=1
ENV DISPLAY=:0

ENTRYPOINT ["/home/rosuser/entrypoint.sh"]
CMD ["bash"]
