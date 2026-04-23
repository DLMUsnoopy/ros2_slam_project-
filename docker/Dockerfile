FROM osrf/ros:jazzy-desktop-full
# do i get env variables from the shell in the build? i dont think so
ENV ROS_DISTRO=jazzy
# ubuntu 24.04 images include an ubuntu user by default which has GID 1000
RUN userdel -r ubuntu

RUN apt-get update && apt-get upgrade -y \
  && apt-get install --no-install-recommends -y \
  zsh \
  git \
  python3-pip \
  python3-dev \
  python3-setuptools \
  python-is-python3 \
  tmux \
  sudo \
  curl \
  wget \
  build-essential \
  tree \
  openssh-client \
  software-properties-common \
  ninja-build \
  gettext \
  unzip \
  libgmp-dev \
  libmpfr-dev \
  libmpc-dev \
  libisl-dev \
  zlib1g-dev \
  file \
  cmake \
  libssl-dev \
  ros-${ROS_DISTRO}-rosbag2-storage-mcap \
  ros-${ROS_DISTRO}-rosbag2 \
  ros-${ROS_DISTRO}-rmw-zenoh-cpp \
  ros-${ROS_DISTRO}-rviz2 \
  ros-${ROS_DISTRO}-plotjuggler-ros \
  libeigen3-dev \
  libtbb-dev \
  clang \
  libclang-dev \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp

ARG UID=1000
ARG GID=1000
ENV UNAME=dev

# Add normal sudo-user to container, passwordless, taken from nacho's ros in docker
RUN addgroup --gid $GID $UNAME \
  && adduser --disabled-password --gecos '' --uid $UID --gid $GID $UNAME \
  && adduser $UNAME sudo \
  && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
  && sed -i 's/required/sufficient/' /etc/pam.d/chsh \
  && touch /home/$UNAME/.sudo_as_admin_successful

WORKDIR /home/${UNAME}/
ENV HOME=/home/${UNAME}
USER ${UNAME}
ENV PATH="${PATH}:${HOME}/.local/bin"

RUN sudo sh -c "$(curl -L https://raw.githubusercontent.com/balancedscorpion/zsh-in-docker/new_user/zsh-in-docker.sh)" -- \
  -u ${UNAME} \
  -x \
  -p git -p z -p sudo -p history -p copybuffer -p copypath -p copyfile \
  -p https://github.com/zsh-users/zsh-autosuggestions \
  -p https://github.com/zsh-users/zsh-completions

RUN sudo chsh -s $(which zsh) ${UNAME}
ENV SHELL=/usr/bin/zsh
SHELL ["/bin/zsh", "-lc"]

WORKDIR /home/${UNAME}/p03_ws
CMD ["/bin/zsh", "-i"]
