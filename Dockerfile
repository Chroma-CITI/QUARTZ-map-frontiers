# Start from the ROS Noetic base image
FROM osrf/ros:noetic-desktop-full

# To avoid deprecation warnings in rviz
ENV DISABLE_ROS1_EOL_WARNINGS=true

# Set the working directory
WORKDIR /opt/catkin_ws/src

# install ssh client and git
RUN apt-get update && apt-get install -y git openssh-client
# download public key for github.com
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

# Install necessary packages
COPY requirements.txt /tmp/requirements.txt
RUN apt-get update && \
    apt-get install -y $(cat /tmp/requirements.txt | cut -d'=' -f1) && \
    rm -rf /var/lib/apt/lists/*

# Clone the repository and get the commit to be checkout at from the docker-compose.yaml file
#ARG BRANCH='main'
ARG BRANCH='main'
ARG BRANCH_COMMIT=$BRANCH # Checkout the last commit per default

RUN --mount=type=ssh git clone --single-branch --branch $BRANCH git@github.com:Chroma-CITI/QUARTZ-map-frontiers.git map-frontiers \
    && cd /opt/catkin_ws/src/map-frontiers/ \
    && echo "The commit to be check out is: $BRANCH_COMMIT" \
    && git checkout $BRANCH_COMMIT \
    && git submodule update --init --recursive

# Install the package's dependencies
RUN --mount=type=ssh wstool init . /opt/catkin_ws/src/map-frontiers/voxfield_https.rosinstall \
&& wstool update

# Go back to the workspace root
WORKDIR /opt/catkin_ws/

# Initialize and build the Catkin workspace
RUN catkin config  --extend /opt/ros/noetic \
    && catkin build voxblox_ros 

# # Hacky hack
RUN mkdir -p /opt/catkin_ws/devel/.private/voxblox_map/include/ \
    && mkdir -p /opt/catkin_ws/devel/.private/map_frontiers/include/ \
    && catkin build map_frontiers test_scripts

# # Source the setup.bash so that the package is available in the environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /opt/catkin_ws/devel/setup.bash" >> ~/.bashrc

# # Set the entrypoint
CMD ["bash", "-c"]
