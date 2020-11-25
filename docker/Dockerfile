FROM ros:melodic

# Basic tools and dependencies and nodejs LTS PPA.
RUN apt update && apt install -y curl && curl -sL https://deb.nodesource.com/setup_8.x | bash -

# Install rvizweb.
WORKDIR /rvizweb_ws
ARG rvizweb_branch=master
RUN git clone https://github.com/Sausy/rvizweb.git src/rvizweb -b ${rvizweb_branch} && \
    rosdep install --from-paths src --ignore-src -r -y && \
    . /opt/ros/melodic/setup.sh && catkin_make install && \
    apt-get clean

ENTRYPOINT ["/bin/bash", "-c", "source /rvizweb_ws/install/setup.bash && roslaunch rvizweb rvizweb.launch"]
