FROM ros:kinetic

# Basic tools and dependencies
RUN apt update && apt install -y curl && curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.33.4/install.sh | bash
RUN . ~/.bashrc && nvm install node

# Install rvizweb
WORKDIR /rvizweb_ws
RUN git clone https://github.com/osrf/rvizweb/ src/rvizweb
RUN rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/kinetic/setup.sh && . ~/.bashrc && catkin_make install

# Clear apt cache.
RUN apt clean

ENTRYPOINT ["/bin/bash", "-c", "source /rvizweb_ws/install/setup.bash && /bin/bash"]
