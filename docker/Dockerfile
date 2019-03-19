FROM ros:kinetic

# Install rvizweb
WORKDIR /rvizweb_ws
ARG rvizweb_branch=master
RUN git clone https://github.com/osrf/rvizweb/ -b ${rvizweb_branch} src/rvizweb
RUN apt update && rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/kinetic/setup.sh && . ~/.bashrc && catkin_make install

# Clear apt cache.
RUN apt clean

ENTRYPOINT ["/bin/bash", "-c", "source /rvizweb_ws/install/setup.bash && /bin/bash"]
