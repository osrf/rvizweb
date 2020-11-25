FROM ros:melodic

# Install rvizweb.
RUN apt update && apt install -y curl && curl -sL https://deb.nodesource.com/setup_8.x | bash - && \
    mkdir /rvizweb_ws && \
    cd /rvizweb_ws && \
    git clone https://github.com/Sausy/rvizweb.git src/rvizweb && \
    rosdep install --from-paths src --ignore-src -r -y && \
    . /opt/ros/melodic/setup.sh && catkin_make install && \
    apt-get clean

ENTRYPOINT ["/bin/bash", "-c", "source /rvizweb_ws/install/setup.bash && roslaunch rvizweb rvizweb.launch"]
