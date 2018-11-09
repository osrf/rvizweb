# RVizWeb - RViz, but on your browser

RVizWeb provides a convenient way of building and launching a web application
with features similar to [RViz](https://github.com/ros-visualization/rviz).

This project makes use of the following:

* @jstnhuang's [ros-rviz](https://github.com/jstnhuang/ros-rviz) web component
* @tork-a's [roswww](https://github.com/tork-a/roswww) web server
* @RobotWebTools's [rosbridge_server](https://github.com/RobotWebTools/rosbridge_suite)
  and [tf2_web_republisher](https://github.com/RobotWebTools/tf2_web_republisher)

## Quickstart

1. Create a directory for your catkin workspace:

        mkdir -p ~/ws/src
        cd ~/ws/src
        git clone https://github.com/osrf/rvizweb/

1. Install non-ROS dependencies:

        curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.33.4/install.sh | bash
        . ~/.bashrc
        nvm install node
        npm install -g bower polymer-cli

1. Install ROS dependencies (assumes you already have ROS core):

        cd ~/ws
        rosdep install --from-paths src --ignore-src -r -y

1. Build and install your workspace; this will run `bower` and `polymer-cli`
   to generate and install the site:

        cd ~/ws
        catkin_make install

1. Launch RVizWeb:

        . ~/ws/install/setup.bash
        roslaunch rvizweb rvizweb.launch

1. Open the site on the browser

    http://localhost:8001/rvizweb/www/index.html

1. Let's try an example display to check everything is working. On the UI, click on the `+` and choose "Markers".

1. Now open a new terminal and publish the following marker:

        rostopic pub /visualization_marker visualization_msgs/Marker '{header: {frame_id: "base_link"}, id: 1, type: 1, action: 0, pose: {position: {x: 0., y: 0.2, z: 0.}, orientation: {x: 0.3, y: 0.2, z: 0.52, w: 0.85}}, scale: {x: 0.2, y: 0.3, z: 0.1}, color: {r: 1., g: 0., b: 1., a: 0.3}, lifetime: 50000000000}'

1. You should see a pink box show up on the site.

## Viewing URDF

By default, RVizWeb will serve the `share` folder of all the ROS packages that
are currently installed in the system to the following address by default:

    http://localhost:8001/<package_name/

This means that if you have robot description files installed, all these resources
will be automatically served and are ready to be used by RVizWeb.

Let's try an example using a PR2 simulated on Gazebo.

1. Install the PR2 Gazebo package:

        sudo apt install -y ros-kinetic-pr2-gazebo

1. Launch the simulation:

        roslaunch pr2_gazebo pr2_empty_world.launch

1. Launch RVizWeb:

        roslaunch rvizweb rvizweb.launch

1. Open the site on the browser

    http://localhost:8001/rvizweb/www/index.html

1. On the UI, click on the `+` and choose "Robot model".

1. You should see the PR2 on the browser (it will be dark due to a texture issue).

