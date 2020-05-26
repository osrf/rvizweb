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

1. You will need the LTS version of Node.js. Add the PPA so that `rosdep` can fetch it:

        curl -sL https://deb.nodesource.com/setup_8.x | sudo bash -

1. Install ROS and system dependencies (assumes you already have ROS core):

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

## Viewing interactive markers

To view interactive markers, you will need an [interactive_marker_proxy](https://wiki.ros.org/interactive_marker_proxy) for
each interactive marker server you want to run, specifying a target frame and topic.

By default, RVizWeb will run one of these proxies using `/base_link` as target frame and `/basic_controls` as the topic.

Let's see an example:

1. Install `interactive_marker_tutorials`:

        sudo apt install -y ros-kinetic-interactive-marker-tutorials

1. Run the sample interactive marker server:

        rosrun interactive_marker_tutorials basic_controls

Launch RVizWeb as described in [Viewing URDF section](#viewing-urdf); this time click `+` and choose `Interactive markers`.
You should see markers all around the viewer; you can modify their poses with the controls around them!

The target frame and topic are configurable when launching the application, e.g.:

    roslaunch rvizweb rvizweb.launch interactive_markers_target_frame:=/base_footprint interactive_markers_topic:=/advanced_controls

If you need additional proxies, you can run them on separate consoles:

    rosrun interactive_marker_proxy proxy topic_ns:=/your_topic target_frame:=/your_frame

Finally, you can disable the proxy if you don't need it at all:

    roslaunch rvizweb rvizweb.launch interactive_markers:=false

## Viewing depth clouds

To view depth clouds, you will need [web_video_server](https://wiki.ros.org/web_video_server) and [depthcloud_encoder](https://wiki.ros.org/depthcloud_encoder) running.

Here's a basic example using Turtlebot:

1. Install `turtlebot_gazebo`:

        sudo apt install -y ros-kinetic-turtlebot-gazebo

1. Run `turtlebot_world` in Gazebo:

        roslaunch turtlebot_gazebo turtlebot_world.launch

1. Launch RVizWeb enabling depth clouds:

        roslaunch rvizweb rvizweb.launch depth_cloud:=true

Open RVizWeb as described in [Viewing URDF section](#viewing-urdf); click `+` and choose `Depth cloud`.
You should see Turtlebot's depth cloud stream in the viewer.

Under the hood, `depthcloud_encoder` is subscribing to depth and RGB image streams and combining them into a single topic (`/depthcloud_encoded`).
You can change the default image stream sources like this:

        roslaunch rvizweb rvizweb.launch depth_cloud:=true depth_topic:=/your_depth_topic rgb_topic:=/your_rgb_topic

## Use custom configuration on startup

To launch `rvizweb` with a custom configuration you will need to provide a JSON file as argument to the launchfile:

```
roslaunch rvizweb rvizweb.launch config_file:=/path/to/config_file.json
```

The easiest way of generating a configuration file is the following:
- Launch `rvizweb` without any particular arguments, and open it in the browser.
- Open the components you want to be opened on start, and edit `Global options` at will.
- Click on `LOAD CONFIG` at the left panel to open the popup with the complete configuration file, and copy its contents to a local file.
- Send that file as an argument when launching the application as shown above.

**Notes:**
- Editing `config/configuration.json` won't work if the application is not reinstalled; providing a separate custom file is recommended.
- Empty or undefined fields for `globalOptions` will be set to default.

## RVizWeb in a Docker container

To run `RVizWeb` inside a container use the scripts to build and run the application:

1. Clone the repository:

        git clone https://github.com/osrf/rvizweb ~/rvizweb

1. Build the docker image:

        ~/rvizweb/docker/build.sh

1. Run the container:

        ~/rvizweb/docker/run.sh

1. Once inside the container, launch `RVizWeb`:

        roslaunch rvizweb rvizweb.launch

The network will be shared with the host by default.
