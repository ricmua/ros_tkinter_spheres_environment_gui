<!-- License

Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)

Contributors:
  a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->


## Command line example

This example is intended to illustrate basic usage of the GUI via the 
[ROS2 command line tools]. This is most useful for quickly becoming acquainted 
with this package.

Run the GUI node using the `run` sub-command / verb. For the sake of this 
example, set the canvas background color to white via the ROS2 `canvas.color` 
parameter.[^canvas_color] A GUI canvas should appear immediately.

[^canvas_color]: A white canvas color is required for this example because the 
                 `color` property of sphere objects has not yet been fully 
                 implemented.

```bash
ros2 run ros_tkinter_spheres_environment_gui main --ros-args -p canvas.color:=white
```

Open a second command line for a [configured ROS2 environment]. Unless 
otherwise stated, the instructed commands in the remainder of this example are 
to be entered at the second command line.

A number of [ROS2 topics] have been initialized. Running `ros2 topic list` 
should show the availability of the `initialize` topic. Create a circle on the 
canvas by publishing to this topic with a key (i.e., name or label) for the 
new object. Here, the circle will be referred to using the key "target".

```bash
ros2 topic pub --once /initialize example_interfaces/msg/String "{data: target}"
```

Listing topics should now show the addition of the `target/position` and 
`target/radius` topics, and a black circle should appear in the center of the 
canvas.[^sphere_color] Re-size and re-position the target sphere by publishing 
messages to these topics.

[^sphere_color]: All circles initialized on the canvas will be colored black 
                 when initialized via command line utilities, at present. This 
                 is because the `color` property of sphere objects has not yet 
                 been fully implemented.

```bash
ros2 topic pub --once /target/radius example_interfaces/msg/Float64 "{data: 0.20}"
ros2 topic pub --once /target/position geometry_msgs/msg/Point "{x: 0.50, y: -0.55, z: 1.00}"
```

Add a second sphere, and similarly re-size and re-position it.

```bash
ros2 topic pub --once /initialize example_interfaces/msg/String "{data: cursor}"
ros2 topic pub --once /cursor/radius example_interfaces/msg/Float64 "{data: 0.10}"
ros2 topic pub --once /cursor/position geometry_msgs/msg/Point "{x: -0.25, y: 0.25, z: 0.00}"
```

Move the cursor sphere to overlap with the target sphere.

```bash
ros2 topic pub --once /cursor/position geometry_msgs/msg/Point "{x: 0.35, y: -0.35, z: 0.00}"
```

Destroy the target sphere and re-size the cursor sphere.[^destroy_bug]

[^destroy_bug]: At present, the `destroy` call does not immediately update the 
                visual appearance of the canvas. This is a bug.

```bash
ros2 topic pub --once /destroy example_interfaces/msg/String "{data: target}"
ros2 topic pub --once /cursor/radius example_interfaces/msg/Float64 "{data: 0.05}"
```

Clean up by closing the second command line environment. Shut-down the ROS2 
node and destroy the GUI by pressing `Ctrl-C` at the command prompt for the 
first command line environment.

<!---------------------------------------------------------------------
   References
---------------------------------------------------------------------->

[neuromechatronics]: https://www.meche.engineering.cmu.edu/faculty/neuromechatronics-lab.html

[ROS2 graph]: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html#background

[ros_spheres_environment]: https://github.com/ricmua/ros_spheres_environment.git

[tkinter_spheres_environment_gui]: https://github.com/ricmua/tkinter_spheres_environment_gui

[z-order]: https://en.wikipedia.org/wiki/Z-order

[RGBA]: https://en.wikipedia.org/wiki/RGBA_color_model

[source the workspace overlay]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#source-the-overlay

[source the ROS2 environment]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#source-ros-2-environment

[ROS2 command line tools]: https://docs.ros.org/en/humble/Concepts/About-Command-Line-Tools.html

[configured ROS2 environment]: https://docs.ros.org/en/humble/Tutorials/Configuring-ROS2-Environment.html

[ROS2 topics]: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#background


