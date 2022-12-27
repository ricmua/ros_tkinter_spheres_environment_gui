---
title: README
author: a.whit ([email](mailto:nml@whit.contact))
date: December 2022
---

<!-- License

Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

# ros_tkinter_spheres_environment_gui

A ROS2 package that provides a Tkinter-based 2D graphical interface for a 
virtual environment, in which spherical objects interact in a 3D space.

This package uses the [ros_spheres_environment] to add ROS2 capabilities to the 
[tkinter_spheres_environment_gui] Python package. 

## Installation

This package can be added to any [ROS2 workspace] (also see the explanation of 
[colcon workspaces] for additional information). See the [installation documentation](doc/markdown/installation.md) for further details.

### Testing

See the [testing documentation](doc/markdown/testing.md) for further 
information.

## Getting started

Perhaps the best way to get started is via a simple example. The code that 
follows must be run from within a [configured ROS2 environment]. Be sure to 
[source the workspace overlay], in addition to the 
[ROS2 environment][source the ROS2 environment].

### Example 1: Command line

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

### Example 2: Python

For the sake of this example, it will be assumed that the objective is to 
create an environment where a spherical cursor can be moved around a 
two-dimensional plane to interact with a set of spherical targets. As noted 
in the [tkinter_spheres_environment] documentation, however, the third (`z`) 
dimension of the spheres is ignored for the sake of visualization; that is, the 
spheres are represented as circles on the 2D canvas plane (see images below).

The first step is to initialize a ROS2 interface.

```python
>>> import rclpy
>>> rclpy.init()

```

Next, create a GUI environment node. This initializes a top-level GUI window, 
with a blank canvas, and connects the canvas to the [ROS2 graph].

```python
>>> from ros_tkinter_spheres_environment_gui import Node
>>> node = Node()

```

The ROS2 node wraps a GUI that implements the interface to a virtual 
environment defined in the [spheres_environment] package. The methods of this 
interface are mapped to GUI elements. Establish some shorthand to clarify the 
relevant objects and interfaces.

```python
>>> environment = node.environment
>>> canvas = environment.gui.canvas

```

Add a spherical target to the environment. Although the GUI will not change 
(without further action), 
it can be verified that the target object has been 
added to the environmental state. The parameters of the target are set to some 
default values. 

```python
>>> target = environment.initialize_object('target')
>>> environment['target']
{'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'radius': 1.0}

```

To make the sphere visible, change the target color to blue and update the 
canvas. The default color of both the sphere and the background is black. 
Sphere colors are specified via an [RGBA] color model, where values can 
range from `0.0` to `1.0`. Due to limitations of tkinter, the alpha parameter 
of the color model is thresholded: the object is completely transparent for any 
values less than `1.0`.

```python
>>> target.color = (0.0, 0.0, 1.0, 1.0)
>>> environment.update()

```

The canvas should now appear as shown in Figure 1. The target fills the center 
of the canvas, because the radius is set to `1.0` and it is positioned at the 
origin. The canvas GUI uses normalized coordinates that range from `-1` 
to `+1` in each dimension. The target radius is expressed in terms of these 
coordinates. Therefore, a unit radius will result in a diameter two circle that 
intersects with the edges of the workspace.

![Figure 1](data/images/reference_image_1.svg "Figure 1")

Set new parameters to re-size and re-position the target.

```python
>>> target.radius = 0.20
>>> target.position = (0.50, -0.55, 1.00)
>>> environment.update()

```

The canvas should now appear as in Figure 2. The target is smaller and is away 
from the origin. Note that the 3rd (`z`) coordinate of the target position is 
set arbitarily, and that this has no visible effect.

![Figure 2](data/images/reference_image_3.svg "Figure 2")

Add a cursor to the workspace, and similarly set parameters.

```python
>>> cursor = environment.initialize_object('cursor')
>>> cursor.color = (0.0, 1.0, 0.0, 1.0)
>>> cursor.radius = 0.10
>>> cursor.position = (-0.25, 0.25, 0.00)
>>> environment.update()
>>> list(environment)
['target', 'cursor']

```

The canvas should now appear as in Figure 3. Both a cursor and a target are 
visible on the canvas.

![Figure 3](data/images/reference_image_4.svg "Figure 3")

Move the cursor to intersect with the target. The canvas should now appear as 
in Figure 4.

```python
>>> cursor.position = (0.35, -0.35, 1.0)
>>> environment.update()

```

![Figure 4](data/images/reference_image_5.svg "Figure 4")

Note that the cursor appears in the foreground of the canvas, even thought the 
3rd (`z`) coordinate of the target position exceeds that of the cursor 
position. Note further that the `z` coordinates does not affect the appearance 
of the spheres in any way (e.g., size). This is because the canvas is 2D, and 
the third coordinate does not affect visualization. It is possible to change 
the [z-order] of the spheres by bringing the target to the foreground.

```python
>>> target.to_foreground()
>>> environment.update()

```

Finally, clean up by deleting the environment, destroying the node, and 
shutting down the ROS2 interface. This closes the GUI window. This step is not 
strictly necessary, as the GUI will otherwise be destroyed when the environment 
is implicitly deleted by the Python environment. However, it is good practice 
to shut down the ROS2 interface.

```python
>>> #environment.gui.destroy()
>>> del environment
>>> node.destroy_node()
>>> rclpy.shutdown()

```

### Example doctests

The examples in this README are rendered in [doctest] format, and can be run 
via the following code:[^python_paths]

[^python_paths]: Provided that the package is installed, or the [Python path] 
                 is otherwise set appropriately.

```
import doctest
doctest.testfile('README.md', module_relative=False)

```

## License

Copyright 2022 [Neuromechatronics Lab][neuromechatronics], 
Carnegie Mellon University

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

<!---------------------------------------------------------------------
   References
---------------------------------------------------------------------->

[Python path]: https://docs.python.org/3/tutorial/modules.html#the-module-search-path

[doctest]: https://docs.python.org/3/library/doctest.html

[setuptools]: https://setuptools.pypa.io/en/latest/userguide/quickstart.html#basic-use

[neuromechatronics]: https://www.meche.engineering.cmu.edu/faculty/neuromechatronics-lab.html

[pip install]: https://pip.pypa.io/en/stable/cli/pip_install/

[spheres_environment]: https://github.com/ricmua/spheres_environment

[ROS2 graph]: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html#background

[ROS2 executor]: https://docs.ros.org/en/humble/Concepts/About-Executors.html

[ros_spheres_environment]: https://github.com/ricmua/ros_spheres_environment.git

[tkinter_spheres_environment_gui]: https://github.com/ricmua/tkinter_spheres_environment_gui.git

[tkinter_shapes]: https://github.com/ricmua/tkinter_shapes.git

[z-order]: https://en.wikipedia.org/wiki/Z-order

[RGBA]: https://en.wikipedia.org/wiki/RGBA_color_model

[source the workspace overlay]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#source-the-overlay

[source the ROS2 environment]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#source-ros-2-environment

[ROS2 command line tools]: https://docs.ros.org/en/humble/Concepts/About-Command-Line-Tools.html

