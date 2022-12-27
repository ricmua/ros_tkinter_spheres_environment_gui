<!-- License

Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)

Contributors:
  a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

## Installation

This package can be added to any [ROS2 workspace] (also see the explanation of 
[colcon workspaces] for additional information) via the `git clone` 
command:[^dependencies]

[^dependencies]: The `--recurse-submodules` git command line flag is optional. 
                 See the information about [dependencies](#dependencies).

```bash
git clone --recurse-submodules https://github.com/ricmua/ros_tkinter_spheres_environment_gui.git path/to/workspace/src/
```

### Dependencies

This package depends directly on the [ros_spheres_environment] ROS2 package and 
the [tkinter_spheres_environment_gui] Python package. It depends indirectly on 
the [spheres_environment] and [tkinter_shapes] Python packages.

Both the [tkinter_spheres_environment_gui] and [tkinter_shapes] Python packages 
are included as [git submodules] in this repository. These packages will be 
installed with this package during the build process. If these two packages 
have already been installed on the target machine, then they can be excluded 
from the build by omitting the `--recurse submodules` flag of the clone 
command.

Unlike the [tkinter_spheres_environment_gui] and [tkinter_shapes] packages, it 
is expected that the [spheres_environment] Python package will be used by 
multiple packages in the ROS2 workspace. For this reason, it is not included as 
a submodule in this repository. It is recommended that this package should be 
installed in the normal manner of Python packages (e.g., via `pip`).

The [ros_spheres_environment] package must be present in the same workspace as 
this package. Otherwise, the build will fail. Clone the package as one 
normal, prior to build:

```bash
git clone https://github.com/ricmua/ros_spheres_environment.git path/to/workspace/src/
```

### Build

ROS2 workspaces are built using [colcon], from within a 
[configured ROS2 environment].

```bash
cd path/to/workspace
source path/to/ros/setup.bash
colcon build
```

### Microsoft Windows

The above examples are tailored to Linux installations. The installation 
commands will differ slightly for the Windows Operating system.

The command to initialize a configured ROS2 environment uses the `call` 
function on `setup.bat`, instead of `source` on `setup.bash`. The path to the 
ROS2 installation will likely also differ.

```
call path\to\ros\setup.bat
```

For Windows, the `--merge-install` [colcon build flag] is recommended:

> for workspaces with many packages otherwise the environment variables might 
  exceed the supported maximum length.

```
colcon build --merge-install
```

<!---------------------------------------------------------------------
   References
---------------------------------------------------------------------->

[ROS2]: https://docs.ros.org/en/humble/index.html

[ROS2 workspace]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html

[colcon]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html

[colcon build flag]: https://colcon.readthedocs.io/en/released/reference/verb/build.html

[configured ROS2 environment]: https://docs.ros.org/en/humble/Tutorials/Configuring-ROS2-Environment.html

[colcon workspaces]: https://colcon.readthedocs.io/en/released/user/what-is-a-workspace.html

[git submodules]: https://git-scm.com/book/en/v2/Git-Tools-Submodules

[spheres_environment]: https://github.com/ricmua/spheres_environment

[ros_spheres_environment]: https://github.com/ricmua/ros_spheres_environment.git

[tkinter_spheres_environment_gui]: https://github.com/ricmua/tkinter_spheres_environment_gui.git

[tkinter_shapes]: https://github.com/ricmua/tkinter_shapes.git
