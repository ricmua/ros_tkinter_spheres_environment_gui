""" Test [fixtures] for using the [pytest] framework to test the 
    `ros_tkinter_spheres_environment_gui` package.

[fixtures]: https://docs.pytest.org/en/6.2.x/fixture.html
[pytest]: https://docs.pytest.org

Examples
--------

>>> def test_object_creation(node, client, spin):
...     assert len(node.environment) == 0
...     client.initialize_object('sphere_a')
...     spin()
...     assert 'sphere_a' is in node.environment
>>> # TODO: Determine if this can be run from a doctest.

"""

# Copyright 2022 Carnegie Mellon University Neuromechatronics Lab (a.whit)
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# Contact: a.whit (nml@whit.contact)


# Import os.path.
import os.path

# Import pytest.
import pytest

# Import ROS.
import rclpy
import rclpy.node

# Import Tkinter canvas postscript utilities.
from . import tkinter_canvas_postscript

# Import tkinter_shapes.
import tkinter_shapes

# Import tkinter_spheres_environment_gui
import tkinter_spheres_environment_gui

# Import ros_tkinter_spheres_environment_gui.node.
import ros_tkinter_spheres_environment_gui.node

# Import ros_spheres_environment.
import ros_spheres_environment


# Initialize a ROS2 interface.
@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()
    
  

# Initialize a `ros_tkinter_spheres_environment_gui` node (server).
@pytest.fixture
def node(ros):
    
    # Initialize a ROS2 server node that wraps a GUI with a spheres_environment 
    # interface.
    node = ros_tkinter_spheres_environment_gui.node.Node()
    
    # Initialize a rectangular, black polygon, with the same size as the 
    # canvas. This can be necessary to capture the black background, which is 
    # important when the saved test images are used for other purposes (e.g., 
    # documentation). Otherwise, the saved images will not match the canvas as 
    # it appears to the user.
    canvas = node.environment.gui.canvas
    (w, h) = canvas.dimensions
    vertices = [(0, 0), (0, h), (w, h), (w, 0)]
    rectangle = tkinter_shapes.Polygon(canvas=canvas, vertices=vertices)
    rectangle['fill'] = rectangle['outline'] = 'black'
    
    yield node
    node.destroy_node()
    
  

# Initialize a `ros_spheres_environment` client (and associated ROS2 node).
@pytest.fixture
def client(ros):
    client = ros_spheres_environment.Client(node=rclpy.node.Node('client'))
    yield client
    client.node.destroy_node()
    del client
    
  

# Initialize a function for spinning both the client and server nodes once each.
SPIN_TIMEOUT_SECONDS = 0.005
@pytest.fixture
def spin(ros, node, client):
    def spin():
        rclpy.spin_once(client.node, timeout_sec=SPIN_TIMEOUT_SECONDS)
        rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT_SECONDS)
    yield spin
    
  

# Initialize a reference GUI
@pytest.fixture
def reference_environment_gui():
    
    # Initialize a GUI with a spheres_environment interface.
    environment_gui = tkinter_spheres_environment_gui.Environment()
    
    # Initialize a rectangular, black polygon, with the same size as the 
    # canvas. This can be necessary to capture the black background, which is 
    # important when the saved test images are used for other purposes (e.g., 
    # documentation). Otherwise, the saved images will not match the canvas as 
    # it appears to the user.
    canvas = environment_gui.gui.canvas
    (w, h) = canvas.dimensions
    vertices = [(0, 0), (0, h), (w, h), (w, 0)]
    rectangle = tkinter_shapes.Polygon(canvas=canvas, vertices=vertices)
    rectangle['fill'] = rectangle['outline'] = 'black'
    
    # Update the GUI / canvas.
    environment_gui.update()
    
    # Yield the fixture product.
    yield environment_gui
    
    # Cleanup the fixture product.
    del rectangle
    del environment_gui
    
  

# Initialize a path to an image file repository.
@pytest.fixture
def images_basepath(): return 'data/images'


# Initialize a baseline reference image.
@pytest.fixture
def reference_image_0_postscript(reference_environment_gui):
    """ Image of a freshly-initialized spheres_environment GUI canvas, without 
        any modification. """
    
    # Initialize shorthand.
    canvas = reference_environment_gui.gui.canvas
    extract = tkinter_canvas_postscript.extract
    
    # Update the canvas.
    canvas.update()
    
    # Extract the postscript image data that represents the current appearance 
    # of the environment canvas.
    ps = extract(canvas)
    
    # Return the result.
    yield ps
    
  

## Initialize a reference image with a fresh-initialized green object.
#@pytest.fixture
#def reference_image_1_postscript(reference_environment_gui, 
#                                 reference_image_0):
#    """ Image of a spheres_environment GUI canvas containing a single, 
#        freshly-initialized object, with the color set to green. """
#    
#    # Initialize parameters.
#    key = 'object_a'
#    
#    # Initialize shorthand.
#    canvas = reference_environment_gui.gui.canvas
#    extract = tkinter_canvas_postscript.extract
#    
#    # Initialize a new object in the environment.
#    obj = reference_environment_gui.initialize_object(key)
#    
#    # Update the object color property.
#    obj.color = (0.0, 1.0, 0.0, 1.0)
#    
#    # Update the canvas.
#    canvas.update()
#    
#    # Extract the postscript image data that represents the current appearance 
#    # of the environment canvas.
#    ps = extract(canvas)
#    
#    # Return the result.
#    yield ps
#    
#    # Clean up.
#    reference_environment_gui.destroy_object(key)
    
  

# Initialize a generator that yields postscript data for reference images 
# extracted from the GUI canvas following successive manipulations.
# This is NOT A PYTEST FIXTURE.
def reference_image_generator(reference_environment_gui):
    """ Generator of postscript data for reference images representing a 
        `spheres_environment` GUI canvas as it is sequentially modified. """
    
    # Initialize parameters.
    key_a = 'object_a'
    key_b = 'object_b'
    
    # Initialize shorthand.
    canvas = reference_environment_gui.gui.canvas
    extract = tkinter_canvas_postscript.extract
    
    # Image 0: baseline.
    canvas.update()
    yield extract(canvas)
    
    # Image 1: Initialize a new object and update the color property.
    obj_a = reference_environment_gui.initialize_object(key_a)
    obj_a.color = (0.0, 0.0, 1.0, 1.0)
    canvas.update()
    yield extract(canvas)
    
    # Image 2: Re-size the object.
    #obj_a.width = 0.10
    obj_a.radius = 0.20
    canvas.update()
    yield extract(canvas)
    
    # Image 3: Re-position the object.
    obj_a.position = (0.50, -0.55, 1.00)
    canvas.update()
    yield extract(canvas)
    
    # Image 4: Initialize a second object and update the color, position,  
    #          and size properties.
    obj_b = reference_environment_gui.initialize_object(key_b)
    obj_b.color = (0.0, 1.0, 0.0, 1.0)
    obj_b.position = (-0.25, 0.25, 0.00)
    #obj_b.width = 0.10
    obj_b.radius = 0.10
    canvas.update()
    yield extract(canvas)
    
    # Image 5: Move the second object to overlap with the first.
    obj_b.position = (0.35, -0.35, 1.0)
    canvas.update()
    yield extract(canvas)
    
  

# Initialize an array of postscript data for reference images extracted from 
# the GUI canvas.
@pytest.fixture
def reference_image_sequence(reference_environment_gui):
    """ Images -- represented as postscript data -- of a spheres_environment 
        GUI canvas, as it is sequentially modified. """
    
    # Initialize parameters.
    generator = reference_image_generator(reference_environment_gui)
    yield tuple(ps for ps in generator)
    
    # Clean up.
    pass
    
  

# __main__
if __name__ == '__main__':
    import doctest
    doctest.testmod()
    
  


