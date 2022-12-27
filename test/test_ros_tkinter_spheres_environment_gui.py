""" Tests conforming with [pytest] framework requirements, for testing the 
    `ros_tkinter_spheres_environment_gui` package.

[pytest]: https://docs.pytest.org

Usage examples: 

`pytest test_ros_tkinter_spheres_environment_gui`

`pytest test_ros_tkinter_spheres_environment_gui::test_object_initialization`

`pytest -k test_object_initialization test_ros_tkinter_spheres_environment_gui`

"""

# Copyright 2022 Carnegie Mellon University Neuromechatronics Lab (a.whit)
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# Contact: a.whit (nml@whit.contact)


# Import pytest.
import pytest

# Import Tkinter canvas postscript utilities.
from . import tkinter_canvas_postscript

# Import fixtures.
from .fixtures import ros
from .fixtures import node
from .fixtures import client
from .fixtures import spin
from .fixtures import reference_environment_gui
from .fixtures import images_basepath
#from .fixtures import reference_image_1_postscript
from .fixtures import reference_image_sequence



# Test initialization of an object in the environment.
def test_object_initialization(node, client, spin):
    
    # Initialize an object key.
    key = 'object_a'
    
    # Request object initialization via the client.
    client.initialize_object(key)
    
    # Confirm that the object has been created locally.
    assert key in client
    
    # Send / receive ROS2 messages.
    spin()
    
    # Confirm that the object has been created remotely.
    assert key in node.environment
    
  

# Test destruction of an object in the environment.
def test_object_destruction(node, client, spin):
    
    # Initialize an object key.
    key = 'object_a'
    
    # Request object initialization via the client.
    client.initialize_object(key)
    
    # Transmit the request to the remote server via ROS2 messages.
    spin()
    
    # Confirm that the object has been created remotely.
    assert key in node.environment
    
    # Request object destruction via the client.
    client.destroy_object(key)
    
    # Confirm that the object has been destroyed locally.
    assert key not in client
    
    # Transmit the request to the remote server via ROS2 messages.
    spin()
    
    # Confirm that the object has been destroyed remotely.
    assert key not in node.environment
    
  

# Test the equivalence of the canvas with that of a reference environment.
def test_reference_environment(node, reference_environment_gui):
    
    # Initialize shorthand.
    extract = tkinter_canvas_postscript.extract
    equals = tkinter_canvas_postscript.equals
    
    # Extract postscript image data to represent the canvas.
    ps_o = extract(node.environment.gui.canvas)
    
    # Create a reference image to compare against.
    # Extract postscript image data to represent the reference canvas.
    ps_e = extract(reference_environment_gui.gui.canvas)
    
    # Verify that the environment canvas visually matches the reference.
    # Verify that the observed canvas matches the expected.
    assert equals(ps_o, ps_e)
    
  

## Test a generated reference image against a copy stored on disk.
## Saves the generated image to disk, if the file does not exist.
## This is NOT A PYTEST. It is to be invoked by pytests.
#def reference_image_disk_test(image_postscript, filepath):
#    
#    # If a postscript image file does not exist at the specified path, then 
#    # generate such a file by saving the reference image postscript data.
#    import os.path
#    if not os.path.exists(filepath):
#        with open(filepath, 'w') as f: f.write(image_postscript)
#    
#    # Load a copy of the reference postscript image data from disk.
#    with open(filepath, 'r') as f: ps = f.read()
#    
#    # Verify that the data loaded from disk matches the fixture.
#    assert tkinter_canvas_postscript.equals(ps, image_postscript)
#    
#  
#
## Test reference image 0.
#def test_reference_image_0(images_basepath, reference_image_0_postscript):
#    
#    # Initialize parameters.
#    filename = 'reference_image_0.ps'
#    postscript = reference_image_0_postscript
#    
#    # Initialize the file path.
#    from os import sep
#    filepath = f'{images_basepath}{sep}{filename}'
#    
#    # Test the generated image against the image stored on disk.
#    # Save the generated image to disk, if the file does not exist.
#    reference_image_disk_test(postscript, filepath)
#        
#  
#
## Test reference image 1.
#def test_reference_image_1(images_basepath, reference_image_1_postscript):
#    
#    # Initialize parameters.
#    filename = 'reference_image_1.ps'
#    postscript = reference_image_1_postscript
#    
#    # Initialize the file path.
#    from os import sep
#    filepath = f'{images_basepath}{sep}{filename}'
#    
#    # Test the generated image against the image stored on disk.
#    # Save the generated image to disk, if the file does not exist.
#    reference_image_disk_test(postscript, filepath)
    
  

# Test generated reference images against copies stored on disk.
# Save the generated image to disk, if any file does not exist.
def test_reference_images(reference_image_sequence, images_basepath):
    
    # Iterate through all reference images in the sequence.
    for (index, postscript) in enumerate(reference_image_sequence):
        
        # Initialize parameters.
        filename = f'reference_image_{index}.ps'
        
        # Initialize the file path.
        from os import sep
        filepath = f'{images_basepath}{sep}{filename}'
        
        # If a postscript image file does not exist at the specified path, then 
        # generate such a file by saving the reference image postscript data.
        # Save the generated image to disk, if the file does not exist.
        import os.path
        if not os.path.exists(filepath):
            with open(filepath, 'w') as f: f.write(postscript)
        
        # Load a copy of the reference postscript image data from disk.
        with open(filepath, 'r') as f: postscript_d = f.read()
        
        # Test the generated image against the image stored on disk.
        # Verify that the data loaded from disk matches the fixture.
        assert tkinter_canvas_postscript.equals(postscript_d, postscript)
    
  

# __main__
if __name__ == '__main__':
    pytest.main(['test_ros_tkinter_spheres_environment_gui.py'])
    
  


