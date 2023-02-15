""" ROS2 node that links a Tkinter GUI to the ROS2 graph, such that the 
    properties of objects in the GUI can be modified via ROS2 messages.

Examples
--------

>>> import rclpy
>>> rclpy.init()

>>> spin = lambda n: rclpy.spin_once(n, timeout_sec=0.005)

>>> node = Node()

>>> from ros_spheres_environment import Client
>>> client_node = rclpy.node.Node('client')
>>> client = Client(node=client_node)

>>> client.initialize_object('cursor')
>>> spin(client_node)
>>> spin(node)
>>> node.environment['cursor']
{'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'radius': 1.0}

>>> client['cursor'].radius = 0.10
>>> client['cursor'].position = (0.25, -0.10, 0.0)
>>> spin(client_node)
>>> spin(client_node)
>>> spin(node)
>>> spin(node)
>>> node.environment['cursor']
{'position': {'x': 0.25, 'y': -0.1, 'z': 0.0}, 'radius': 0.1}

>>> del client
>>> del server
>>> client_node.destroy_node()
>>> node.destroy_node()
>>> rclpy.shutdown()


"""

# Copyright 2022-2023 Carnegie Mellon University Neuromechatronics Lab (a.whit)
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# Contact: a.whit (nml@whit.contact)


# Import ROS.
import rclpy.node

# Import tkinter_spheres_environment_gui.
from tkinter_spheres_environment_gui import Environment

# Import ros_spheres_environment.
import ros_spheres_environment
#from ros_spheres_environment import Server


# Server class.
# This is a quick hack to facilitate integration with the 
# `ros_delay_out_center_task` package.
# This modification moves the `cursor` object to the foreground whenever the 
# position of ANY object is modified.
# https://github.com/ricmua/ros_tkinter_spheres_environment_gui/issues/1
class Server(ros_spheres_environment.Server):
    
    def initialize_object(self, *args, key, **kwargs):
        
        # Initialize the superclass method.
        super().initialize_object(*args, key=key, **kwargs)
        
        # Prepare a callback for setting the z-order of the cursor.
        def callback(message):
            if 'cursor' in self.environment:
                self.environment.cursor.to_foreground()
        
        # Create a new subscriber for position messages.
        property_key = 'position'
        
        # Retrieve the default message type for the object property.
        msg_type = self.DEFAULT_PROPERTY_MESSAGE_MAP[property_key]
        
        # Prepare keyword arguments.
        topic = f'{key}/{property_key}'
        kwargs = {**self.DEFAULT_TOPIC_PARAMETER_RECORD,
                  'msg_type': msg_type,
                  'topic': topic,
                  'callback': callback,
                  **self._topic_parameter_map.get(topic, {})}
        
        # Initialize a subscription.
        self._z_order_subscription \
          = self.node.create_subscription(**kwargs)
        
        # Redundant, and perhaps a simpler solution.
        # Change the z-order when objects are initialized.
        if 'cursor' in self.environment:
            self.environment.cursor.to_foreground()
    
  

# Node class.
class Node(rclpy.node.Node):
    """ A node that implements a linkage between a GUI representation of a 
        virtual environment and the ROS2 graph.
    
    Arguments
    ---------
    *args : list
        Arguments passed to the superclass (`rclpy.node.Node`) constructor.
    node_name : str
        Name of the ROS2 node.
    **kargs : dict
        Keyword arguments passed to the superclass (`rclpy.node.Node`) 
        constructor.
    """
    def __init__(self, *args, node_name='gui', **kwargs):
        
        # Invoke the superclass constructor.
        super().__init__(*args, node_name=node_name, **kwargs)
        
        # Declare a background color parameter.
        # This is a stopgap feature.
        # It can be removed when a color topic has been implemented.
        self.declare_parameter('canvas.color', 'black')
        
        # Initialize the spheres environment GUI.
        self.gui = self.environment = Environment()
        
        # Set the background color.
        # This is a stopgap feature.
        # It can be removed when a color topic has been implemented.
        background_color = self.get_parameter('canvas.color').value
        canvas = self.environment.gui.canvas
        canvas.background_color = background_color
        canvas.update()
        
        # Initialize a server interface to link the GUI to ROS2.
        self.server = Server(node=self, environment=self.gui)
        
    def __del__(self):
        """ Destructor cleans up the server and Tkinter GUI. """
        del self.server
        del self.gui
        #super().__del__()
    
  

# __main__
if __name__ == '__main__':
    import doctest
    doctest.testmod()
    
  


