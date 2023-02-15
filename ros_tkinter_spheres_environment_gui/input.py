""" ROS2 node for funneling mouse input to a cursor-based GUI.

Since mouse input is obtained from a Tkinter gui, this node must run in the 
same process as the GUI canvas. The Node class requires a reference to the GUI
object.

Relevant links:

* https://github.com/ros2/examples/blob/master/rclpy/executors/examples_rclpy_executors/composed.py
"""

# Copyright 2021 Andrew S. Whitford
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

import rclpy, rclpy.node
from geometry_msgs.msg import Point
from .node import Node as GUINode

DEFAULT_QOS_PROFILE = rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value


class Node(rclpy.node.Node):
    """ ROS2 node that publishes mouse input as cursor/position messages. """
    
    def __init__(self, *args, gui, node_name='input', **kwargs):
        super().__init__(*args, node_name=node_name, **kwargs)
        self.gui = gui
        self.initialize_controller()
        self.initialize_publisher()
        
    def initialize_controller(self):
        self.gui.bind_mouse(callback=self.on_mouse_movement)
        
    def initialize_publisher(self):
        qosp           = DEFAULT_QOS_PROFILE
        self.publisher = self.create_publisher(Point, 'cursor/position', qosp)
                
    def on_mouse_movement(self, event):
        (x, y) = self.gui.canvas.inverse_transform_coordinates(event.x, event.y)
        self.publisher.publish(Point(x=x, y=y, z=0.0))
        
    def destroy_node(self, *args, **kwargs):
        super().destroy_node(*args, **kwargs)
        
    #def __del__(self, *args, **kwargs): self.destroy_node()
    


def main(args=None):
    """ ROS2 entry point for running the GUI and mouse input nodes.
    
    Since mouse input is obtained from a Tkinter gui, this node must run in the 
    same process as the GUI canvas. The Node class requires a reference to the 
    GUI object. The nodes are run in the same process by adding both to a 
    ROS2 SingleThreadedExecutor.
    """
    
    rclpy.init(args=args)
    try:
        node_g = GUINode()
        node_i = Node(gui=node_g.gui)
        
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node_g)
        executor.add_node(node_i)
        
        try:
            executor.spin()
        except KeyboardInterrupt: pass
        finally:
            executor.shutdown()
            node_i.destroy_node()
            node_g.destroy_node()
    finally:
        rclpy.shutdown()
    


if __name__ == '__main__':
    main()
    

