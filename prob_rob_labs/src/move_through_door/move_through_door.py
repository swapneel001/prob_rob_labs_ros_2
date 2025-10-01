import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

torque_hold = 10
heartbeat_period = 0.1

class MoveThroughDoor(Node):

    def __init__(self):
        super().__init__('move_through_door')
        self.log = self.get_logger()
        self.pub_torque = self.create_publisher(Float64,'/hinged_glass_door/torque',1)
        self.pub_vel = self.create_publisher(Twist,'/cmd_vel',10)
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        self.torque = 5.0
        self.heartbeat_counter = 0
        
    def heartbeat(self):
        self.log.info('heartbeat')
        if self.heartbeat_counter<50:
            self.open_door()
        elif self.heartbeat_counter<100:
            self.move_robot()
        elif self.heartbeat_counter<120:
            self.stop_robot()
        else:
            self.close_door()
        self.heartbeat_counter+=1

        
    def open_door(self):
        self.log.info('Opening door')
        self.pub_torque.publish(Float64(data =self.torque))
    def close_door(self):
        self.log.info('Closing door')
        self.pub_torque.publish(Float64(data =-self.torque))
    def move_robot(self):
        self.log.info('Moving robot')
        vel_msg = Twist()
        vel_msg.linear.x = 1.0
        self.pub_vel.publish(vel_msg)
    def stop_robot(self):
        self.log.info('Stopping robot')
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        self.pub_vel.publish(vel_msg)
    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    move_through_door = MoveThroughDoor()
    move_through_door.spin()
    move_through_door.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
