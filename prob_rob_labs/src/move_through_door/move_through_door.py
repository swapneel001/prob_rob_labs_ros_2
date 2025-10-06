import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

heartbeat_period = 0.1

class MoveThroughDoor(Node):

    def __init__(self):
        super().__init__('move_through_door')
        self.log = self.get_logger()
        self.declare_parameter('robot_speed',1.0)
        self.pub_torque = self.create_publisher(Float64,'/hinged_glass_door/torque',1)
        self.pub_vel = self.create_publisher(Twist,'/cmd_vel',10)
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        self.robot_speed = self.get_parameter('robot_speed').get_parameter_value().double_value
        self.log.info(f'Robot speed is {self.robot_speed}')
        self.sub_feature_mean = self.create_subscription(Float64,'/feature_mean',self.check_feature_mean,1)
        self.feature_mean_value = 500
        self.torque = 5.0
        self.heartbeat_counter = 0
        self.moving_started = False
        self.stop_started = False
        self.close_started = False
        self.above330_counts = 0
        
    def heartbeat(self):
        self.log.info('heartbeat')
        
        # Before moving: keep opening the door until it's open enough (<280)
        if not self.moving_started:
            if self.feature_mean_value < 280.0:
                self.log.info('Door open detected (feature<280). Starting to move robot.')
                self.moving_started = True
                self.above330_ticks = 0
                self.move_robot()
            else:
                # keep trying to open
                self.log.info('Door closed. Opening door...')
                self.open_door()

        # While moving: continue moving; when feature_mean > 310 for 1s, stop then close
        elif self.moving_started and not self.close_started:
            if self.feature_mean_value > 330.0:
                self.above330_ticks += 1
                self.log.info(f'Feature>330 while moving, stop robot')
                if self.above330_ticks >= 20:
                    self.stop_robot()
                    self.log.info('Stopping robot , closing door.')
                    self.close_started = True
        
        elif self.close_started:
            self.close_door()



    def check_feature_mean(self, f):
        self.feature_mean_value = f.data
        self.log.info(f'Feature mean value is {self.feature_mean_value}')
        
    def open_door(self):
        self.log.info('Opening door')
        self.pub_torque.publish(Float64(data= self.torque))

    def close_door(self):
        self.log.info('Closing door')
        self.pub_torque.publish(Float64(data =-self.torque))
    def move_robot(self):
        vel_msg = Twist()
        vel_msg.linear.x = self.robot_speed
        self.log.info(f'Moving robot at speed {self.robot_speed}')

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
