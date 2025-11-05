#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed

class BatteryNode(Node): 
    def __init__(self):
        super().__init__("battery") 
        self.battery_state = True
        self.led_client_ = self.create_client(SetLed, "set_led")
        self.discharge_timer_ = self.create_timer(4, self.callback_discharge)
        
    def callback_discharge(self):

        request = SetLed.Request()
        request.led_number = 3
        request.state = True

        future = self.led_client_.call_async(request)
        future.add_done_callback(self.callback_set_led)

        self.charging_timer_ = self.create_timer(6, self.callback_charged)
        self.discharge_timer_.cancel()
        
    def callback_charged(self):
        request = SetLed.Request()
        request.led_number = 3
        request.state = False

        future = self.led_client_.call_async(request)
        future.add_done_callback(self.callback_set_led)
    
        self.charging_timer_.cancel()
        self.discharge_timer_ = self.create_timer(4, self.callback_discharge)

    def callback_set_led(self, future):
        response = future.result()
        self.get_logger().info(f"Got response: {response.sucess}" )
        

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
