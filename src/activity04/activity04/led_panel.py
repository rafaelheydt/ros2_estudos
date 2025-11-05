#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LedPanelState
from my_robot_interfaces.srv import SetLed

class LedPanelNode(Node): 
    def __init__(self):
        super().__init__("led_panel") 
        self.leds_state = [False, False, False]
        self.state_publisher_ = self.create_publisher(LedPanelState, "led_panel_state", 10)
        self.set_led_service_ = self.create_service(SetLed, "set_led", self.callback_set_led)
        

    def callback_set_led(self,request: SetLed.Request, response: SetLed.Response):
        state = request.state
        self.leds_state[request.led_number-1] = state

        msg = LedPanelState()
        msg.leds = self.leds_state
        self.state_publisher_.publish(msg)

        response.sucess =True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
