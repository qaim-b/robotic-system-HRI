#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PnnxToEmotionNode(Node):
    def __init__(self):
        super().__init__('pnnx_to_emotion_node')
        self.create_subscription(
            String,
            'pnnx2',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(String, 'emotion_state', 10)
        self.get_logger().info('pnnx_to_emotion_node started.')
        self.pub_face = self.create_publisher(String, 'face', 10)
        self.pub_motion = self.create_publisher(String, 'motion', 10)

    def listener_callback(self, msg):
        raw = msg.data.strip()
        try:
            valence = float(raw)
            # Clamp value between 0 and 1
            if valence < 0:
                valence = 0.0
            elif valence > 1:
                valence = 1.0
            self.get_logger().info(f'Publishing valence: {valence:.3f}')
            out = String()
            out.data = f"{valence:.3f}"
            self.publisher_.publish(out)
            
            if valence > 0.236: # Pleasant
                out.data = "happy5"
                self.pub_face.publish(out)
                #out.data = "LR,0.2,2.0,0.6,0"
                #self.pub_motion.publish(out)
            else: # Unpleasant
                out.data = "anger5"
                self.pub_face.publish(out)
                #out.data = "LR,0.4,2.0,0.6,0"
                #self.pub_motion.publish(out)
        except ValueError:
            self.get_logger().warning(f"Ignored non-numeric pNN50: '{raw}'")

def main(args=None):
    rclpy.init(args=args)
    node = PnnxToEmotionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
