import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
import time

from rosidl_parser.definition import BoundedSequence



waypoints_x = [-6.287268377005353,-1.1813752419060253, 2.629514445760446, 3.184386182952682, 0.8882434924923639,0.9396438941589098 , -5.841809047647793 , 4.091897477921052, 7.529818014035269 , 4.621444372054703 , 10.112957277409249]
waypoints_y = [-4.479027182807097, 2.7512986406561306, 1.1433171895991998, 4.65312763347442, -4.488522700301216,-3.3096329535696447 , 4.609365808146734 , -8.171539094439712, -8.197913975665248 , 7.775613826036862 , -2.7602363476567153]
waypoints_z = [-0.15764436037503124,-0.11679901400631035, -0.11913263833051216, 0.9986217129709447, -0.19869342146245125, 0.7288682262187526 , 0.9982893186587009 , -0.5783819612277716, 0.07956194395952955 , 0.612393655616828 , 0.0191252367635799]
waypoints_w = [0.9874959522154748,0.9931555720667099, 0.9928783482805995, 0.05248499197843286, 0.9800616941129495, 0.6846540066403828 , 0.05846739477646419 , 0.8157660859133067, 0.996829923845277 , 0.7905529777062749 , 0.9998170959324195]





class robotPublisher(Node):
    def __init__(self):
        super().__init__('robotPublisher')
        
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.msg_goal = NavigateToPose.Goal()
        self.index = 0
        self.send_goal()
        
        
    def send_goal(self):
        self.msg_goal.pose.pose.position.x = waypoints_x[self.index]
        self.msg_goal.pose.pose.position.y = waypoints_y[self.index]
        self.msg_goal.pose.pose.position.z = 0.0
        self.msg_goal.pose.pose.orientation.x = 0.0
        self.msg_goal.pose.pose.orientation.y = 0.0
        self.msg_goal.pose.pose.orientation.z = waypoints_z[self.index]
        self.msg_goal.pose.pose.orientation.w = waypoints_w[self.index]
        self.index = self.index + 1
        self.action_client.wait_for_server()
        print(f'x: {self.msg_goal._pose.pose.position.x}\ny: {self.msg_goal._pose.pose.position.y}\nw: {self.msg_goal._pose.pose.orientation.w}')
        self.send_goal_future = self.action_client.send_goal_async(self.msg_goal, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.response_callback)


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    def response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        print(result)
        print('Arrived at location')
        
        if self.index < 11:
            time.sleep(5)
            self.send_goal()
        else:
            rclpy.shutdown()

        
            

    

def main():
    rclpy.init(args=None)
    rp = robotPublisher()
    rclpy.spin(rp)

if __name__ == '__main__':
    main()




 

 