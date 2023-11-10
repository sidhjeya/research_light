# subscriber_node.py

import rospy
from std_msgs.msg import String  # Adjust the message type as needed
from test_class import DataProcessor
data_processor = DataProcessor()
def callback(data):
    # Process the data using your class
    data_processor.process_data(data.data)

def main():
    rospy.init_node('subscriber_node')
  

    # Subscribe to the topic you're interested in
    rospy.Subscriber('your_topic_name', String, callback)

    rospy.spin()

if __name__ == '__main__':
    main()
