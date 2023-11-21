// This program uses OpenCV, PyTorch, and ROS libraries
#include <opencv2/opencv.hpp>
#include <torch/torch.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <realsense2/rs.hpp>
#include <statistics.h>

#include "test/realsense_camera.h"
RealsenseCamera rs;
int point[2] = {0, 0};
int xA = 0;
int xB = 0;
int yA = 0;
int yB = 0;
int distance = 0;
auto model = torch::hub::load("ultralytics/yolov5", "custom", "/home/light/re_ws/src/test/src/yolov5-best.pt", true);

void callback(const std_msgs::String::ConstPtr& data)
{
    ROS_INFO("I heard: [%s]", data->data.c_str());
}

void talker()
{
    ros::Publisher pub = n.advertise<std_msgs::String>("chatter", 10);
    ros::NodeHandle n;
    ros::Rate rate(10); // 10hz
    while (ros::ok())
    {
        int dis = processing();
        std::stringstream ss;
        ss << dis << " at time " << ros::Time::now();
        std_msgs::String msg;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }
}

int processing()
{
    global distance;
    while (true)
    {
        try
        {
            bool ret;
            cv::Mat color, depth;
            std::tie(ret, color, depth) = rs.get_frame_stream();
            if (ret == true)
            {
                std::cout << "no frame received" << std::endl;
            }
            auto results = model.forward(color);
            for (auto box : results.xyxy[0])
            {
                xB = int(box[2]);
                xA = int(box[0]);
                yB = int(box[3]);
                yA = int(box[1]);
                int center_x = (xA + xB) / 2;
                int center_y = (yA + yB) / 2;
                // std::cout << center_x << std::endl;
                // std::cout << center_y << std::endl;
                point[0] = int(center_x);
                point[1] = int(center_y);
            }

            distance = depth.at<int>(point[0], point[1]);
            // std::cout << "beforemsg->info.width median" << distance << std::endl;
            // std::cout << "type---------->" << typeid(depth).name() << std::endl;
            // int dis=    st.median(distance);
            // std::cout << "after median" << dis << std::endl;
            std::cout << results.pandas().xyxy[0] << std::endl;
            // std::cout << "new distance ---------->" << rs.get_distance(point[0],point[1]) << std::endl;
            cv::putText(color, std::to_string(distance / 10) + "cm", cv::Point(point[0], point[1] - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            cv::circle(color, cv::Point(point[0], point[1]), 4, cv::Scalar(0, 0, 255));
            cv2.rectangle(color, cv2.Point(xA, yA), cv2.Point(xB, yB), cv2.Scalar(0, 255, 0), 2);

            cv2.imshow("YOLO Detection", color);
            // cv2.imshow("YOLO Detectin", depth);
            if (cv2.waitKey(1) == 'q')
            {
                rs.release();
                cv2.destroyAllWindows();
                break;
            }
        }
        catch (...)
        {
            std::cout << "out of range" << std::endl;
        }
        return distance;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "talker");
    talker();
    // listener();
    // processing();
}

