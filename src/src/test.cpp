#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include<std_msgs/Int8MultiArray.h>
#include<std_msgs/String.h>


using namespace std;
 ros::Publisher pub_map;
 ros::Publisher pub_pos;
using namespace std;
float position;
float width;
float ct=0;
float cunt=0.00;
float y=0,x=0;

void mapCallback(const std_msgs::String::ConstPtr& msg)
{
    // This function is called whenever a new map message is received.
    // You can process the map data here.R
    cout<<" inside call back "<<endl;
  ROS_INFO("Received string: %s", msg->data.c_str());

  float num_float = std::stof(msg->data.c_str());

  position=num_float;
 
    
}
void mapCallback_width(const std_msgs::String::ConstPtr& msg1)
{
    // This function is called whenever a new map message is received.
    // You can process the map data here.R
    cout<<" inside call back "<<endl;
  ROS_INFO("Received string: %s", msg1->data.c_str());

  float num_float1 = std::stof(msg1->data.c_str());

  width=num_float1;
 
    
}

int main (int argc, char** argv)
{
 
  ros::init (argc, argv, "Occupanct_Grid");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/chatter", 1, mapCallback);
  ros::Subscriber sub1 = nh.subscribe("/chatter_width", 1, mapCallback_width);
  pub_map = nh.advertise<nav_msgs::OccupancyGrid> ("/map_new", 1);
 

    unsigned int c_x,c_y;
    std::vector<geometry_msgs::Point> polygon;
    geometry_msgs::Point pt;
   
    geometry_msgs::PoseStamped pose_out;
    //nav_msgs::OccupancyGrid::Ptr map_pub;
    boost::shared_ptr<nav_msgs::OccupancyGrid> map_pub;
    map_pub = boost::make_shared<nav_msgs::OccupancyGrid>(); 

    boost::shared_ptr<geometry_msgs::PoseStamped> pose;
    pose= boost::make_shared<geometry_msgs::PoseStamped>();
    
   

    map_pub->header.frame_id ="map";
    map_pub->header.stamp.sec=1;
    map_pub->header.stamp.nsec=1;
    map_pub->info.map_load_time.sec=1;
    map_pub->info.map_load_time.nsec=2;
    map_pub->info.resolution=0.1;
    map_pub->info.width=300;
    map_pub->info.height=300; 
    map_pub->info.origin.position.x=0;
    map_pub->info.origin.position.y=0;
    map_pub->info.origin.position.z=0;
    map_pub->info.origin.orientation.x=0;
    map_pub->info.origin.orientation.y=0;
    map_pub->info.origin.orientation.z=0;
    map_pub->info.origin.orientation.w=1;

 
    pose->header.frame_id="odom";
    pose->header.stamp.sec=2;
    pose->header.stamp.nsec=3;
    pose->pose.position.x =8.5;//robot x position
    pose->pose.position.y =5;//robot y position
    pose->pose.position.z = 0;
    pose->pose.orientation.w =0.923;
    pose->pose.orientation.x = 0;
    pose->pose.orientation.y =0;
    pose->pose.orientation.z =-0.865;

    cout<<" MAP SENT  "<<endl;
 
 for (unsigned i=0; i<map_pub->info.width*map_pub->info.height; i++)
    {
       
        map_pub->data.push_back(0);
    
    }
 
   while(ros::ok())
{ 
 cout<<"position"<<endl;
  cout<<position<<endl;
 
 cout<<"width"<<endl;
  cout<<width<<endl;
    //  cout<<"before data"<<endl;

      // c_x = int((pos - map_pub->info.origin.position.x) / map_pub->info.resolution);
	    // c_y = int((map_pub->info.origin.position.y - 2) / map_pub->info.resolution) + map_pub->info.height;
      // int i = c_x + (map_pub->info.height - c_y - 1) * map_pub->info.width;
      // map_pub->data[i] = 127;  
      // y=y+0.01;

  cunt=width/0.1;
 
    cout<<"count"<<endl;
  cout<<cunt<<endl;

         while(ct<cunt)
    {  
      c_x = int((position - map_pub->info.origin.position.x) / map_pub->info.resolution);
      c_y = int((map_pub->info.origin.position.y - y) / map_pub->info.resolution) + map_pub->info.height;
      int i = c_x + (map_pub->info.height - c_y - 1) * map_pub->info.width;
      map_pub->data[i] = 127;  
      y=y+0.1;
      ct=ct+1;
      // cout<<" x ------------------------------------------"<<endl;
      // cout<<position<<endl;
      cout<<" y -----------------------------------------"<<endl;
       cout<<y<<endl;
       
    }
     cout<<"at end data ---------------------------------------"<<endl;
     cout<<y<<endl;
    ct=0;
    y=0;
   pub_map.publish(map_pub);
  
    ros::spinOnce();
    //  loop_rate.sleep();
}   
  
//   ros::spin();
  return 0;
  

}


