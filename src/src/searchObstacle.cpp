#include <ros/ros.h>
// #include<code1/FootPrintUtils.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include<std_msgs/Int8MultiArray.h>


using namespace std;
 ros::Publisher pub_map;
 ros::Publisher pub_pos;
using namespace std;
int main (int argc, char** argv)
{
 
  ros::init (argc, argv, "Occupanct_Grid");
  ros::NodeHandle nh;
  // FootPrintUtils fp;
  

  //ros::Subscriber sub = nh.subscribe ("/map", 1, cloud_cb);

  
 pub_map = nh.advertise<nav_msgs::OccupancyGrid> ("/map_new", 1);
 

    unsigned int c_x,c_y;
    std::vector<geometry_msgs::Point> polygon;
    geometry_msgs::Point pt;
    float y=3,x=3;
    geometry_msgs::PoseStamped pose_out;
    //nav_msgs::OccupancyGrid::Ptr map_pub;
    boost::shared_ptr<nav_msgs::OccupancyGrid> map_pub;
    map_pub = boost::make_shared<nav_msgs::OccupancyGrid>(); 

    boost::shared_ptr<geometry_msgs::PoseStamped> pose;
    pose= boost::make_shared<geometry_msgs::PoseStamped>();
    
   

    map_pub->header.frame_id ="odom";
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
      ///creating a empty map

    for (unsigned i=0; i<map_pub->info.width*map_pub->info.height; i++)
    {
       
        map_pub->data.push_back(0);
    
    }
    //inserting obstacle into the map
      //object x and y coordinates enter below
     cout<<"before data"<<endl;
   while(y<20)
    {  
      c_x = int((8 - map_pub->info.origin.position.x) / map_pub->info.resolution);
	    c_y = int((map_pub->info.origin.position.y - y) / map_pub->info.resolution) + map_pub->info.height;
      int i = c_x + (map_pub->info.height - c_y - 1) * map_pub->info.width;
      map_pub->data[i] = 127;
      y=y+0.01;
    //  cout<<"inside data "<<endl;
    }
  
    //  while(x<20)
    // {  
    //   c_x = int((x - map_pub->info.origin.position.x) / map_pub->info.resolution);
	  //   c_y = int((map_pub->info.origin.position.y - 5) / map_pub->info.resolution) + map_pub->info.height;
    //   int i = c_x + (map_pub->info.height - c_y - 1) * map_pub->info.width;
    //   map_pub->data[i] = 127;
    //   x=x+0.01;
    //  // cout<<"inside data "<<endl;
    // }
 
    ///sending fake map to the footprintutlis class
    // fp.updateCostMapPtr(map_pub);
    
   
    pt.x=1.23;pt.y=0.43;polygon.push_back(pt);
    pt.x=1.23; pt.y=-0.43; polygon.push_back(pt);
    pt.x=-0.42;pt.y=-0.43;polygon.push_back(pt); 
     pt.x=-0.42;pt.y=0.43; polygon.push_back(pt); 
    
   //sending polygon to the footprintutlis class
    // fp.setFootprint(polygon);
    cout<<"  Before isCheckright  "<<endl;
    //  bool isCheckright=fp.searchObstacleFootprint(pose,pose_out,3);
    // if(isCheckright==true)
    // {
    //  cout<<"obstacle cleared present"<<endl;
    // cout<<"pose_out.position.x : "<<pose_out.pose.position.x <<" pose_out.position.y : "<<pose_out.pose.position.y<<endl;
    // cout<<"z : "<<pose_out.pose.orientation.z<<" x : "<<pose_out.pose.orientation.x<<" y : "<<pose_out.pose.orientation.y<<" w : "<<pose_out.pose.orientation.w<<endl ;
    // }
    // else{cout<<"obstacle present"<<endl;
      
    // }
while(ros::ok())
{ 
   pub_map.publish(map_pub);
    ros::spinOnce();
}
 // ros::spin ();
}


