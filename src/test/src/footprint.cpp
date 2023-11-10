// #include <code1/FootPrintUtils.h>
#include <geometry_msgs/PolygonStamped.h>
#include <ros/ros.h>

using namespace std;

ros::Publisher pub_poly;

// FootPrintUtils fp;

int main(int argc, char **argv) {

  ros::init(argc, argv, "Footprint");
  ros::NodeHandle nh;

  std::vector<geometry_msgs::Point32> polygon;
  geometry_msgs::PolygonStamped footprint_poly;
  geometry_msgs::Point32 pt;
  double x = 10;
  double y = 10;
  double rad = 105 * 3.142 / 180;

  pub_poly = nh.advertise<geometry_msgs::PolygonStamped>("/footprint", 1);
  footprint_poly.header.frame_id = "odom";
  footprint_poly.header.stamp = ros::Time::now();

  // pt.x = 1.23;
  // pt.y = 0.43;
  // polygon.push_back(pt);
  // pt.x = 1.23;
  // pt.y = -0.43;
  // polygon.push_back(pt);
  // pt.x = -0.42;
  // pt.y = -0.63;
  // polygon.push_back(pt);
  // pt.x = -0.42;
  // pt.y = 0.63;
  // polygon.push_back(pt);
  // pt.x = -2;
  // pt.y = 3;
  // polygon.push_back(pt);
  // pt.x = 5;
  // pt.y = 1;
  // polygon.push_back(pt);
  // pt.x = 5;
  // pt.y = -3;
  // polygon.push_back(pt);
  // pt.x = -2;
  // pt.y = -1;
  // polygon.push_back(pt);
  pt.x = -5;
  pt.y = 5;
  polygon.push_back(pt);
  pt.x = 5;
  pt.y = 5;
  polygon.push_back(pt);
  pt.x = 5;
  pt.y = -5;
  polygon.push_back(pt);
  pt.x = -5;
  pt.y = -5;
  polygon.push_back(pt);
  footprint_poly.polygon.points.resize(4);

  /////footprint change
  double cos_th = cos(rad);
  double sin_th = sin(rad);

  std::vector<geometry_msgs::Point32> oriented_footprint;
  for (unsigned int i = 0; i < polygon.size(); ++i) {
    geometry_msgs::Point32 new_pt;

    new_pt.x = x + (polygon[i].x * cos_th - polygon[i].y * sin_th);
    new_pt.y = y + (polygon[i].x * sin_th + polygon[i].y * cos_th);
    oriented_footprint.push_back(new_pt);
    cout << " new_pt->" << new_pt << endl;
  }
  /////transfer footprint to new position
  for (unsigned int i = 0; i < 4; i++) {
    footprint_poly.polygon.points[i].x = oriented_footprint[i].x;
    footprint_poly.polygon.points[i].y = oriented_footprint[i].y;
    footprint_poly.polygon.points[i].z = oriented_footprint[i].z;
    /*  footprint_poly.polygon.points[i]=oriented_footprint[i]; */
  }

  while (ros::ok()) {

    pub_poly.publish(footprint_poly);
    ros::spinOnce();
  }
}
