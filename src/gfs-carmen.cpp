#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "stdint.h"

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PointStamped.h"
#include "fast_frontier_detector/PointArray.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>
#include <scanmatcher/FrontierManager.h>
#include <scanmatcher/gridlinetraversal.h>
//#include "tf/message_filter.h"
//#include "message_filters/subscriber.h"
//message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
//scan_sub_(nh_, "scan", 50);
// global variables
nav_msgs::OccupancyGrid mapData;
sensor_msgs::LaserScan laserdata;
geometry_msgs::PointStamped clickedpoint;
fast_frontier_detector::PointArray exploration_goal;
visualization_msgs::Marker points,line;
float xdim,ydim,resolution,Xstartx,Xstarty,init_map_x,init_map_y;
double m_laserMaxRange,m_initialBeamsSkip,m_laserAngles,m_usableRange,m_laserBeams;
//Subscribers callback functions---------------------------------------
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	mapData=*msg;
}

//Subscriber
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{	
    laserdata=*msg;
	//ROS_INFO("I heard: [%f]", msg->range_max);
  	//cout << msg->range_max <<endl;
	//Calculation of array size from angle range and angle increment.
	ROS_INFO("scanCallBack called");
}

int main(int argc, char **argv)
{

// generate the same numbers as in the original C test program
  ros::init(argc, argv, "fast_frontier_detector");
  ros::NodeHandle nh;
  
  // fetching all parameters
  float eta,init_map_x,init_map_y,range;
  std::string map_topic,base_frame_topic;
  
  std::string ns;
  ns=ros::this_node::getName();

  ros::param::param<float>(ns+"/eta", eta, 0.5);
  ros::param::param<std::string>(ns+"/map_topic", map_topic, "/robot_1/map"); 

//---------------------------------------------------------------
ros::Subscriber sub= nh.subscribe(map_topic, 100 ,mapCallBack);
ros::Subscriber lasersub = nh.subscribe("/robot_1/base_scan", 100, laserCallback);		

ros::Publisher targetspub = nh.advertise<fast_frontier_detector::PointArray>("/detected_points", 10);
tf::TransformListener listener;
ros::Rate rate(100); 
 
 
// wait until map is received, when a map is received, mapData.header.seq will not be < 1  
while (mapData.header.seq<1 or mapData.data.size()<1)  {  ros::spinOnce();  ros::Duration(0.1).sleep();}

/** Frontier Manager */
FrontierManager manager;

// Main loop
 while (ros::ok()){
 	tf::StampedTransform transform;
	int  temp=0;
	while (temp==0){
		try{
			temp=1;
			listener.lookupTransform("/robot_1/map", "/robot_1/base_link" , ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			temp=0;
			ros::Duration(0.1).sleep();
		}
	}

	OrientedPoint lp;
	lp.x=transform.getOrigin().x();
	lp.y=transform.getOrigin().y();
	lp.theta=transform.getRotation().getAngle();
	IntPoint p0=map.world2map(lp);


 	manager.clear();
	m_laserMaxRange=laserdata.range_max;
	m_initialBeamsSkip=laserdata.angle_min;
	m_usableRange=laserdata.range_max;
	double m_laserAngles[2880];
	double readings[2880];
	double theta = - std::fabs(laserdata.angle_min - laserdata.angle_max)/2;
    for(unsigned int i=0; i<2880-1; ++i)
    {
        m_laserAngles[i]=theta;
        theta += std::fabs(laserdata.angle_increment);
    }

    for(unsigned int i=0; i<2880-1; ++i)
    {
        readings[i]=laserdata.ranges[i];
    }
   /* for(unsigned int i=0; i<2880-1; ++i)
    {
        cout<<m_laserAngles[i]<<' '<<readings[i]<<endl;
    }*/


 	//const double * angle=m_laserAngles+m_initialBeamsSkip;
 	const double * angle=&m_laserAngles[0];
 	double esum=0;
	//for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++)
	for (const double* r=&readings[0]; r<&readings[2880-1]; r++, angle++){
		double d=*r;
		if (d>m_laserMaxRange)
			continue;
		if (d>m_usableRange)
			d=m_usableRange;
		Point phit=lp+Point(d*cos(lp.theta+*angle),d*sin(lp.theta+*angle));
		IntPoint p1=map.world2map(phit);
		IntPoint linePoints[20000] ;
		GridLineTraversalLine line;
		line.points=linePoints;
		GridLineTraversal::gridLine(p0, p1, &line);
		if (d<m_usableRange){
			manager.addReading(p1,SampleObstacle);
		}
		if (d == m_usableRange) { //matan
			manager.addReading(p1,SampleUnknown);
		}
	}
	//cout  << "informationGain=" << -esum << endl;
    
	// RUSAGE
	manager.calcFrontiers(map, map.world2map(lp));

	vector<IntLine> target;
	target = manager.getFrontiers();
	
	Point point;
	Point tempArray[20000];
	for (int i = 0; i < target.size(); ++i)
	{
		for (IntLine::iterator lItr = target[i].begin(); lItr != target[i].end(); ++lItr) {
			point.x = lItr->x;
			point.y = lItr->y;
			tempArray[i] = point;
		}
	}
	
	std::vector<Point> my_vector (tempArray, tempArray + sizeof(tempArray) / sizeof(Point));

    for (std::vector<Point>::iterator it = my_vector.begin(); it != my_vector.end(); ++it) {
        geometry_msgs::Point point;
        point.x = (*it).x;
        point.y = (*it).y;
        point.z = 0;
        exploration_goal.points.push_back(point);
    }

    targetspub.publish(exploration_goal);

 ros::spinOnce();
 rate.sleep();
   }
return 0;}
