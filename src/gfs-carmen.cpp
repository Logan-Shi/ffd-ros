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
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>
#include <scanmatcher/FrontierManager.h>


// global variables
nav_msgs::OccupancyGrid mapData;
geometry_msgs::PointStamped clickedpoint;
geometry_msgs::PointStamped exploration_goal;
visualization_msgs::Marker points,line;
float xdim,ydim,resolution,Xstartx,Xstarty,init_map_x,init_map_y;


//Subscribers callback functions---------------------------------------
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	mapData=*msg;
}

//Subscriber
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
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
ros::Subscriber lasersub = nh.subscribe("/robot_1/filtered_scan", 100, laserCallback);		

ros::Publisher targetspub = nh.advertise<geometry_msgs::PointStamped>("/detected_points", 10);

ros::Rate rate(100); 
 
 
// wait until map is received, when a map is received, mapData.header.seq will not be < 1  
while (mapData.header.seq<1 or mapData.data.size()<1)  {  ros::spinOnce();  ros::Duration(0.1).sleep();}

/** Frontier Manager */
FrontierManager manager;

// Main loop
while (ros::ok()){

	manager.clear();
	
	const double * angle=m_laserAngles+m_initialBeamsSkip;
	double esum=0;
	for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++)
		if (m_generateMap){
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
			for (int i=0; i<line.num_points-1; i++){
				PointAccumulator& cell=map.cell(line.points[i]);
				double e=-cell.entropy();
				cell.update(false, Point(0,0));
				e+=cell.entropy();
				esum+=e;
			}
			if (d<m_usableRange){
				double e=-map.cell(p1).entropy();
				map.cell(p1).update(true, phit);
				e+=map.cell(p1).entropy();
				esum+=e;
				manager.addReading(p1,SampleObstacle);
			}

			if (d == m_usableRange) { //matan
				manager.addReading(p1,SampleUnknown);
			}

		} else {
			if (*r>m_laserMaxRange||*r>m_usableRange) continue;
			Point phit=lp;
			phit.x+=*r*cos(lp.theta+*angle);
			phit.y+=*r*sin(lp.theta+*angle);
			IntPoint p1=map.world2map(phit);
			assert(p1.x>=0 && p1.y>=0);
			map.cell(p1).update(true,phit);
		}
	//cout  << "informationGain=" << -esum << endl;

	// RUSAGE
	manager.calcFrontiers(map, map.world2map(p));

	//publish frontier point
	exploration_goal.header.stamp=ros::Time(0);
    exploration_goal.header.frame_id=mapData.header.frame_id;
    exploration_goal.point.x=0;
    exploration_goal.point.y=0;
    exploration_goal.point.z=0.0;

    targetspub.publish(exploration_goal);

ros::spinOnce();
rate.sleep();
  }return 0;}
