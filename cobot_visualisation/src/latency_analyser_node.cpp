#include <stdio.h>

#include <ros/ros.h>
#include <ros/master.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <tf/tf.h>
#include <std_msgs/String.h>

#include "StopWatch.h"

float pi = 3.14159265359;

static const std::string THIS_PACKAGE = "cobot_visualisation";
static const std::string TOPIC_PUB_OBJECT_DATA = "/DetectedObjects";
static const std::string TOPIC_SUB_UR5_CONTROL ="ur5_control";
ros::Publisher pub_object_data;


StopWatch latencyTimer;
int pubCount=0;


visualization_msgs::MarkerArray spreadObjects(geometry_msgs::Point p1, geometry_msgs::Point p2, visualization_msgs::Marker m, int numObjects){
    geometry_msgs::Vector3 dist;
    dist.x=(p2.x-p1.x)/numObjects;
    dist.y=(p2.y-p1.y)/numObjects;
    dist.z=(p2.z-p1.z)/numObjects;

    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker marker =m;
    m.pose.position.x=p1.x;
    m.pose.position.y=p1.y;
    m.pose.position.z=p1.z;
    for(int i = 0; i<numObjects;i++){
        array.markers.push_back(m);
        m.pose.position.x+=dist.x;
        m.pose.position.y+=dist.y;
        m.pose.position.z+=dist.z;
    }
    return array;
}

int numObjects=1;
void ur5_control_callback(std_msgs::String data){
    latencyTimer.stop();
    printf("%d: Got response: .%s. in %0.6f seconds\n\n",pubCount,data.data.c_str(),latencyTimer.lastTime());
    if(data.data.compare("stop")==0){
        numObjects*=2;
    }
}


void pub_callback(const ros::TimerEvent&){
    visualization_msgs::Marker marker;
    marker.scale.x=0.2;     marker.color.r=255;
    marker.scale.y=0.2;     marker.color.g=255;
    marker.scale.z=0.2;     marker.color.b=0;

    geometry_msgs::Point p1;    geometry_msgs::Point p2;
    p1.x=-2;                    p2.x=2;
    p1.y=0;                     p2.y=0;
    p1.z=p2.z=0.5+2*(sin(ros::Time::now().toSec())>0);

    pub_object_data.publish(spreadObjects(p2,p1,marker,numObjects));
    latencyTimer.start();
    pubCount++;
    printf("%d: Published %d objects z=%0.2f\n",pubCount,numObjects,p1.z);

}

int main(int argc, char **argv){
    ros::init(argc,argv,"latency_analyser_node");
    ros::NodeHandle n;
    pub_object_data = n.advertise<visualization_msgs::MarkerArray>(TOPIC_PUB_OBJECT_DATA, 1);
    ros::Timer pub_timer = n.createTimer(ros::Duration(1),pub_callback);
    ros::Subscriber sub =n.subscribe(TOPIC_SUB_UR5_CONTROL, 1, ur5_control_callback);
    
    ROS_INFO("latency_analyser_node started");

    ros::spin();
    return 0;
}

