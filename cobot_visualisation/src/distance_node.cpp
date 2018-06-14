#include <stdio.h>
#include <math.h>

#include <ros/ros.h>
#include <ros/master.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <cobot_visualisation/VisualisationData.h>

#include <string>
#include <tf/transform_listener.h>

#include "StopWatch.h" //StopWatch class for keeping track of time;

static const std::string THIS_PACKAGE = "cobot_visualisation";
static const std::string BASE_FRAME = "/world";

static const std::string TOPIC_SUB_OBJECT_DATA = "/DetectedObjects";

static const std::string TOPIC_PUB_VISUALISATION_DATA = THIS_PACKAGE+"/visualisation_data";
static const std::string TOPIC_PUB_SAFETY_STATUS = THIS_PACKAGE+"/safety_status";
static const std::string TOPIC_PUB_UR5_CONTROL ="ur5_control";

//Define publishers
ros::Publisher pub_visualisation_data;
ros::Publisher pub_safety_status;
ros::Publisher pub_ur5_control;

//Frames from robot to calculate distance from:
static const std::vector<std::string> frames={"world","forearm_link","tool0"};

//System parameters:
static const float TRIGGER_DISTANCE = 1.26;//meter
static const float COOLDOWN_TIME = 2.5;//seconds
static const float OBJECT_CALLBACK_TIMEOUT = 2;//seconds

static float pi = 3.14159265359;
float closest_distance=-1;
geometry_msgs::Point closest_frame;
geometry_msgs::Point closest_object;
std::string status="waiting for data";
std::string statusOld=" ";
visualization_msgs::MarkerArray objectData;

StopWatch cooldownTimer;
StopWatch callbackTimer;

//Move object origin as close as possible to the frame origin without leaving the object
geometry_msgs::Point moveOriginTowardsFrame(geometry_msgs::Point origin, geometry_msgs::Vector3 scale, geometry_msgs::Point frame){
    //Set the maximum distance the origin is allowed to move in each axis which keeps the origin within the objects boundries.
    scale.x/=2;
    scale.y/=2;
    scale.z/=2;
    
    geometry_msgs::Vector3 dir;
    //Determine in which direction the origin has to move
    dir.x=1-(origin.x>frame.x)*2; //Returns -1 if object.x> frame.x  
    dir.y=1-(origin.y>frame.y)*2; //Returns  1 if object.x<=frame.x
    dir.z=1-(origin.z>frame.z)*2;

    //Move origin to the new position:
    //origin+=distance_to_move*direction
    //std::min(..,..) takes the lowest value
    origin.x+=std::min(std::abs(origin.x-frame.x),scale.x)*dir.x;
    origin.y+=std::min(std::abs(origin.y-frame.y),scale.y)*dir.y;
    origin.z+=std::min(std::abs(origin.z-frame.z),scale.z)*dir.z;
    return origin;
}

void calcObjectDistance(){
    //Set defaults for distance and closest frame+object. (draws line in the air)
    closest_distance=999;
    closest_frame.x=0;  closest_object.x=0;
    closest_frame.y=1;  closest_object.y=-1;
    closest_frame.z=1;  closest_object.z=1;

    //Calculate actual distance if objects are available:
    if(objectData.markers.size()>0){
        for(int i=0; i<frames.size();i++){
            try{
                //Calculate relative position between BASE_FRAME and frame i in frames
                tf::TransformListener tr;
                tr.waitForTransform(BASE_FRAME,frames[i],ros::Time(0),ros::Duration(1.0));

                tf::StampedTransform transform;
                tr.lookupTransform(BASE_FRAME,frames[i],ros::Time(0),transform);

                geometry_msgs::TransformStamped msg;
                tf::transformStampedTFToMsg(transform, msg);

                geometry_msgs::Point frame;
                //Load frame origin
                frame.x=msg.transform.translation.x;
                frame.y=msg.transform.translation.y;
                frame.z=msg.transform.translation.z;

                for(int j=0; j<objectData.markers.size();j++){
                    geometry_msgs::Point object;

                    //Move object origin as close as possible to the frame origin without leaving the object
                    //  This makes it possible to calculate the distance between the surface of the object and the frame origin.
                    object=moveOriginTowardsFrame(objectData.markers[j].pose.position,objectData.markers[j].scale,frame);

                    float dx=object.x-frame.x;
                    float dy=object.y-frame.y;
                    float dz=object.z-frame.z;
                    float dist=sqrt(dx*dx+dy*dy+dz*dz);
                    //printf("distance between frame %i and object %i:%0.2f\n",i,j,dist);
                    if(dist<closest_distance){
                        closest_distance=dist;
                        closest_frame=frame;
                        closest_object=object;
                    }
                }
            }catch(...){
                ROS_INFO("Transform error");
                //Maybe perform safety actions here
            }
        } 
    }
}

void publishData(){
    //Publish status when status has changed
    if(status.compare(statusOld)!=0 && status.compare("cooldown")!=0){ //status != statusOld
        std_msgs::String stat;
        stat.data=status;
        pub_safety_status.publish(stat);
        pub_ur5_control.publish(stat);
    }
    statusOld=status;

    //Combine data in one message for visualisation and publish
    cobot_visualisation::VisualisationData visData;
    visData.objects.markers=objectData.markers;
    visData.closestFrame=closest_frame;
    visData.closestObject=closest_object;
    visData.closestDistance=closest_distance;
    visData.statusMessage=status;
    visData.statusDescription="statusDescription is not implemented yet";
    pub_visualisation_data.publish(visData);

}

void distanceResponse(){
    if(closest_distance<TRIGGER_DISTANCE){
        status="stop";
        cooldownTimer.start();
    }else if(cooldownTimer.elapsed()<COOLDOWN_TIME){
        status="cooldown";
    }else{
        status="start";
    }
}

void objectCallback(visualization_msgs::MarkerArray data){
    callbackTimer.start(); //(re)start callbackTimer
    objectData=data;       //Transfer data
    calcObjectDistance();
    distanceResponse();
    printf("numObjects:%i distance:%0.2f status:%s\n",int(data.markers.size()),closest_distance,status.c_str());
    publishData();
}


//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------


void mainLoop(const ros::TimerEvent&){
    bool error=0;
    if(!callbackTimer.started()){
        ROS_INFO("Waiting for object data! status:%s",status.c_str());
        error=1;
    }

    if(callbackTimer.elapsed()>OBJECT_CALLBACK_TIMEOUT){
        ROS_INFO("Object data timed out! status:%s",status.c_str());
        error=1;
    }

    if(error){
        status="stop";
        closest_frame.x=0;  closest_object.x=0;
        closest_frame.y=1;  closest_object.y=-1;
        closest_frame.z=1;  closest_object.z=1;
        closest_distance=0;
        publishData();
    }
}


int main(int argc, char **argv){
    ros::init(argc,argv,"distance_node");
    ros::NodeHandle n;

    //ros::Duration(5).sleep();
    ROS_INFO("distance_node started");
    cooldownTimer.init();

    //Setup subscribers
    ros::Subscriber sub = n.subscribe(TOPIC_SUB_OBJECT_DATA, 1, objectCallback);
    //Initialise publishers
    pub_visualisation_data = n.advertise<cobot_visualisation::VisualisationData>(TOPIC_PUB_VISUALISATION_DATA, 1);
    pub_safety_status = n.advertise<std_msgs::String>(TOPIC_PUB_SAFETY_STATUS, 1);
    pub_ur5_control = n.advertise<std_msgs::String>(TOPIC_PUB_UR5_CONTROL, 1);

    ros::Timer mainTimer = n.createTimer(ros::Duration(0.33),mainLoop);

    ros::spin();
    return 0;
}