#include <stdio.h>

#include <ros/ros.h>
#include <ros/master.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <tf/tf.h>

float pi = 3.14159265359;


geometry_msgs::PoseArray drawCircle(geometry_msgs::PoseArray pArray){
    float t=ros::Time::now().toSec();
    int numObjects=1+int(t)%10;
    for(int i = 0; i<numObjects;i++){
        geometry_msgs::Pose p;
        p.position.x=sin(pi*2/numObjects*i)*2;
        p.position.y=cos(pi*2/numObjects*i)*2;
        p.position.z=0;

        pArray.poses.push_back(p);
    }
    return pArray;
}

geometry_msgs::PoseArray drawStack(float x, float y,float z, int height,geometry_msgs::PoseArray pArray){
    int numObjects=height;
    for(int i = 0; i<numObjects;i++){
        geometry_msgs::Pose p;
        p.position.x=x;
        p.position.y=y;
        p.position.z=z+i*0.1;

        pArray.poses.push_back(p);
    }
    return pArray;
}

int main(int argc, char **argv){
    ros::init(argc,argv,"object_publisher_node");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("/DetectedObjects", 100);
    
    ROS_INFO("object_publisher_node started");
    
    

    ros::Rate loop_rate(1.5);
    int pubCount=0;
    while (ros::ok())
    {   
        float t=ros::Time::now().toSec();
        visualization_msgs::MarkerArray mArray;
        visualization_msgs::Marker marker;

        // marker.scale.x=0.2;
        // marker.scale.y=0.2;
        // marker.scale.z=0.2;

        // marker.pose.position.x=sin(t/5);
        // marker.pose.position.y=-0.5;
        // //marker.pose.position.y=0.65;
        // //marker.pose.position.z=cos(t/5)*0.15;
        // marker.pose.position.z=0.13;

        // marker.color.r=255;
        // marker.color.g=255;
        // marker.color.b=0;

        // mArray.markers.push_back(marker);

        // marker.pose.position.x=cos(t/5);
        // marker.pose.position.y=0.5;
        // //marker.pose.position.y=0.65;
        // //marker.pose.position.z=cos(t/5)*0.15;
        // marker.pose.position.z=0.13;

        // marker.color.r=0;
        // marker.color.g=255;
        // marker.color.b=255;

        // mArray.markers.push_back(marker);
        for(int i = 0; i<2;i++){
            marker.scale.x=0.5;
            marker.scale.y=0.5;
            marker.scale.z=0.5;

            float _i=i*pi/4+t/7;
            marker.pose.position.x=sin(_i)*3;
            marker.pose.position.y=cos(_i);
            //marker.pose.position.y=0.65;
            //marker.pose.position.z=cos(t/5)*0.15;
            marker.pose.position.z=0;

            marker.color.r=127.5+127.5*sin(_i);
            marker.color.g=127.5+127.5*sin(_i+2*pi/3);
            marker.color.b=127.5+127.5*sin(_i+2*pi/3*2);

            //if(sin(t)>0)
            mArray.markers.push_back(marker);
        }

    
        pub.publish(mArray);
        pubCount++;
        ROS_INFO("pubCount=%d",pubCount);

        ros::spinOnce();
        loop_rate.sleep();
        
        //Check if ROS master is running
        if(!ros::master::check()){
            printf("ROS Master has stopped!\n");
            break;
        }
    }
    return 0;
}

