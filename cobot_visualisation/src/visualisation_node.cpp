#include <stdio.h>
#include <math.h>

#include <ros/ros.h>
#include <ros/master.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection/collision_tools.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <cobot_visualisation/VisualisationData.h>

#include <string>
#include <tf/tf.h>
#include <Eigen/Geometry>

#include "StopWatch.h"

namespace rvt = rviz_visual_tools;
static const std::string THIS_PACKAGE = "cobot_visualisation";
static const std::string PLANNING_GROUP_NAME = "manipulator";  // RRBot Specific
static const std::string PLANNING_SCENE_TOPIC = "/planning_scene";
//static const std::string PLANNING_SCENE_TOPIC = "/move_group/monitored_planning_scene";
static const std::string BASE_FRAME = "/world";

static const std::string TOPIC_SUB_VISUALISATION_DATA = THIS_PACKAGE+"/visualisation_data";
static float pi = 3.14159265359;
static float callback_timeout=2.0; //Time before timing out
static float CALLBACK_TIMEOUT=2.0;

class Visualisation
{
public:


    std::string workcell_path;
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
    bool callback_active=0;
    bool callback_timedout=0;
    float callback_tOld=0;
    float callback_dt=0;

    StopWatch callbackTimer;
    
    //Start moveit visual tools
    void init(){
        workcell_path = "file://" + ros::package::getPath(THIS_PACKAGE);
        if (workcell_path == "file://")
            ROS_FATAL_STREAM_NAMED("visual_tools", "Unable to get " << THIS_PACKAGE << " package path ");
        workcell_path.append("/resources/Workcell_ASCII.STL");

        visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(BASE_FRAME,"/moveit_visual_tools"));
        visual_tools_->setPlanningSceneTopic(PLANNING_SCENE_TOPIC);
        //visual_tools_->loadPlanningSceneMonitor();
        visual_tools_->loadMarkerPub(true);
        visual_tools_->loadRobotStatePub("display_robot_state");
        visual_tools_->setManualSceneUpdating();

        ros::spinOnce();
        ros::Duration(0.1).sleep();

        removeAllObjects();
        triggerPlanningSceneUpdate();
        ros::Duration(0.1).sleep();

        callback_tOld=ros::Time::now().toSec();
        //callbackTimer.init();
    }

    //Publish workcell
    void publishCollisionWorkcell(){
        //Place Workcell:
        geometry_msgs::Pose p;
        p.position.x=0;
        p.position.y=0;
        p.position.z=0;
        tf::Quaternion q= tf::createQuaternionFromRPY(pi/2,0,0);
        p.orientation.x=q.getX();
        p.orientation.y=q.getY();
        p.orientation.z=q.getZ();
        p.orientation.w=q.getW();

        visual_tools_->publishCollisionMesh(p,"Workcell",workcell_path,rvt::GREEN);
    }

    //Remove object with name "name"
    bool removeCollisionObject(std::string name){
        moveit_msgs::CollisionObject collision_obj;
        collision_obj.header.stamp = ros::Time::now();
        collision_obj.header.frame_id = BASE_FRAME;
        collision_obj.id = name;
        collision_obj.operation = moveit_msgs::CollisionObject::REMOVE;

        return visual_tools_->processCollisionObjectMsg(collision_obj,rvt::WHITE);
    }

    //Callback of the object toppic:
    //Pass objects to moveit
    //std::vector<geometry_msgs::Pose> objects;
    void visualisationDataCallback(cobot_visualisation::VisualisationData data){

        if(!callback_active){
            callback_active=1;
            printf("First callback detected! (%0.1f sec)\n",callback_dt);
        }

        int numObjects=data.objects.markers.size();
        printf("numObjects=%d\n",numObjects);

        removeAllObjects();
        //publishCollisionArena();
        publishCollisionWorkcell();
        
        //Draw objects:
        for(int i=0;i<numObjects;i++){
            std::string objName="object"+std::to_string(i);

            geometry_msgs::Vector3 size;
            size.x= data.objects.markers[i].scale.x;
            size.y= data.objects.markers[i].scale.y;
            size.z= data.objects.markers[i].scale.z;

            float r=data.objects.markers[i].color.r;
            float g=data.objects.markers[i].color.g;
            float b=data.objects.markers[i].color.b;
            publishCollisionBox(data.objects.markers[i].pose,size,objName,RgbToRvizColor(r,g,b));
        }

        //Draw line between the closest object to the robot
        rvt::colors lineColor = rvt::WHITE;
        printf("%s\n",data.statusMessage.c_str());
        if(data.statusMessage.compare("start")==0){ // string1.compare(string2) Returns 0 if string1 equals string2
            lineColor=rvt::GREEN;
        }
        if(data.statusMessage.compare("cooldown")==0){
            lineColor=rvt::YELLOW;
        }
        if(data.statusMessage.compare("stop")==0){
            lineColor=rvt::RED;
        }
        drawLine(data.closestFrame,data.closestObject,lineColor);
        drawTwoPointText(data.closestFrame,data.closestObject,std::to_string(data.closestDistance));
        
        callback_tOld=ros::Time::now().toSec();
        callbackTimer.start(); //(re)start callback timer
        triggerPlanningSceneUpdate();
    }

    //Draw text centered between two points
    void drawTwoPointText(geometry_msgs::Point p1, geometry_msgs::Point p2, std::string text){
        geometry_msgs::Pose textPose;
        textPose.position.x=p1.x+(p2.x-p1.x)/2;
        textPose.position.y=p1.y+(p2.y-p1.y)/2;
        textPose.position.z=p1.z+(p2.z-p1.z)/2+0.1;
        visual_tools_->RvizVisualTools::publishText(textPose,text,rvt::WHITE,rvt::XXLARGE);
    }

    //Chech if the callback is started and not timed out.
    bool callbackOK(){
        if(!callbackTimer.started()){
            printf("Waiting for callback! %0.1f\n",callbackTimer.elapsed());
            return 0;
        }
        if(callbackTimer.elapsed()>CALLBACK_TIMEOUT){
            printf("Callback timedOut! (%0.1f sec)\n",callbackTimer.elapsed());
            return 0;
        }
        printf("Callback healthy (%0.1f sec)\n",callbackTimer.elapsed());
        return 1;
    }

    //Draw a sizable box (rotation not supported!):
    void publishCollisionBox(geometry_msgs::Pose pos,geometry_msgs::Vector3 size,std::string name,rvt::colors color){        
        geometry_msgs::Point p1;
        geometry_msgs::Point p2;
        //Set box size:
        p1.x=-size.x/2;
        p1.y=-size.y/2;
        p1.z=-size.z/2;

        p2.x=size.x/2;
        p2.y=size.y/2;
        p2.z=size.z/2;

        //Move box to desired location:
        p1.x+=pos.position.x;
        p1.y+=pos.position.y;
        p1.z+=pos.position.z;

        p2.x+=pos.position.x;
        p2.y+=pos.position.y;
        p2.z+=pos.position.z;

        //Publish box:
        visual_tools_->publishCollisionCuboid(p1,p2,name,color);
    }

    //Publish arena to test robot planning
    //Draws circular floor and two walls
   void publishCollisionArena(){ 
        visual_tools_->deleteAllMarkers();
        visual_tools_->removeAllCollisionObjects();
        
        geometry_msgs::Point pA;
        geometry_msgs::Point pB;
        pB.z=-0.1;
        visual_tools_->publishCollisionCylinder(pA,pB,"Floor1",0.7,rvt::RED);

        //Wall position and size adjustments:
        double wall_rotation=-pi/2;
        double wall_distance=0.5;
        double wall_width=2;
        double wall_height=1;

        //Wall calculations (dont touch!)
        double x = sin(wall_rotation)*wall_distance;
        double y = cos(wall_rotation)*wall_distance;
        double z = 0;
        visual_tools_->publishCollisionWall(x,y,z,wall_rotation+pi/2,wall_width,wall_height,"Wall1",rvt::PINK);

        //Wall position and size adjustments:
         wall_rotation=0;
         wall_distance=0.35;
         wall_width=2;
         wall_height=1;

        //Wall calculations (dont touch!)
         x = sin(wall_rotation)*wall_distance;
         y = cos(wall_rotation)*wall_distance;
         z = 0;
        visual_tools_->publishCollisionWall(x,y,z,wall_rotation+pi/2,wall_width,wall_height,"Wall2",rvt::PINK);
        visual_tools_->triggerPlanningSceneUpdate();
    }

    void removeAllObjects(){
        visual_tools_->deleteAllMarkers();
        visual_tools_->removeAllCollisionObjects();
    }

    void triggerPlanningSceneUpdate(){
        visual_tools_->RvizVisualTools::trigger();
        visual_tools_->triggerPlanningSceneUpdate();
    }

    //Draw a line between two points and add spheres to the ends
    void drawLine(geometry_msgs::Point p1, geometry_msgs::Point p2,rvt::colors color){
        visual_tools_->RvizVisualTools::deleteAllMarkers();
        visual_tools_->RvizVisualTools::publishCylinder(
            visual_tools_->RvizVisualTools::convertPoint(p1),
            visual_tools_->RvizVisualTools::convertPoint(p2),
            color,0.025,"line");
        visual_tools_->RvizVisualTools::publishSphere(p1,color,rvt::XLARGE,"sphere1");
        visual_tools_->RvizVisualTools::publishSphere(p2,color,rvt::XLARGE,"sphere2");
    }

    //Convert RGB to HSV color space
    static void RGB2HSV(float r, float g, float b,     float &h, float &s, float &v) 
    {
        float rgb_max = std::max(r, std::max(g, b));
        float rgb_min = std::min(r, std::min(g, b));
        float delta = rgb_max - rgb_min;
        s = delta / (rgb_max + 1e-20f);
        v = rgb_max;

        float hue;
        if (r == rgb_max)
            hue = (g - b) / (delta + 1e-20f);
        else if (g == rgb_max)
            hue = 2 + (b - r) / (delta + 1e-20f);
        else
            hue = 4 + (r - g) / (delta + 1e-20f);
        if (hue < 0)
            hue += 6.f;
        h = hue * (1.f / 6.f);
    }

    //Convert RGB to a standard color supported by Rviz
    //
    rvt::colors RgbToRvizColor(float r,float g,float b){ 
        float h; float s; float v; int color=13;
        
        RGB2HSV(r,g,b,h,s,v);
        h*=360; s*=255; v*=1;
        //printf("h:%f,s:%f,v:%f\n",h,s,v);

        if(r==g && r==b){ //Gray colors
            //printf("Gray\n");
            if(v>212){ //White
                color=13;
            }else if(v>127){ //Grey
                color=4;
            }else if(v>85){ //Dark grey
                color=5;
            }
        }else if(s>200){ //Saturated colors
            //printf("Saturated colors\n");

            if(h>330 || h<=20){ //Red
                color=11;
            }else if(h>270){ //Magenta
                color=10;
            }else if(h>210){ //Blue
                color=2;
            }else if(h>150){ //Cyan
                color=3;                
            }else if(h>90){ //Green
                color=6;
            }else if(h>50){ //Yellow
                color=14;
            }else if(h>20){ //Orange
                color=9;
            }
        }

        return visual_tools_->RvizVisualTools::intToRvizColor(color);
    }
};




//--------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------
Visualisation vis;
void mainLoop(const ros::TimerEvent&){
    if(!vis.callbackOK()){
        vis.removeAllObjects();
        vis.publishCollisionWorkcell();
        //vis.publishCollisionArena();

        geometry_msgs::Point p1;
        geometry_msgs::Point p2;
        p1.x=1;     p2.x=-1;
        p1.y=0;     p2.y=0;
        p1.z=1;     p2.z=1;

        vis.drawLine(p1,p2,rvt::WHITE);
        vis.drawTwoPointText(p1,p2,"No data!");
        vis.triggerPlanningSceneUpdate();
    }
}

int main(int argc, char **argv){
    ros::init(argc,argv,"visualisation_node");
    ros::NodeHandle n;

    //ros::Duration(5).sleep();
    ROS_INFO("visualisation_node started");
    vis.init();
    ros::Subscriber sub1 = n.subscribe(TOPIC_SUB_VISUALISATION_DATA, 1, &Visualisation::visualisationDataCallback,&vis);
    ros::Timer mainTimer = n.createTimer(ros::Duration(0.33),mainLoop);
    
    ros::spin();
    return 0;
}