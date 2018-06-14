//Simple timer class to keep track of elapsed time

class StopWatch{
    double tStart;
    double tElapsed;
    bool EN=0;

    public:
    void init(){
        start();
    }

    void start(){
        EN=true;
        //ros::spinOnce();
        tStart=ros::Time::now().toSec();
        tElapsed=0;
    }

    void stop(){
        //ros::spinOnce();
        tElapsed=ros::Time::now().toSec()-tStart;
        EN=false;
    }
    
    bool started(){
        return EN;
    }
    double elapsed(){
        if(EN){
            //ros::spinOnce();
            tElapsed=ros::Time::now().toSec()-tStart;
            return tElapsed;
        }else{
            return -1;
        }
    }
    double lastTime(){
            return tElapsed;
    }
};