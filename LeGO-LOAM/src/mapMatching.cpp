# include "utility.h"

class MapMatching{
    private:
    public:
    
}
int main(int argc, char** argv){

    ros::init(argc, argv, "lego_loam");

    ImageProjection IP;

    ROS_INFO("\033[1;32m---->\033[0m Map Matching Started.");

    ros::spin();
    return 0;
}