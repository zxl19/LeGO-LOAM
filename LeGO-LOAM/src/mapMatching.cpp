// todo: 地图匹配
// !测试中
// LeGO-LOAM相关
# include "utility.h"
// 读取点云相关
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// 点云配准相关
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

class MapMatching
{
private:
    ros::NodeHandle nh;

    ros::Subscriber subSegmentedCloud;
    ros::Subscriber subLaserOdometry;

    ros::Publisher pubPoseInfo;

    pcl::PointCloud<PointType>::Ptr segmentedCloud;
    sensor_msgs::PointCloud2 ;
    nav_msgs::Odometry laserOdometry2;

public:
    MapMatching() : nh("~")
    {
        // 订阅地面分割后的点云，调用函数
        subSegmentedCloud = nh.subscribe<sensor_msgs::PointCloud2>("/segmented_cloud", 1, &MapMatching::cloudHandler, this);
        // subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1, &MapMatching::cloudHandler, this);
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 1);

        pubPoseInfo = nh.advertise<nav_msgs::Odometry>("/pose", 5);
        allocateMemory();
    }

    void allocateMemory(){
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
    }

    ~MapMatching() {}

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {
        // 1. Convert ros message to pcl point cloud
        copyPointCloud(laserCloudMsg);
        // 2. NDT Matching
        mapRegistration();
    }

    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {

        cloudHeader = laserCloudMsg->header;
        // cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        // Remove Nan points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
        // have "ring" channel in the cloud
        if (useCloudRing == true)
        {
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudInRing);
            if (laserCloudInRing->is_dense == false)
            {
                ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
                ros::shutdown();
            }
        }
    }

    void mapRegistration()
    {
        // Initializing Normal Distributions Transform (NDT).
        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

        // Setting scale dependent NDT parameters
        // Setting minimum transformation difference for termination condition.
        ndt.setTransformationEpsilon(0.01);
        // Setting maximum step size for More-Thuente line search.
        ndt.setStepSize(0.1);
        //Setting Resolution of NDT grid structure (VoxelGridCovariance).
        ndt.setResolution(1.0);

        // Setting max number of registration iterations.
        ndt.setMaximumIterations(35);

        // Setting point cloud to be aligned.
        ndt.setInputSource(map);
        // Setting point cloud to be aligned to.
        ndt.setInputTarget(target_cloud);
    }
};

// *加载地图
// 加载整个地图，地图应当只加载一次
// todo 动态加载
void loadMap()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(HD_MAP_PATH, *map) == -1) //* load the file
    {
        ROS_ERROR("Couldn't read map file.\n");
        ros::shutdown();
    }
}

// ********以下无需更改********
// *类比LeGO-LOAM的代码组织形式
int main(int argc, char **argv)
{
    ros::init(argc, argv, "lego_loam");

    // 读入点云地图
    loadMap();
    // 在类的构造函数中完成地图匹配
    MapMatching MM;

    ROS_INFO("\033[1;32m---->\033[0m Map Matching Started.");

    ros::spin();
    return 0;
}