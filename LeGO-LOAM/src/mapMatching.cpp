// todo: 地图匹配
// !测试中
# include "utility.h"
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

// 定义相关文件存放路径
#define HD_MAP_PATH "/media/zhangxl/LiDAR-SLAM/map.pcd" // change to local hdmap pcd path
Eigen::Vector3d OFFSET = Eigen::Vector3d(429708.731457, 4414356.870536, 58.895004); // yuanboyuan-map offset

typedef struct MinMax
{
    float min = 1000;
    float max = -1000;
    float hG = -2.35;
    int cls = 0;
    std::vector<int> point_idx;
} MinMax;

void build_2D_cell(pcl::PointCloud<pcl::PointXYZI>::Ptr input,
                   const float *range,
                   std::vector<MinMax> &voxels)
{
    float cell_size = 0.4;
    std::vector<int> grid_size;
    grid_size.resize(3);
    for (int i = 0; i < 3; ++i)
    {
        grid_size[i] = int((range[i + 3] - range[i]) / cell_size + 0.5); // x y z
    }

    voxels.resize(grid_size[0] * grid_size[1]);

    for (int i = 0; i < voxels.size(); ++i)
    {
        voxels[i].min = 1000;
        voxels[i].max = -1000;
        voxels[i].point_idx.resize(0);
        voxels[i].cls = 0;
        voxels[i].hG = 0;
    }

    for (int i = 0; i < input->size(); ++i)
    {
        pcl::PointXYZI pt = input->points[i];
        int c_x = int((pt.x - range[0]) / cell_size + 0.5);
        int c_y = int((pt.y - range[1]) / cell_size + 0.5);
        int c_z = int((pt.z - range[2]) / cell_size + 0.5);
        if (c_x >= 0 && c_x < grid_size[0] && c_y >= 0 && c_y < grid_size[1])
        {
            voxels[c_x * grid_size[1] + c_y].point_idx.push_back(i);
            if (voxels[c_x * grid_size[1] + c_y].min > pt.z)
            {
                voxels[c_x * grid_size[1] + c_y].min = pt.z;
            }
            if (voxels[c_x * grid_size[1] + c_y].max < pt.z)
            {
                voxels[c_x * grid_size[1] + c_y].max = pt.z;
            }
        }
    }

    for (int i = 0; i < grid_size[1]; ++i)
    {
        for (int j = 0; j < grid_size[0]; ++j)
        {
            if (i == 0)
            {
                voxels[j * grid_size[1] + i].hG = -2.35;
                continue;
            }
            MinMax &v = voxels[j * grid_size[1] + i];
            float hG = voxels[j * grid_size[1] + i - 1].hG;
            if (j >= 1)
                if (hG > voxels[(j - 1) * grid_size[1] + i - 1].hG)
                    hG = voxels[(j - 1) * grid_size[1] + i - 1].hG;
            if (j < grid_size[0] - 1)
                if (hG > voxels[(j + 1) * grid_size[1] + i - 1].hG)
                    hG = voxels[(j + 1) * grid_size[1] + i - 1].hG;
            if (fabs(v.max - v.min) < 0.15 && v.min > (hG - 0.15) && v.point_idx.size() > 0)
            {
                v.hG = v.min;
                v.cls = 1;
            }
            else
            {
                v.hG = hG;
                v.cls = 0;
            }
        }
    }
}

/* filter the road plane */
void PointCloudSegment(pcl::PointCloud<pcl::PointXYZI>::Ptr input,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr plane,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr object)
{
    // std::vector<pcl::PointIndices>& cluster_indices, float distance_thresh) {
    std::vector<MinMax> voxels;
    float range[6] = {-70, -70, -3, 70, 70, 0};
    build_2D_cell(input, range, voxels);
    pcl::PointIndices::Ptr candidate_inliers(new pcl::PointIndices());
    ;
    for (auto &voxel : voxels)
    {
        if (voxel.cls)
        {
            for (int i = 0; i < voxel.point_idx.size(); ++i)
                candidate_inliers->indices.push_back(voxel.point_idx[i]);
        }
        else
        {
            for (int i = 0; i < voxel.point_idx.size(); ++i)
                if (fabs(input->points[voxel.point_idx[i]].y - voxel.hG) < 0.05)
                    candidate_inliers->indices.push_back(voxel.point_idx[i]);
        }
    }

    int num = 0;
    for (auto &voxel : voxels)
    {
        num = num + voxel.point_idx.size();
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr candidate(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    std::cout << num << " " << input->size() << " " << candidate_inliers->indices.size() << std::endl;
    extract.setInputCloud(input);
    extract.setIndices(candidate_inliers);
    extract.setNegative(false);
    extract.filter(*plane);

    extract.setNegative(true);
    extract.filter(*object);

    // Euclidean Clustering
    // pcl::PointCloud<pcl::PointXYZI>::Ptr object_bv (new pcl::PointCloud<pcl::PointXYZI>);
    // for (int i = 0; i < object->points.size(); ++i) {
    //     pcl::PointXYZI p;
    //     p = object->points[i];
    //     p.y = 0;
    //     object_bv->push_back(p);
    // }
    // pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    // tree->setInputCloud (object_bv);

    // pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    // ec.setClusterTolerance (distance_thresh); // 2cm
    // ec.setMinClusterSize (5);
    // ec.setMaxClusterSize (5000);
    // ec.setSearchMethod (tree);
    // ec.setInputCloud (object_bv);
    // ec.extract (cluster_indices);
}

void loadMapPose()
{
    std::cout << "Loading Map Pose..." << std::endl;
    std::fstream gt_poses_file;
    std::string line;
    gt_poses_file.open(MAP_POSE_FILE);
    std::cout << MAP_POSE_FILE << std::endl;
    while (getline(gt_poses_file, line))
    {
        std::stringstream ss(line);
        double qw, qx, qy, qz, tx, ty, tz;
        std::string timestamp;
        ss >> qw >> qx >> qy >> qz >> tx >> ty >> tz >> timestamp;
        Eigen::Quaterniond q(qw, qx, qy, qz);
        Eigen::Matrix3d R = q.toRotationMatrix();
        Eigen::Vector3d t = Eigen::Vector3d{tx, ty, tz};
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block(0, 0, 3, 3) = R;
        T.block(0, 3, 3, 1) = t;
        map_poses.push_back(T);
    }
}

void loadList()
{
    std::cout << "Loading PCD list..." << std::endl;
    std::fstream gt_poses_file;
    std::string line;
    gt_poses_file.open(PCD_LIST);
    std::cout << PCD_LIST << std::endl;
    while (getline(gt_poses_file, line))
    {
        filenames.push_back(line);
    }
}

void loadVOPose()
{
    std::cout << "Loading VO Pose..." << std::endl;
    std::fstream gt_poses_file;
    std::string line;
    gt_poses_file.open(VO_POSE_FILE);
    std::cout << VO_POSE_FILE << std::endl;
    while (getline(gt_poses_file, line))
    {
        std::stringstream ss(line);
        double qw, qx, qy, qz, tx, ty, tz;
        std::string timestamp;
        ss >> timestamp;
        double p[12];
        for (int i = 0; i < 12; ++i)
            ss >> p[i];
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        R << p[0], p[1], p[2], p[4], p[5], p[6], p[8], p[9], p[10];
        t << p[3], p[7], p[11];
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block(0, 0, 3, 3) = R;
        T.block(0, 3, 3, 1) = t;
        vo_poses.push_back(T);
    }
}
// 主函数
int main(int argc, char **argv)
{
    int frame_id = 0;
    // 输入一个大于800的数字
    if (argc == 2)
        frame_id = atoi(argv[1]);

    loadList();
    loadVOPose();
    loadMapPose();

    pcl::PointCloud<pcl::PointXYZI>::Ptr mapI(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr vector_map(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_map(new pcl::PointCloud<pcl::PointXYZ>);

    /* Load HD Map */
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(HD_MAP_PATH, *mapI) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    for (size_t i = 0; i < mapI->points.size(); ++i)
    {
        pcl::PointXYZ p;
        p.x = mapI->points[i].x + 430000.00 - 429708.731457;
        p.y = mapI->points[i].y + 4415000.00 - 4414356.870536;
        p.z = mapI->points[i].z - 58.895004;
        map->points.push_back(p);
    }

    /* prepare for NDT */
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud(map);
    approximate_voxel_filter.filter(*filtered_map);
    //初始化正态分布变换（NDT）
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    //设置依赖尺度NDT参数
    //为终止条件设置最小转换差异
    ndt.setTransformationEpsilon(0.01);
    //为More-Thuente线搜索设置最大步长
    ndt.setStepSize(0.1);
    //设置NDT网格结构的分辨率（VoxelGridCovariance）
    ndt.setResolution(1.0);
    //设置匹配迭代的最大次数
    ndt.setMaximumIterations(35);
    // 设置要配准的点云
    ndt.setInputTarget(filtered_map);
    //设置点云配准目标

    int start = frame_id;
    Eigen::Matrix4d LastTwc = vo_poses[start - 1];
    // 输出相对轨迹
    ofstream lidar_pose;
    lidar_pose.open("trajectory_lidar_v2.txt");

    Eigen::Matrix4d velocity = vo_poses[start - 1] * vo_poses[start - 2].inverse();

    for (int id = start; id < 1300; ++id)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudI(new pcl::PointCloud<pcl::PointXYZI>); // 创建点云（指针）
        pcl::PointCloud<pcl::PointXYZI>::Ptr planeI(new pcl::PointCloud<pcl::PointXYZI>); // 创建点云（指针）
        pcl::PointCloud<pcl::PointXYZI>::Ptr object(new pcl::PointCloud<pcl::PointXYZI>); // 创建点云（指针）
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(PCD_PATH + filenames[id] + ".pcd", *cloudI) == -1)
        {
            PCL_ERROR("Couldn't read file test_pcd.pcd \n"); //文件不存在时，返回错误，终止程序。
            return (-1);
        }

        PointCloudSegment(cloudI, planeI, object);

        for (size_t i = 0; i < object->points.size(); ++i)
        {
            pcl::PointXYZ p;
            p.x = object->points[i].x;
            p.y = object->points[i].y;
            p.z = object->points[i].z;
            cloud->points.push_back(p);
        }

        for (size_t i = 0; i < planeI->points.size(); ++i)
        {
            pcl::PointXYZ p;
            p.x = planeI->points[i].x;
            p.y = planeI->points[i].y;
            p.z = planeI->points[i].z;
            plane->points.push_back(p);
        }
        // LOAM输出的是相机坐标系（/camera_init），需要转换
        Eigen::Matrix3d lidar2CamR;
        Eigen::Vector3d lidar2CamT;
        lidar2CamR << -0.026526, 0.999625, -0.0067377,
            -0.021887, -0.007319, -0.9997336,
            -0.999408, -0.026372, 0.0220739;
        lidar2CamT << 0.35, 0.0121, -0.58259; // lidar coorindates in camera coorindates
        Eigen::Matrix4d Tcl = Eigen::Matrix4d::Identity();
        Tcl.block(0, 0, 3, 3) = lidar2CamR;
        Tcl.block(0, 3, 3, 1) = lidar2CamT;
        // Tcl = Tcl.inverse();

        Eigen::Matrix3d world2camR;
        Eigen::Vector3d world2camT;
        world2camR << 0.477468, -0.878644, -0.00301306,
            0.0383549, 0.0242683, -0.998969,
            0.877812, 0.47686, 0.0452876;
        world2camT << 0.174803, 1.07101, -4.92232;
        Eigen::Matrix4d Twc;

        Twc = velocity * LastTwc;

        std::cout << Twc << " " << cloud->points.size() << std::endl;

        // Eigen::Matrix4d::Identity();
        // Twc.block(0,0,3,3) = world2camR;
        // Twc.block(0,3,3,1) = world2camT;
        Twc = Twc.inverse();

        Eigen::Matrix4d Twl = LastTwc.inverse() * Tcl;
        Twl = Twl * map_poses[id - 1].inverse() * map_poses[id];

        // pcl::transformPointCloud(*cloud, *cloud, Twl);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> fieldColor(cloud, 255.0, 255.0, 255.0);
        // viewer->addPointCloud(cloud, fieldColor, "cloud");

        //creates an instance of an IterativeClosestPoint and gives it some useful information
        // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        // icp.setInputCloud(cloud);
        // icp.setInputTarget(map);
        // icp.setMaxCorrespondenceDistance(3);
        // icp.setTransformationEpsilon(1e-10);
        // icp.setEuclideanFitnessEpsilon(0.001);
        // icp.setMaximumIterations(100);

        ndt.setInputCloud(cloud);

        //Creates a pcl::PointCloud<pcl::PointXYZ> to which the IterativeClosestPoint can save the resultant cloud after applying the algorithm
        pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);

        //Call the registration algorithm which estimates the transformation and returns the transformed source (input) as output.
        // icp.align(*Final, Twl.cast<float>());
        ndt.align(*Final, Twl.cast<float>());

        //Return the state of convergence after the last align run.
        //If the two PointClouds align correctly then icp.hasConverged() = 1 (true).
        std::cout << "has converged: " << ndt.hasConverged() << std::endl;

        //Obtain the Euclidean fitness score (e.g., sum of squared distances from the source to the target)
        std::cout << "score: " << ndt.getFitnessScore() << std::endl;
        std::cout << "----------------------------------------------------------" << std::endl;

        //Get the final transformation matrix estimated by the registration method.
        std::cout << ndt.getFinalTransformation().cast<double>() << std::endl;

        Eigen::Matrix4f first_Twl = ndt.getFinalTransformation();
        *cloud += *plane;
        std::cout << plane->size() << " " << cloud->size() << std::endl;
        // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp2;
        ndt.setInputCloud(cloud);
        // icp2.setInputTarget(map);
        // icp2.setMaxCorrespondenceDistance(2);
        // icp2.setTransformationEpsilon(1e-10);
        // icp2.setEuclideanFitnessEpsilon(0.001);
        // icp2.setMaximumIterations(100);

        ndt.align(*Final, first_Twl);

        Eigen::Matrix4d pose_now = ndt.getFinalTransformation().cast<double>() * Tcl.inverse();
        std::cout << "----------------------------------------------------------" << std::endl;
        velocity = pose_now.inverse() * LastTwc.inverse();
        std::cout << "Velocity: " << velocity << std::endl;
        LastTwc = pose_now.inverse();

        /* visualization */
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> mapColor(map, 0.0, 255.0, 255.0);
        viewer->addPointCloud(map, mapColor, "map");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> finalColor(cloud, 255.0, 0.0, 255.0);
        viewer->addPointCloud(Final, finalColor, "final" + std::to_string(id));
        int time_id = 60;
        while (!(viewer->wasStopped()) && (time_id > 0))
        {
            viewer->spinOnce(100);
            // time_id--;
        }
        viewer->close();

        Eigen::Matrix4d p = LastTwc;
        lidar_pose << filenames[id] << " "
                   << p(0, 0) << " " << p(0, 1) << " " << p(0, 2) << " " << p(0, 3) << " "
                   << p(1, 0) << " " << p(1, 1) << " " << p(1, 2) << " " << p(1, 3) << " "
                   << p(2, 0) << " " << p(2, 1) << " " << p(2, 2) << " " << p(2, 3) << std::endl;
    }
    lidar_pose.close();

    return (0);
}

class MapMatching
{
private:
public:
    MapMatching() : nh("~")
    {
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1, &MapMatching::cloudHandler, this);

        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_projected", 1);
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_info", 1);

        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 1);
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud", 1);
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud_pure", 1);
        pubSegmentedCloudInfo = nh.advertise<cloud_msgs::cloud_info>("/segmented_cloud_info", 1);
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2>("/outlier_cloud", 1);

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        allocateMemory();
    }
    ~MapMatching(){}

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {
        // 1. Convert ros message to pcl point cloud
        copyPointCloud(laserCloudMsg);
        // 2. Start and end angle of a scan
        findStartEndAngle();
        // 3. Range image projection
        projectPointCloud();
        // 4. Mark ground points
        groundRemoval();
        // 5. Point cloud segmentation
        cloudSegmentation();
        // 6. Publish all clouds
        publishCloud();
        // 7. Reset parameters for next iteration
        resetParameters();
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
};

int main(int argc, char** argv){

    ros::init(argc, argv, "lego_loam");

    MapMatching MM;

    ROS_INFO("\033[1;32m---->\033[0m Map Matching Started.");

    ros::spin();
    return 0;
}