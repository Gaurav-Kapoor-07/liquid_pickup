#include "pointcloud_processing.h"

#pragma region public

/**
 * @brief Fits a sphere model into a pointcloud by croping the pointcloud around the region of interest (roi), segregating the surface plane and fitting a ransac sphere into the remaining points.
 * 
 * @param input Pointcloud containing sphere-like objects
 * @param roi region of interest --> where to search for shpere in the pointcloud 
 * @param reset_rviz_cloud flag to clear pointcloud visualized in rviz (topic /summit_xl/tomato_clouds)
 * @return TomatoCoordinates 
 */
TomatoCoordinates pointcloud_processing::FitSphereInCloud(const sensor_msgs::PointCloud2 &input, pcl::PointXYZ roi, bool reset_rviz_cloud)
{
    std::vector<float> coordinates;
    pcl::PCLPointCloud2 pcl_pc2;
    BAPointCloudPtr cloud(new BAPointCloud);
    BAPointCloudPtr cloud_pass(new BAPointCloud);
    BAPointCloudPtr cloud_filtered(new BAPointCloud);

    pcl_conversions::toPCL(input, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    pointcloud_processing::CropPointCloud(cloud, cloud_pass, roi);
    pointcloud_processing::SegregatePlane(cloud_pass, cloud_filtered);
    TomatoCoordinates tomato = pointcloud_processing::FitSphere(cloud_filtered);

#ifdef BA_PCL_DEBUG
    ROS_INFO("%s", "Model coefficients are: ");
    ROS_INFO("%s %.3f", "\tcenter.x is: ", tomato.x);
    ROS_INFO("%s %.3f", "\tcenter.y is: ", tomato.y);
    ROS_INFO("%s %.3f", "\tcenter.z is: ", tomato.z);
    ROS_INFO("%s %.3f m", "\tsphere radius is: ", tomato.r);

    static BAPointCloudPtr tomatoscene(new BAPointCloud);
    if (reset_rviz_cloud)
    {
            *tomatoscene = *cloud_filtered;
    }
    else
    {
        *tomatoscene += *cloud_filtered;
    }

    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(*tomatoscene, out_msg);
    out_msg.header = input.header;
    ros::Publisher tomato_publisher = ros::NodeHandle().advertise<sensor_msgs::PointCloud2>("tomato_clouds", 100, true);
    tomato_publisher.publish(out_msg);
    ros::Rate loop_rate(100);
    ROS_INFO("%s", "sending...");
    for (int i = 0; i < 20; i++)
    {
        tomato_publisher.publish(out_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
#endif

    for (int i = 0; i < 4; i++)
    {
        coordinates.push_back(0);
    }
    return tomato;
}

/**
 * @brief Crops the point cloud information
 * 
 * @param input The input BAPointCloudPtr
 * @param output The output BAPointCloudPtr
 * @param roi The region of interest
 */
void pointcloud_processing::CropPointCloud(BAPointCloudPtr input, BAPointCloudPtr output, pcl::PointXYZ roi)
{
    BAPointCloudPtr helper(new BAPointCloud);
    pcl::PassThrough<pcl::PointXYZ> pass;

    // crop depth
    pass.setInputCloud(input);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(roi.z - BOX_SIZE / 2, roi.z + BOX_SIZE / 2);
    pass.filter(*output);
    // crop vertically
    pass.setInputCloud(output);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(roi.y - BOX_SIZE / 2, roi.y + BOX_SIZE / 2);
    pass.filter(*helper);
    // crop horizontally
    pass.setInputCloud(helper);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(roi.x - BOX_SIZE / 2, roi.x + BOX_SIZE / 2);
    pass.filter(*output);
}

/**
 * @brief Segregates the plane
 * 
 * @param input The input BAPointCloudPtr
 * @param output The output BAPointCloudPtr
 */
void pointcloud_processing::SegregatePlane(BAPointCloudPtr input, BAPointCloudPtr output)
{
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.003);
    seg.setInputCloud(input);
    seg.segment(*inliers, *coeffs);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*output);
}

/**
 * @brief Fits a sphere with ransac algorithm into the pointcloud (input)
 * 
 * @param input The input BAPointCloudPtr
 * @return TomatoCoordinates The tomato coordinates
 */
TomatoCoordinates pointcloud_processing::FitSphere(BAPointCloudPtr input)
{
    std::vector<int> inliers;
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(input));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_s);
    ransac.setDistanceThreshold(0.01);
    ransac.computeModel();
    ransac.getInliers(inliers);

    Eigen::VectorXf coeffs;
    ransac.getModelCoefficients(coeffs);
    TomatoCoordinates result = {coeffs[0], coeffs[1], coeffs[2], coeffs[3], false};
    return result;
}

#pragma endregion