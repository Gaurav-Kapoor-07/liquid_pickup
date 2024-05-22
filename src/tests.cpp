#include "tests.h"
#define BLOB_MIN_PX 800
#define BLOB_MAX_PX 4000


std::string img_path;

/**
 * @brief Construct a new Tomato Queue Mutation:: Tomato Queue Mutation object
 *
 */
TomatoQueueMutation::TomatoQueueMutation(){

};

/**
 * @brief Initializes the TomatoQueue reference
 *
 * @param tomato_queue The reference to the TomatoQueue
 */
void TomatoQueueMutation::init(TomatoQueue &tomato_queue)
{
    tomato_queue_ = &tomato_queue;
};

int CheckPerceptionPipelineWithRealFootage(std::string filepath)
{
    cv::SimpleBlobDetector::Params params;
    params.minArea = BLOB_MIN_PX;
    params.maxArea = BLOB_MAX_PX;
    params.filterByArea = true;
    params.filterByColor = false;
    params.filterByInertia = false;
    params.filterByConvexity = false;
    params.thresholdStep = 50;
    cv::Mat img = cv::imread(img_path+filepath, cv::IMREAD_COLOR);
    cv::Mat img_hsv, img_threshold, img_keypoints;
    cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);
    image_processing::PerceptFilterThreshold("red", img_hsv, img_threshold);
    std::vector<cv::KeyPoint> keypoints = image_processing::PerceptBlobDetection(img, img_threshold, img_keypoints, params);
    cv::namedWindow("Image");
    cv::imshow("Image", img_keypoints);
    cv::waitKey(3000);
    cv::namedWindow("thresh");
    cv::imshow("thresh", img_threshold);
    cv::waitKey(3000);
    cv::destroyAllWindows();
    cv::imwrite(img_path+"/testresult"+filepath,img_keypoints);
    return keypoints.size();
}

void CheckImages(void){
    char filename[100];
    ROS_INFO("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    ROS_INFO("Testing perception with real footage");
    ROS_INFO("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    for(int i = 1; i <=6; i++){
        sprintf(filename, "summit_xl_%d.png", i);
        int keypoints_found = CheckPerceptionPipelineWithRealFootage(std::string(filename));
        ROS_INFO_COND(keypoints_found==2, "All 'tomatos' found in %s", filename);
        ROS_ERROR_COND(keypoints_found!=2, "Found %d tomatoes instead of 2 in image %s", keypoints_found, filename);
    }
    ROS_INFO("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
}

void TestTomatoQueue(void){
    ROS_INFO("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    ROS_INFO("Testing tomato queue");
    ROS_INFO("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    TomatoQueue tq;
    ROS_INFO("Should add tomato @ 0 0 0 with r=4cm");
    tq.AddTomato({0,0,0,0.04});         // shoud add
    ROS_INFO("Should fail to add tomato @ 0 0 0 with r=4cm");
    tq.AddTomato({0,0,0,0.04});         // shoud NOT add since alread exists
    ROS_INFO("Should fail to add tomato @ 0 0 0.02 with r=4cm");
    tq.AddTomato({0,0,0.02,0.04});      // shoud NOT add since (similar) alread exists
    ROS_INFO("Should add tomato @ 1 0 0 with r=4cm");
    tq.AddTomato({1,0,0,0.04});         // shoud add
    ROS_INFO("Should add tomato -1 0 0 0 with r=4cm");
    tq.AddTomato({-1,0,0,0.04});        // shoud add
    ROS_INFO_COND(tq.GetReachableTomatoCount(0,0,0,100.0)==3, "Queue successfully filled");
    ROS_ERROR_COND(tq.GetReachableTomatoCount(0,0,0,100.0)!=3, "Something went wrong while filling queue --> overlapping tomatoes added!");
    ROS_INFO_COND(tq.GetReachableTomatoCount(1,0,0,1.1)==2, "Queue successfully filtered for reachable tomatoes");
    ROS_ERROR_COND(tq.GetReachableTomatoCount(1,0,0,1.1)!=2, "Something went wrong while filtering the queue for reacheable tomatoes!");
    ROS_INFO("FINAL QUEUE:");
    tq.PrintQueue();
    ROS_INFO("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "bt_test");
    ros::NodeHandle nh;
    
    ros::param::get("real_footage", img_path);
    TestTomatoQueue();
    CheckImages();

    std::cout << '\n'
              << "Press a key to continue...";
    do
    {
    } while (std::cin.get() != '\n');
}
