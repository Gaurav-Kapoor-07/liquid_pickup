#ifndef TOMATO_QUEUE_H
#define TOMATO_QUEUE_H

// #include "ros/message.h"
#include "ros_logs.h"
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp/behavior_tree.h"

#include "ba_types.h"

#pragma region TomatoLocationComparer

/**
 * @brief Class to compare two TomatoLocations
 * 
 */
class TomatoLocationComparer{
    public:
        TomatoLocationComparer(float x, float y, float z);
        bool operator()(const TomatoCoordinates *t1, const TomatoCoordinates *t2);
    private:
        float CalculateDistance(TomatoCoordinates t);
        float x_;
        float y_;
        float z_;
};

#pragma endregion

#pragma region Tomato_Queue

/**
 * @brief The TomatoQueue class whose object is manipulated in several behaviors
 * 
 */
class TomatoQueue
{
public:
    TomatoQueue();
    void AddTomato(TomatoCoordinates tomato_coordinates);
    void SortQueue(float x, float y, float z);
    void ClearQueue();
    void PrintQueue(void);
    std::list<TomatoCoordinates> filtered(float x, float y, float z);
    int GetReachableTomatoCount(float x, float y, float z, float armlength);
    bool IsBasketFull(void);
    void SetTomatoAsPicked(void);
    void AddTomatoToBasketQueue();
    void EmptyBasket(void);
    std::list<TomatoCoordinates*> GetReachableTomatoes(void);
    TomatoCoordinates GetNextReachableTomato(void);
private:
    int basket_capacity_;
    TomatoCoordinates *current_tomato_;
    std::list<TomatoCoordinates> queue_;
    std::list<TomatoCoordinates*> reachable_tomatoes_;
    std::list<TomatoCoordinates> picked_tomatoes_;
    bool Exists_Tomato(TomatoCoordinates t);
};

#pragma endregion

#endif