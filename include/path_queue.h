#ifndef PATH_QUEUE_H
#define PATH_QUEUE_H

#pragma region includes

// #include "ros/message.h"
// #include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "ros_logs.h"
#include "ba_types.h"

#pragma endregion

class PathQueue
{
public:
    void Enqueue(QueueSide side, PathLocation path_location);
    PathLocation Dequeue(QueueSide side, bool should_pop = true);

    void SetEmptyBasketLocation(PathLocation location);
    PathLocation GetEmptyBasketLocation();

    void SetRechargeLocation(PathLocation location);
    PathLocation GetRechargeLocation();

    std::list<PathLocation> GetQueue();
private:
    std::list<PathLocation> queue_;
    PathLocation empty_basket_location_;
    PathLocation recharge_location_;
};

#endif