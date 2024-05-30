#ifndef BASKET_H
#define BASKET_H

#pragma region includes

// #include "ros/message.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros_logs.h"
#include "ba_interfaces.h"
#include "ba_types.h"

#pragma endregion

#pragma region BasketCheck

/**
 * @brief Class/Behavior to check whether the basket is full and has to be emptied or not
 * 
 */
class BasketCheck : public BT::SyncActionNode, public IBAInitTomatoQueue
{
public:
    BasketCheck(const std::string &name, const BT::NodeConfiguration &config);
    void init(TomatoQueue &tomato_queue) override;
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    TomatoQueue *tomato_queue_;
};

#pragma endregion

#pragma region BasketChange

/**
 * @brief Class/Behavior to change the basket when it's full
 * 
 */
class BasketChange : public BT::SyncActionNode, public IBAInitTomatoQueue
{
public:
    BasketChange(const std::string &name, const BT::NodeConfiguration &config);
    void init(TomatoQueue &tomato_queue) override;
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    TomatoQueue *tomato_queue_;
};

#pragma endregion

#endif