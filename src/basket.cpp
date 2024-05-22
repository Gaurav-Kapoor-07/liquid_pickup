#include "basket.h"

#pragma region BasketCheck

/**
 * @brief Construct a new Basket Check:: Basket Check object
 *
 * @param name The name of the behavior
 * @param config The node configuration
 */
BasketCheck::BasketCheck(const std::string &name, const BT::NodeConfiguration &config) : BT::SyncActionNode(name, config)
{
    ROS_LOG_INIT(this->name().c_str());
}

/**
 * @brief Initializes reference to the TomatoQueue
 *
 * @param tomato_queue Reference to the TomatoQueue
 */
void BasketCheck::init(TomatoQueue &tomato_queue)
{
    tomato_queue_ = &tomato_queue;
}

/**
 * @brief Handles the tick from the behavior tree
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus BasketCheck::tick()
{
    if (tomato_queue_->IsBasketFull() == true)
    {
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList BasketCheck::providedPorts()
{
    return {BT::OutputPort<int>("num_locs")};
}

#pragma endregion

#pragma region BasketChange

/**
 * @brief Construct a new Basket Change:: Basket Change object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */
BasketChange::BasketChange(const std::string &name, const BT::NodeConfiguration &config) : BT::SyncActionNode(name, config)
{
    ROS_LOG_INIT(this->name().c_str());
}

/**
 * @brief Initializes referece to the TomatoQueue
 * 
 * @param tomato_queue Reference to the TomatoQueue
 */
void BasketChange::init(TomatoQueue &tomato_queue)
{
    tomato_queue_ = &tomato_queue;
}

/**
 * @brief Handles the tick from the behavior tree
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus BasketChange::tick()
{
    tomato_queue_->EmptyBasket();
    ROS_INFO("Basked changed");
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList BasketChange::providedPorts()
{
    return {BT::OutputPort<int>("num_locs")};
}

#pragma endregion