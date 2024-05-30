#ifndef BA_INITIALIZABLE_NODE
#define BA_INITIALIZABLE_NODE

#include "manipulator.h"
#include "tomato_queue.h"
#include "path_queue.h"

#pragma region IBAInitManipulatorNode

/**
 * @brief Interface for all classes which depend on the Manipulator
 *
 */
class IBAInitManipulatorNode
{
public:
    IBAInitManipulatorNode(){};
    virtual ~IBAInitManipulatorNode(){};
    virtual void init(Manipulator manipulator) = 0;
};

#pragma endregion

#pragma region IBAInitNodeHandle

/**
 * @brief Interface for all classes which depend on the ros::NodeHandle
 *
 */
class IBAInitNodeHandle
{
public:
    IBAInitNodeHandle(){};
    virtual ~IBAInitNodeHandle(){};
    virtual void init(std::shared_ptr<rclcpp::Node> node_handle) = 0;
};
#pragma endregion

// #pragma region IBAInitTomatoQueue

// /**
//  * @brief Interface for all classes which depend on the TomatoQueue
//  *
//  */
// class IBAInitTomatoQueue
// {
// public:
//     IBAInitTomatoQueue(){};
//     virtual ~IBAInitTomatoQueue(){};
//     virtual void init(TomatoQueue &tomato_queue) = 0;
// };

// #pragma endregion

// #pragma region IBAInitPathQueue

// /**
//  * @brief Interface for all classes which depend on the PathQueue
//  *
//  */
// class IBAInitPathQueue
// {
// public:
//     IBAInitPathQueue(){};
//     virtual ~IBAInitPathQueue(){};
//     virtual void init(PathQueue &path_queue) = 0;
// };

// #pragma endregion

#endif