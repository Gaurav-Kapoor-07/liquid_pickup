#ifndef BA_TESTS_H
#define BA_TESTS_H

#pragma region includes

#include "image_processing.h"
#include "tomato_queue.h"
#include "ros/ros.h"

#pragma endregion

/**
 * @brief Little test to test the queue mutations of the tomato queue
 *
 */
class TomatoQueueMutation
{
public:
    TomatoQueueMutation(void);
    void AddRandomTomato(void);
    void init(TomatoQueue &tq_);

private:
    TomatoQueue *tomato_queue_;
};

#endif