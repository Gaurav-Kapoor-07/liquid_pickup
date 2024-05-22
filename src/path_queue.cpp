#include "path_queue.h"

/**
 * @brief Put an element into the queue
 * 
 * @param side The side where it should be put into the queue
 * @param path_location The location to put into the queue
 */
void PathQueue::Enqueue(QueueSide side, PathLocation path_location)
{
    if (side == front) {
        queue_.push_front(path_location);
    } else {
        queue_.push_back(path_location);
    }
}

/**
 * @brief Get and/or pop an element from the queue
 * 
 * @param side The side where it should be put into the queue
 * @param should_pop Boolean to define if it should really be popped or just returned
 * @return PathLocation The element from the queue
 */
PathLocation PathQueue::Dequeue(QueueSide side, bool should_pop) {
    PathLocation location;
    if (side == front) {
        location = queue_.front();
        if (should_pop) 
        {
            queue_.pop_front();
        }
    } else {
        location = queue_.back();
        if (should_pop) 
        {
            queue_.pop_back();
        }
    }
    return location;
}

/**
 * @brief Sets the location to empty the basket
 * 
 * @param location The location
 */
void PathQueue::SetEmptyBasketLocation(PathLocation location){
    empty_basket_location_ = location;
}

/**
 * @brief Gets the location to empty the basket
 * 
 * @return PathLocation The location
 */
PathLocation PathQueue::GetEmptyBasketLocation(){
    return empty_basket_location_;
}

/**
 * @brief Sets the location to recharge the battery
 * 
 * @param location The location
 */
void PathQueue::SetRechargeLocation(PathLocation location){
    recharge_location_ = location;
}

/**
 * @brief Gets the location to recharge the battery
 * 
 * @return PathLocation The location
 */
PathLocation PathQueue::GetRechargeLocation(){
    return recharge_location_;
}

/**
 * @brief Gets the whole PathQueue
 * 
 * @return std::list<PathLocation> The PathQueue
 */
std::list<PathLocation> PathQueue::GetQueue() {
    return queue_;
}
