#include "tomato_queue.h"

#pragma region TomatoLocationComparer

#pragma region public methods

/**
 * @brief Construct a new Tomato Location Comparer:: Tomato Location Comparer object
 * 
 * @param x The x location from where to compare with the tomatos x coordinate
 * @param y The y location from where to compare with the tomatos y coordinate
 * @param z The z location from where to compare with the tomatos z coordinate
 */
TomatoLocationComparer::TomatoLocationComparer(float x, float y, float z)
{
    x_ = x;
    y_ = y;
    z_ = z;
}

bool TomatoLocationComparer::operator()(const TomatoCoordinates *t1, const TomatoCoordinates *t2)
{
    return CalculateDistance(*t1) < CalculateDistance(*t2);
}

#pragma endregion

#pragma region private methods

/**
 * @brief Calculates the distance between a given point and a certain tomato
 * 
 * @param t The TomatoCoordinates to compare with
 * @return float The distance
 */
float TomatoLocationComparer::CalculateDistance(TomatoCoordinates t)
{
    return pow(x_ - t.x, 2) + pow(y_ - t.y, 2) + pow(z_ - t.z, 2);
}
#pragma endregion

#pragma endregion

#pragma region Tomato_Queue

#pragma region public methods

/**
 * @brief Construct a new Tomato Queue:: Tomato Queue object
 * 
 */
TomatoQueue::TomatoQueue()
{
    ros::param::get("basket_capacity", basket_capacity_);
}

/**
 * @brief Clears the TomatoQueue
 * 
 */
void TomatoQueue::ClearQueue()
{
    queue_.clear();
}

/**
 * @brief Adds a tomato (or the coordinates of it) to the queue
 * 
 * @param tomato_coordinates The TomatoCoordinates
 */
void TomatoQueue::AddTomato(TomatoCoordinates tomato_coordinates)
{
    if (!Exists_Tomato(tomato_coordinates))
    {
        queue_.push_back(tomato_coordinates);
        ROS_INFO("inserted tomato! new length: %ld", queue_.size());
    }
    else
    {
        ROS_INFO("Tomato not enqueued since a similar one already exists!");
    }
}

/**
 * @brief Sorts the queue in terms of distance to the robots base
 * 
 * @param x The x location
 * @param y The y location
 * @param z The z location
 */
void TomatoQueue::SortQueue(float x, float y, float z)
{
    TomatoLocationComparer tlc(x, y, z);
    reachable_tomatoes_.sort(tlc);
}

/**
 * @brief Filters the list due to the distance to the robots base
 * 
 * @param x The x coordinate
 * @param y The y coordinate
 * @param z The z coordinate
 * @return std::list<TomatoCoordinates> 
 */
std::list<TomatoCoordinates> TomatoQueue::filtered(float x, float y, float z)
{
    std::list<TomatoCoordinates> result;
    TomatoCoordinates loc = {x, y, z, 0.0};
    std::copy_if(queue_.begin(), queue_.end(), std::back_inserter(result), [this, &loc](TomatoCoordinates t)
                 { return pow(loc.x - t.x, 2) + pow(loc.y - t.y, 2) + pow(loc.z - t.z, 2) > 3; });
    return result;
}

/**
 * @brief The location from where the the reachable tomatos should be determined
 * 
 * @param x The x coordinate
 * @param y The y coordinate
 * @param z The z coordinate
 * @param armlength The UR_5 arm length or working radius
 * @return int The amount of reachable tomatoes
 */
int TomatoQueue::GetReachableTomatoCount(float x, float y, float z, float armlength)
{
    reachable_tomatoes_.clear();
    for(auto tomato = queue_.begin(); tomato!=queue_.end(); ++tomato){
        if(!tomato->is_picked && pow(tomato->x - x, 2) + pow(tomato->y - y, 2) + pow(tomato->z - z, 2) < pow(armlength+tomato->r, 2)){
            reachable_tomatoes_.push_back(&(*tomato));
        }
    }
    SortQueue(x, y, z);
    return reachable_tomatoes_.size();
}

/**
 * @brief sets the flag "is_picked" in the current tomato and therefore in the queue_
 * 
 */
void TomatoQueue::SetTomatoAsPicked(void){
    current_tomato_->is_picked = true;
}

/**
 * @brief Method to check whether the basket is full or not
 * 
 * @return true If the basket is full
 * @return false If the basket is not full alredy
 */
bool TomatoQueue::IsBasketFull(void)
{
    if (picked_tomatoes_.size() >= basket_capacity_)
    {
        ROS_ERROR("Basket is full: %ld of %d Tomatoes in basket!", picked_tomatoes_.size(), basket_capacity_);
        return true;
    }

    if (picked_tomatoes_.size() < (basket_capacity_ - 2))
    {
        ROS_INFO("Currently %ld of %d tomatoes in Basket.", picked_tomatoes_.size(), basket_capacity_);
    }
    else
    {
        ROS_WARN("Currently %ld of %d tomatoes in Basket.", picked_tomatoes_.size(), basket_capacity_);
    }

    return false;
}

/**
 * @brief Empties the basket 'virtually'; clears the queue with the tomatoes stored in the basket
 * 
 */
void TomatoQueue::EmptyBasket()
{
    picked_tomatoes_.clear();
}

/**
 * @brief Prints the whole tomato queue
 * 
 */
void TomatoQueue::PrintQueue()
{
    int i = 0;
    ROS_INFO("--------------------------------------");
    for (auto t : queue_)
    {
        ROS_INFO("Tomato %d @ %.2f, %.2f, %.2f, radius %.2f cm", i, t.x, t.y, t.z, 100 * t.r);
    }
    ROS_INFO("--------------------------------------");
}

/**
 * @brief Gets a list of all reachable tomatoes
 * 
 * @return std::list<TomatoCoordinates> List containing reachable tomato coordinates
 */
std::list<TomatoCoordinates*> TomatoQueue::GetReachableTomatoes()
{
    return reachable_tomatoes_;
}

/**
 * @brief Gets the next reachable tomato
 * 
 * @return TomatoCoordinates The tomato coordinates
 */
TomatoCoordinates TomatoQueue::GetNextReachableTomato()
{
    TomatoCoordinates *next = reachable_tomatoes_.front();
    reachable_tomatoes_.pop_front();
    current_tomato_ = next;
    return *current_tomato_;
}

/**
 * @brief Adds a picked tomato to the basket queue
 * 
 */
void TomatoQueue::AddTomatoToBasketQueue()
{
    picked_tomatoes_.push_back(*current_tomato_);
}

#pragma endregion


#pragma region private methods

/**
 * @brief Checks if a tomato already exists in the queue
 * 
 * @param tomato_coordinates The coordinates of the tomato to check
 * @return true if the tomato already exists
 * @return false if it is a unknown tomato
 */
bool TomatoQueue::Exists_Tomato(TomatoCoordinates tomato_coordinates)
{
    return GetReachableTomatoCount(tomato_coordinates.x, tomato_coordinates.y, tomato_coordinates.z, 1.1 * tomato_coordinates.r) > 0;
}

#pragma endregion

#pragma endregion