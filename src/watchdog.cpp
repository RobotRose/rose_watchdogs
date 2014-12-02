/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*   Author: Mathijs de Langen
*   Date  : 2014/12/02
*       - File created.
*
* Description:
*   Watchdog for (manual) control of the robot.
* 
***********************************************************************************/
#include "watchdog.hpp"

namespace rose
{

Watchdog::Watchdog( std::string name, ros::NodeHandle n, double timeout, boost::function<void()> callback, bool oneshot )
{
    name_       = name;
    n_          = n;

    timeout_    = timeout;
    callback_   = callback;
    oneshot_    = oneshot;

    timer_ = n_.createTimer(ros::Duration(timeout_), &Watchdog::CB_timer_event, this, oneshot_);
}

Watchdog::~Watchdog()
{

}

void Watchdog::reset()
{
    stop();
    start();
}

void Watchdog::start()
{
    timer_.start();
}

void Watchdog::stop()
{
    timer_.stop();
}

void Watchdog::CB_timer_event( const ros::TimerEvent& event )
{
    ROS_ERROR_NAMED(name_, "Timeout occured.");
    callback_();
}

}