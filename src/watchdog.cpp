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
#include "rose_watchdogs/watchdog.hpp"

namespace rose
{

Watchdog::Watchdog( std::string name, double timeout, boost::function<void()> callback, bool oneshot, bool autostart )
    : running_(false)
    , n_(ros::NodeHandle())
    , name_(name)
{
    timeout_    = timeout;
    callback_   = callback;
    oneshot_    = oneshot;
    autostart_  = autostart; 
    timer_      = n_.createTimer(ros::Duration(timeout_), &Watchdog::CB_timer_event, this, oneshot_, autostart_);
}

Watchdog::Watchdog( std::string name, boost::function<void()> callback, bool oneshot, bool autostart )
    : running_(false)
    , n_(ros::NodeHandle())
    , name_(name)
{
    // Get a private nodehandle to load the configurable parameters
    ros::NodeHandle pn = ros::NodeHandle("~");

    // Retrieve the timeout parameter
    std::string timeout_name = ros::names::append(name_, "timeout");
    ROS_ASSERT_MSG(pn.getParam(timeout_name, timeout_), "Parameter %s must be specified.", timeout_name.c_str());

    callback_   = callback;
    oneshot_    = oneshot;
    autostart_  = autostart; 
    timer_      = n_.createTimer(ros::Duration(timeout_), &Watchdog::CB_timer_event, this, oneshot_, autostart_);
}

Watchdog::~Watchdog()
{

}

void Watchdog::reset()
{
    stop();
    start();
    running_ = true;
}

void Watchdog::start()
{
    timer_.start();
    running_ = true;

}

void Watchdog::stop()
{
    running_ = false;
    timer_.stop();
}

bool Watchdog::is_running()
{
    return running_;
}

void Watchdog::CB_timer_event( const ros::TimerEvent& event )
{
    ROS_ERROR_NAMED(name_, "Timeout occured.");
    callback_();
}

}