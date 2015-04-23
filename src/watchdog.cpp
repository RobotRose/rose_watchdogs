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
    // Get a private NodeHandle
    ros::NodeHandle pn("~");

    // Retrieve the timeout parameter
    std::string timeout_name = ros::names::append(name_, "timeout");
    ROS_ASSERT_MSG(pn.getParam(timeout_name, timeout_), "Watchdog timeout parameter '%s' must be specified. Or a timeout value should be provided in the constructor.", timeout_name.c_str());

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

bool Watchdog::reset( const std_msgs::Header& header )
{
    // Only reset the watchdog when a message is not too old
    if ( ros::Time::now() - ros::Duration(timeout_) < header.stamp )
    {
        reset();
        return true;
    }
    else
    {
        ROS_ERROR("Received message is too old. Difference between received message and now: %fs (timeout = %fs).", (ros::Time::now() - header.stamp).toSec(), timeout_ );
        return false;
    }
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
