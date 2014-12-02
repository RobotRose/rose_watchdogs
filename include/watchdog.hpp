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
#include <ros/ros.h>
#include <string>

#ifndef WATCHDOG_HPP
#define WATCHDOG_HPP

namespace rose
{

class Watchdog
{
  public:
    Watchdog( std::string name, ros::NodeHandle n, double timeout, boost::function<void()> callback, bool oneshot = true, bool autostart = false );
    ~Watchdog();
    
    /**
     * Reset the watchdog. Cancels all pending callbacks and lets the timer forget about the time that has already elapsed.
     */
    void reset();
    
    /**
     * Start the watchdog and its timer. Does nothing if the watchdog is already started. 
     */
    void start();

    /**
     * Stop the watchdog and its timer. Does nothing if the watchdog is already stopped. 
     */
    void stop();

    /**
     * Returns if the timer is running at the moment.
     * @return The timer is running.
     */
    bool is_running();

  private:

    void CB_timer_event( const ros::TimerEvent& event );

    std::string         name_;
    ros::NodeHandle     n_;

    bool                autostart_;
    bool                oneshot_;
    bool                running_;
    ros::Timer          timer_;
    double              timeout_;

    boost::function<void()>  callback_;
};

}

#endif  // WATCHDOG_HPP