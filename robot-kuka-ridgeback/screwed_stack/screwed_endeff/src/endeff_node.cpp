#include <ros/ros.h>
#include <fstream>
#include <screwed_msgs/EndEffector.h>
#include <chrono>
#include <thread>


class EndEffNode
{
    public:
        EndEffNode(ros::NodeHandle& nh) : nh_(nh)
        {

            // Subscribers
            endeff_sub_ = nh.subscribe("/screwed_stack/command/end_effector", 10, &EndEffNode::endeffCB, this);

            // Start a watchdog timer thread
            watchdog_thread_ = std::thread(&EndEffNode::watchdogTimer, this);

            ROS_INFO("EndEffNode initialized.");
        }
        
        ~EndEffNode()
        {
            // Disable PWM before shutting down
            disablePWM();

            // Stop the watchdog thread
            stop_watchdog_ = true;
            if (watchdog_thread_.joinable())
            {
                watchdog_thread_.join();
            }

            ROS_INFO("EndEffNode shutting down.");
        }

    private:


        void endeffCB(const screwed_msgs::EndEffector::ConstPtr& msg)
        {
            // Process end effector command
            run_ = msg->run;
            pwm_ = msg->pwm;

            if (run_)
            {
                // Reset the last callback time
                last_callback_time_ = ros::Time::now();

                // Set period to 1ms (1kHz frequency)
                std::ofstream period_file("/sys/class/pwm/pwmchip0/pwm0/period");
                period_file << "1000000";
                period_file.close();

                // Set duty cycle based on the message (scale to nanoseconds)
                std::ofstream duty_cycle_file("/sys/class/pwm/pwmchip0/pwm0/duty_cycle");
                duty_cycle_file << pwm_ * 10000; // pwm_ is a percentage (0-100)
                duty_cycle_file.close();

                // Enable PWM
                std::ofstream enable_file("/sys/class/pwm/pwmchip0/pwm0/enable");
                enable_file << "1";
                enable_file.close();

                ROS_INFO("PWM enabled on GPIO26 with duty cycle: %d%%", pwm_);
            }
        }

        void disablePWM()
        {
            // Disable PWM
            std::ofstream enable_file("/sys/class/pwm/pwmchip0/pwm0/enable");
            enable_file << "0";
            enable_file.close();

            ROS_INFO("PWM disabled on GPIO26 due to timeout.");
        }

        void watchdogTimer()
        {
            ros::Rate rate(10); // Check every 100ms
            while (ros::ok() && !stop_watchdog_)
            {
                if (run_ && (ros::Time::now() - last_callback_time_).toSec() > timeout_)
                {
                    // Disable PWM if the callback hasn't been called within the timeout period
                    disablePWM();
                    run_ = false; // Reset the run flag
                }
                rate.sleep();
            }
        }

        ros::NodeHandle nh_;
        ros::Subscriber         endeff_sub_;
        std::thread             watchdog_thread_;
        bool                    stop_watchdog_ = false;

        bool                    run_ = false;
        int                     pwm_ = 0;
        ros::Time               last_callback_time_ = ros::Time::now();
        const double            timeout_ = 1.0; // Timeout in seconds
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "endeff_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  EndEffNode server(nh);
  ros::waitForShutdown();
  return 0;
};