#include "land_base.hpp"
#include "as2_motion_command_handlers/speed_motion.hpp"

namespace land_plugins
{
    class LandSpeed : public land_base::LandBase
    {
    public:
        rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::Land::Goal> goal) override
        {
            desired_speed_ = goal->land_speed;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleLand> goal_handle) override
        {
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        bool onExecute(const std::shared_ptr<GoalHandleLand> goal_handle) override
        {
            rclcpp::Rate loop_rate(10);
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<as2_msgs::action::Land::Feedback>();
            auto result = std::make_shared<as2_msgs::action::Land::Result>();

            static as2::motionCommandsHandlers::SpeedMotion motion_handler(node_ptr_);

            time_ = node_ptr_->now();

            // Check if goal is done
            while (!checkGoalCondition())
            {
                if (goal_handle->is_canceling())
                {
                    result->land_success = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(node_ptr_->get_logger(), "Goal canceled");

                    // TODO: change this to hover
                    motion_handler.sendSpeedCommandWithYawSpeed(0.0, 0.0, 0.0, 0.0);
                    return false;
                }

                motion_handler.sendSpeedCommandWithYawSpeed(0.0, 0.0, desired_speed_, 0.0);

                feedback->actual_land_height = actual_heigth_;
                feedback->actual_land_speed = actual_z_speed_;
                goal_handle->publish_feedback(feedback);

                loop_rate.sleep();
            }

            result->land_success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(node_ptr_->get_logger(), "Goal succeeded");

            // TODO: change this to hover
            motion_handler.sendSpeedCommandWithYawSpeed(0.0, 0.0, 0.0, 0.0);
            return true;
        }

    private:
        bool checkGoalCondition()
        {
            if (fabs(actual_z_speed_) < 0.1)
            {
                if ((node_ptr_->now() - this->time_).seconds() > 1)
                {
                    return true;
                }
                else
                {
                    time_ = node_ptr_->now();
                }
            }
            return false;
        }
    
    private:
        rclcpp::Time time_;

    }; // LandSpeed class
} // land_plugins namespace

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(land_plugins::LandSpeed, land_base::LandBase)
