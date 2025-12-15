#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>


#include "rh16_cmd/msg/rh16_cmd.hpp"



namespace ruiyan::rh16
{
    class rh16_test : public rclcpp::Node
    {
    public:

        rh16_test( std::string name );

        void PubCmd();

        float rad_to_deg(float rad);
        float deg_to_rad(float deg);

    private:

        int tick;
        int tspan_ms;

        rh16_cmd::msg::Rh16Cmd rh16cmd;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<rh16_cmd::msg::Rh16Cmd>::SharedPtr ryhand_cmd_publisher_;


    };
}