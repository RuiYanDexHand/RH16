#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <array>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp> 
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/Dense>

#include "rh16_cmd/msg/rh16_cmd.hpp"
#include "rh16_msg/msg/rh16_msg.hpp"
#include "rh16_cmd/srv/rh16fk.hpp"
#include "rh16_cmd/srv/rh16ik.hpp"




namespace ruiyan::rh16
{
    class rh16_ctrl : public rclcpp::Node
    {
    public:
        rh16_ctrl( std::string name );

        void CmdCallback(const rh16_cmd::msg::Rh16Cmd::SharedPtr msg);
        void PubState();
        void UpdataMotor( void );

        void rh16fk_callback(const rh16_cmd::srv::Rh16fk::Request::SharedPtr request,const rh16_cmd::srv::Rh16fk::Response::SharedPtr response);
        void rh16ik( pinocchio::Model& model, pinocchio::Data& data, Eigen::VectorXd &q_ik, std::string ftip[5], const rh16_cmd::srv::Rh16ik::Request::SharedPtr request, const rh16_cmd::srv::Rh16ik::Response::SharedPtr response);
        void rh16ik_callback(const rh16_cmd::srv::Rh16ik::Request::SharedPtr request,const rh16_cmd::srv::Rh16ik::Response::SharedPtr response);

    private:

        pinocchio::Model model_fk_,model_ik_,model_,model_l_,model_r_;
        pinocchio::Data data_fk_,data_ik_,data_,data_l_,data_r_;
        Eigen::VectorXd q_fk_,q_ik_, q_iik_, q_;
        std::string urdf_path,urdf_filename_l,urdf_filename_r;
        std::string fingertip_l_[5], fingertip_r_[5];
        std::string fingertip[5];
        std::string fingertip_fk[5];
        std::string fingertip_ik[5];

        double poly_coeff[5][6];
        int si;

        rh16_msg::msg::Rh16Msg rh16msg;
        rh16_cmd::msg::Rh16Cmd rh16cmd;


        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<rh16_msg::msg::Rh16Msg>::SharedPtr ryhand_state_publisher_;
        rclcpp::Subscription<rh16_cmd::msg::Rh16Cmd>::SharedPtr ryhand_cmd_subscriber_;


        // 声明服务回调组
        rclcpp::CallbackGroup::SharedPtr callback_group_service_;

        // 声明服务端
        rclcpp::Service<rh16_cmd::srv::Rh16fk>::SharedPtr server_rh16fk_;
        rclcpp::Service<rh16_cmd::srv::Rh16ik>::SharedPtr server_rh16ik_;


        rh16_cmd::srv::Rh16fk::Request req_fk;
        rh16_cmd::srv::Rh16fk::Response res_fk;

        rh16_cmd::srv::Rh16ik::Request req_ik;
        rh16_cmd::srv::Rh16ik::Response res_ik;


    };
}
