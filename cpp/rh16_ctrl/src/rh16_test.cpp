#include "rh16_test.hpp"
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <array>
#include <string>
#include <sched.h>
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>



namespace ruiyan::rh16
{
    rh16_test::rh16_test( std::string name ) : Node( name )
    {
        RCLCPP_INFO(this->get_logger(), "hello %s",name.c_str());

        rh16cmd = rh16_cmd::msg::Rh16Cmd();
        rh16cmd.mode = 1;
        rh16cmd.lr = 0;
        tick = 0;
        tspan_ms = 5;

        for(int i = 0; i < 16; i++)
        {
            rh16cmd.m_pos[i] = 4095;
            rh16cmd.m_spd[i] = 1000;
            rh16cmd.m_curlimit[i] = 100;
        }

        // 创建发布器 - 命令
        ryhand_cmd_publisher_ = this->create_publisher<rh16_cmd::msg::Rh16Cmd>( "ryhand_cmd", 1);
        rclcpp::Publisher<rh16_cmd::msg::Rh16Cmd>::SharedPtr ryhand_cmd_publisher_;
        
        // 创建定时器 tspan_ms ms 
        timer_ = this->create_wall_timer(std::chrono::milliseconds(tspan_ms), std::bind(&rh16_test::PubCmd, this));
    }


    float rh16_test::rad_to_deg(float rad)
    {
        return rad * 180 / M_PI;
    }


    float rh16_test::deg_to_rad(float deg) //
    {
        return deg * M_PI / 180;
    }


    void rh16_test::PubCmd()
    {
        float p1,p2,p3,p4; // p5,p6,j; 
        float fs = sin( 2 * M_PI * tick / 4000  );
        float fc = cos( 2 * M_PI * tick / 4000  );

        switch (rh16cmd.mode)
        {
            // raw cmd
            case 0:
                // 生成幅值为4000 的正弦波，周期为4s
                for(int i = 0; i < 16; i++)
                {
                    #if 0

                        if( i%3 == 2 )
                        {
                            p1 = 1500 + 1500 * fs + 1000;
                            p2 = fabs( 1000 * fc ) + 500;
                            rh16cmd.m_pos[i] = p1;
                            rh16cmd.m_spd[i] = p2;
                        }
                        else
                        {
                            rh16cmd.m_pos[i] = 4000;
                            rh16cmd.m_spd[i] = 1000;
                        }

                    #else

                        p1 = 1500 + 1500  * fs + 1000;
                        p2 = fabs( 1000 * fc ) + 500;
                        rh16cmd.m_pos[i] = p1;
                        rh16cmd.m_spd[i] = p2;

                    #endif
                }
                break;

            // angle cmd
            case 1:
                for( int i = 0; i < 21; i += 4 )
                {
                    p1 = deg_to_rad( 0  + 10 * fs );
                    p2 = deg_to_rad( 20 + 20 * fs );
                    p3 = deg_to_rad( 30 + 30 * fs );
                    p4 = deg_to_rad( 30 + 30 * fs );

                    rh16cmd.j_ang[ i + 0 ] = p1;
                    rh16cmd.j_ang[ i + 1 ] = p2;
                    rh16cmd.j_ang[ i + 2 ] = p3;
                    rh16cmd.j_ang[ i + 3 ] = p4;
                }
                break;

            // end pos cmd    
            case 2:

                // float64 x_base
                // float64 y_base
                // float64 z_base

                // float64 roll_base
                // float64 pitch_base
                // float64 yaw_base

                // float64[5] x
                // float64[5] y
                // float64[5] z

                // float64[5] roll
                // float64[5] pitch
                // float64[5] yaw

                // double end_pos[6] = {msg->end_pos[0], msg->end_pos[1], msg->end_pos[2], msg->end_pos[3], msg->end_pos[4], msg->end_pos[5]};
                // Eigen::Isometry3d transform = solve::Xyzrpy2Isometry(end_pos);
                // interfaces_ptr_->setEndPose(transform);
                // std::vector<double> joint_positions = {0, 0, 0, 0, 0, 0};
        
                // for (int i = 0; i < 6; i++)
                // {
                //     joint_positions[i] = msg->joint_pos[i];
                // }

                break;

            default:
                break;
        }
         
        tick = (tick + tspan_ms);

        // 发布消息
        ryhand_cmd_publisher_->publish(rh16cmd);
    }


}




int main(int argc, char *argv[])
{
    // ROS2 初始化
    rclcpp::init(argc, argv);

    auto node =  std::make_shared<ruiyan::rh16::rh16_test>("rh16_test");
    rclcpp::spin(node);
    rclcpp::shutdown();


    return 0;
}



