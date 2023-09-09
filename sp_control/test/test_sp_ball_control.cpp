#include <gtest/gtest.h>

#include <memory>

#include <ros/ros.h>

#include "sp_control/sp_ball_control_node.h"


class SrFingertipAttractorsTest : public ::testing::Test
{
    protected:

        std::unique_ptr<sp_control::SpBallControl> sp_ball_controller;

    void SetUp() override
    {
        sp_ball_controller = std::make_unique<sp_control::SpBallControl>(ros::NodeHandle());
    }

};