/**
 * @file main.cpp
 * @author amparo.sancho-arellano (amparo.sancho-arellano@siemens.com)
 * @brief Main entry point for the AsyncAPI ROS 2 Specification Generator ROS 2 node
 * @version 0.1.0
 * @date 2025-05-21
 *
 * Copyright (c) Siemens AG 2024 All Rights Reserved.
 *
 */

#include "ros2_introspector/introspection_helper.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv); // Initialize ROS2 client library

    utils::Logger logger("main_logger");
    utils::ReturnValue return_value;
    auto introspection_helper = std::make_shared<IntrospectionHelper>();

    // Determine the operating mode from the ros 2 parameter
    introspection_helper->getData().operating_mode = introspection_helper->get_parameter("mode").as_string();
    LOG_INFO(logger, "Operating in {} mode", introspection_helper->getData().operating_mode);

    return_value = introspection_helper->process_interfaces_based_on_mode();
    if (!return_value.success)
    {
        LOG_ERROR(logger, "{}", return_value.message);
        if (return_value.return_code == utils::ReturnCode::CLI_USER_INPUT_EXIT)
        {
            return EXIT_SUCCESS;
        }
        else
            return EXIT_FAILURE;
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}