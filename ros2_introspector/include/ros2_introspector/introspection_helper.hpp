/**
 * @file introspection_helper.hpp
 * @author amparo.sancho-arellano (amparo.sancho-arellano@siemens.com)
 * @brief This class handles the extraction of the information of the instrospection tool
 * @version 0.1.0
 * @date 2025-05-20
 *
 * Copyright (c) Siemens AG 2024 All Rights Reserved.
 *
 */
#pragma once

#include <cstdlib>
#include <fstream>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <string>

// custom header
#include "ros2_introspector/constants.hpp"
#include "utils/logger.hpp"
#include "utils/return_value.hpp"

/**
 * @brief Data structure to hold all the introspection related data
 *
 */
struct IntrospectionData
{
    std::string type_interface; // The type of the interface (msg, srv, or action)
    std::string type_name;      // The name of the interface
    std::string type_hash;      // The hash of the interface
    std::string node_name;      // The name of the node
    std::string package_name;   // The name of the package
    std::string topic_name;     // The name of the topic
    std::string endpoint_type;  // The type of the endpoint (publisher or subscriber)
    std::string command_output; // The output of each command
    std::string operating_mode; // "all" to obtain all interfaces directly or select to obtain just the desired one

    std::vector<std::string> available_interfaces; // List of the available interfaces to extract the information
    std::vector<std::string> all_type_interface;   // The type of the interface (msg, srv, or action)
    std::vector<std::string> all_type_name;        // The name of the interface
    std::vector<std::string> all_type_hash;        // The hash of the interface
    std::vector<std::string> all_node_name;        // The name of the node
    std::vector<std::string> all_package_name;     // The name of the package
    std::vector<std::string> all_topic_name;       // The name of the topic
    std::vector<std::string> all_endpoint_type;    // The type of the endpoint (publisher or subscriber)
    std::vector<std::string> all_command_output;   // The output of each command

    std::vector<std::string> available_msg_interfaces;     // List of the available msg interfaces
    std::vector<std::string> available_service_interfaces; // List of the available service interfaces
    std::vector<std::string> available_action_interfaces;  // List of the available action interfaces
    std::vector<std::string> available_types;              // List for ros2 communication (msg/service/action)
};


class IntrospectionHelper : public rclcpp::Node
{
public:
    /**
     * @brief Destroy the Introspection Helper object
     *
     */
    ~IntrospectionHelper() = default;

    /**
     * @brief Construct a new Introspection Helper object with default ros 2 parameters
     *
     */
    IntrospectionHelper() :
        Node("ros2_introspector_node")
    {
        this->declare_parameters<std::string>("",
                                              {
                                                  {               "mode", "select" },
                                                  {     "interface_type",       "" },
                                                  { "selected_interface",       "" },
                                                  {          "ros2_path",       "" }
        });
    }

    /**
     * @brief Function to extract the topic type, hash and node from ros2 topic info -v output and to know if it is a subscription or
     * publication
     * @return utils::ReturnValue SUCESS if the information was successfully extracted, HASH_TYPE_ERROR otherwise
     */
    [[nodiscard]] utils::ReturnValue get_topic_type_hash_and_node() noexcept;

    /**
     * @brief Function to extract the action type, hash and node from `ros2 action info -t` output
     * @return utils::ReturnValue SUCESS if the information was successfully extracted, HASH_TYPE_ERROR otherwise
     */
    [[nodiscard]] utils::ReturnValue get_action_type_hash_and_node() noexcept;

    /**
     * @brief Function to extract the service type, hash and node from `ros2 service info` output
     * @return utils::ReturnValue SUCESS if the information was successfully extracted, HASH_TYPE_ERROR otherwise
     */
    [[nodiscard]] utils::ReturnValue get_srv_type_hash_and_node() noexcept;

    /**
     * @brief Function to run a shell command and capture the output
     *
     * @param[in] command Command to execute
     * @return utils::ReturnValue SUCESS if the command was executed successfully, CLI_POPEN_ERROR otherwise
     */
    [[nodiscard]] utils::ReturnValue run_command(const std::string & command) noexcept;

    /**
     * @brief Interaction step with the user to know the type interface
     *
     * @return utils::ReturnValue SUCESS if the user chosee one of the correct types,  EXITED_BY_USER if the user exited the program,
     * FAILURE otherwise
     */
    [[nodiscard]] utils::ReturnValue process_type_interface_user_input() noexcept;

    /**
     * @brief Interaction step with the user to see the available interfaces
     *
     * @return utils::ReturnValue SUCESS if the user chosee see/not see interfaces,  EXITED_BY_USER if the user exited the program,
     * FAILURE otherwise
     */
    [[nodiscard]] utils::ReturnValue process_available_interfaces_user_input() noexcept;

    /**
     * @brief Interaction step with the user to see the available interfaces
     *
     * @return utils::ReturnValue SUCESS if the user chosee see/not see interfaces,  EXITED_BY_USER if the user exited the program,
     * FAILURE otherwise
     */
    [[nodiscard]] utils::ReturnValue process_available_interfaces() noexcept;

    /**
     * @brief Interaction step with the user to select the interface
     *
     * @return utils::ReturnValue SUCESS if the user chosee one of the interfaces,  EXITED_BY_USER if the user exited the program,
     * FAILURE otherwise
     */
    [[nodiscard]] utils::ReturnValue process_select_interface_user_input() noexcept;


    /**
     * @brief Interaction step with the user to select the interface
     *
     * @return utils::ReturnValue SUCESS if the user chosee one of the interfaces,  EXITED_BY_USER if the user exited the program,
     * FAILURE otherwise
     */
    [[nodiscard]] utils::ReturnValue process_all_interfaces() noexcept;

    /**
     * @brief Function to call the required process method based on the value of the operating_mode attribute
     *
     * @return utils::ReturnValue SUCESS if the interfaces are processed successfully, corresponding error otherwise
     */
    [[nodiscard]] utils::ReturnValue process_interfaces_based_on_mode() noexcept;

    /**
     * @brief Get the Data object
     *
     * @return const IntrospectionData&
     */
    [[nodiscard]] const IntrospectionData & getData() const
    {
        return data_;
    }

    /**
     * @brief Get the Data object being able to modify it
     *
     * @return IntrospectionData&
     */
    [[nodiscard]] IntrospectionData & getData()
    {
        return data_;
    }

    /**
     * @brief Shared Pointer of the IntrospectionHelper class
     *
     * @return std::shared_ptr<IntrospectionHelper>
     */
    std::shared_ptr<IntrospectionHelper> shared_from_this_helper()
    {
        return std::dynamic_pointer_cast<IntrospectionHelper>(shared_from_this());
    }

private:
    IntrospectionData data_;
    utils::Logger logger_ = utils::Logger("Introspection_logger"); // Custom logger from the utils
};