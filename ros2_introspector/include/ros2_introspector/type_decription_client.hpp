/**
 * @file type_decription_client.hpp
 * @author amparo.sancho-arellano (amparo.sancho-arellano@siemens.com)
 * @brief Class to define the Type Description Service in ROS2 Jazzy
 * @version 0.1.0
 * @date 2025-08-20
 *
 * Copyright (c) Siemens AG 2024 All Rights Reserved.
 *
 */
#pragma once

#include "ros2_introspector/introspection_helper.hpp"
#include "utils/field_info.hpp"

#include <cstdlib>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <type_description_interfaces/srv/get_type_description.hpp>

using GetTypeDescription = type_description_interfaces::srv::GetTypeDescription;

/**
 * @brief Class to handle the Type Description service
 *
 */
class TypeDescriptionClient : public rclcpp::Node
{
public:
    /**
     * @brief Deleted default constructor to prevent instantiation without parameters
     *
     */
    TypeDescriptionClient() = delete;

    /**
     * @brief Construct a new Type Description Client object
     *
     * @param[in] introspection_helper Object of the Introspection Helper class to access its members
     */
    TypeDescriptionClient(std::shared_ptr<IntrospectionHelper> introspection_helper) :
        Node("type_description_client"),
        introspection_helper_(introspection_helper)
    {
        std::string service_name = introspection_helper_->getData().node_name + "/get_type_description";
        client_ = this->create_client<GetTypeDescription>(service_name);
    }

    /**
     * @brief Destroy the Type Description Client object
     *
     */
    ~TypeDescriptionClient() = default;

    /**
     * @brief Start the introspection service and generate the AsyncAPI documentation
     *
     * @return utils::ReturnValue The value of the return of the send_request method
     */
    utils::ReturnValue start();

    bool is_complete() const
    {
        return completion_flag_;
    }

private:
    /**
     * @brief Function to extract all the information from the response of the introspection service
     * @param[in] type_description The type description response from the introspection service
     * @param[out] msg_information The vector to store the field information
     */
    void process_type_description(const type_description_interfaces::msg::TypeDescription & type_description,
                                  std::vector<FieldInfo> & msg_information);


    /**
     * @brief Function to extract all the information from the nested types of a field
     * @param[in] type_description The type description response from the introspection service
     * @param[in] nested_type_name The name of the nested type
     * @param[out] msg_info The field information to be updated
     * @param[in] depth Counter of recursive executions of the function
     */
    void process_nested_type(const type_description_interfaces::msg::TypeDescription & type_description,
                             const std::string & nested_type_name,
                             FieldInfo & msg_info,
                             const int depth = 0);


    /**
     * @brief Function to get a human-readable type from type_id of the introspection's response
     * @param[in] type_id The type ID from the introspection response
     * @return The human-readable type name
     */
    [[nodiscard]] std::string get_type_name(const int type_id);

    /**
     * @brief Function to send the request to the introspection service and generate the AsyncAPI documentation
     *
     * @return utils::ReturnValue SUCESS if the request and the generation process finish succesfully, if error from:
                - AsyncAPI Generator: return_value of the generate_asyncapi method
                - Type description service: TYPE_DES_SRV_ERROR
     */
    [[nodiscard]] utils::ReturnValue send_request();

    /********* VARIABLES **************************************************************************/
    rclcpp::Client<GetTypeDescription>::SharedPtr client_; // Pointer to a ROS2 client for the GetTypeDescription service
    std::shared_ptr<IntrospectionHelper>
        introspection_helper_;      // Class that handles the extraction of the information of the instrospection tool
    std::vector<FieldInfo> fields_; // Struct to save the information of the interfaces
    bool completion_flag_{ false };
};