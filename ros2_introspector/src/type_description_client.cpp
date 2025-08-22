/**
 * @file type_description_client.cpp
 * @author amparo.sancho-arellano (amparo.sancho-arellano@siemens.com)
 * @brief Implementation of the TypeDescriptionClient class
 * @version 0.1.0
 * @date 2025-08-20
 * 
 * Copyright (c) Siemens AG 2024 All Rights Reserved.
 * 
 */

#include "ros2_introspector/asyncapi_generator.hpp"
#include "ros2_introspector/type_decription_client.hpp"

utils::ReturnValue TypeDescriptionClient::start()
{
    while (!client_->wait_for_service(std::chrono::seconds(Constants::SERVICE_TIMEOUT_SECONDS)))
    {
        RCLCPP_INFO(get_logger(), "Waiting for the service to become available...");
    }

    return send_request();
}

void TypeDescriptionClient::process_type_description(const type_description_interfaces::msg::TypeDescription & type_description,
                                                    std::vector<FieldInfo> & msg_information)
{
    // Process the main type description
    int i = 0;

    // 1. Obtain the name and the type of each field of the interface
    for (const auto & field : type_description.type_description.fields)
    {
        msg_information[i].name = field.name;
        msg_information[i].type = get_type_name(field.type.type_id);

        // Check if the field type is a nested message
        if (!field.type.nested_type_name.empty())
        {
            msg_information[i].type_name_nested_fields = field.type.nested_type_name;

            // If the field has nested msgs, recursively process the nested types
            process_nested_type(type_description, field.type.nested_type_name, msg_information[i]);
        }

        i++;
    }
}

void TypeDescriptionClient::process_nested_type(const type_description_interfaces::msg::TypeDescription & type_description,
                                                const std::string & nested_type_name,
                                                FieldInfo & msg_info,
                                                const int depth)

{
    // Check recursion depth to prevent infinite loops
    if (depth >= Constants::MAX_RECURSION_DEPTH)
    {
        RCLCPP_WARN(get_logger(),
                    "Maximum recursion depth (%d) reached while processing nested type '%s'. Stopping recursion.",
                    Constants::MAX_RECURSION_DEPTH,
                    nested_type_name.c_str());
        return;
    }
    // Look for the nested type in the referenced descriptions of the response
    for (const auto & referenced_type : type_description.referenced_type_descriptions)
    {
        // Find the information of the nested type in the referenced descriptions
        if (referenced_type.type_name == nested_type_name)
        {
            msg_info.nested_fields.resize(referenced_type.fields.size());
            int i = 0;

            // Process the fields of the nested type
            for (const auto & field : referenced_type.fields)
            {
                msg_info.nested_fields[i].name = field.name;
                msg_info.nested_fields[i].type = get_type_name(field.type.type_id);

                // Check for further nesting
                if (!field.type.nested_type_name.empty())
                {
                    msg_info.nested_fields[i].type_name_nested_fields = field.type.nested_type_name;

                    // Recursively process any further nested types
                    process_nested_type(type_description, field.type.nested_type_name, msg_info.nested_fields[i], depth + 1);
                }
                i++;
            }

            break;
        }
    }
}

// Function to get a human-readable type from type_id of the introspection's response
std::string TypeDescriptionClient::get_type_name(const int type_id)
{
    static std::map<int, std::string> type_map = {
        { Constants::TypesId::MESSAGE, "message" },
        {    Constants::TypesId::INT8,    "int8" },
        {   Constants::TypesId::INT16,   "int16" },
        {   Constants::TypesId::INT32,   "int32" },
        {   Constants::TypesId::INT64,   "int64" },
        {   Constants::TypesId::UINT8,   "uint8" },
        {  Constants::TypesId::UINT16,  "uint16" },
        {  Constants::TypesId::UINT32,  "uint32" },
        {  Constants::TypesId::UINT64,  "uint64" },
        { Constants::TypesId::FLOAT32, "float32" },
        { Constants::TypesId::FLOAT64, "float64" },
        {  Constants::TypesId::STRING,  "string" }
    };

    return type_map.count(type_id) ? type_map[type_id] : "unknown";
}

utils::ReturnValue TypeDescriptionClient::send_request()
{
    // Definition of the request for the introspection service
    auto request = std::make_shared<GetTypeDescription::Request>();
    request->type_name = introspection_helper_->getData().type_name;
    request->type_hash = introspection_helper_->getData().type_hash;
    request->include_type_sources = true;

    // Call to the introspection service
    auto node_ptr = this;
    auto callback = [node_ptr](rclcpp::Client<GetTypeDescription>::SharedFuture future) -> utils::ReturnValue
    {
        auto result = future.get();

        if (result->successful)
        {
            // Extract all the description of the response
            const auto & type_description = result->type_description;
            node_ptr->fields_.resize(type_description.type_description.fields.size());

            node_ptr->process_type_description(type_description, node_ptr->fields_);

            AsyncAPIGenerator asyncapi_generator(node_ptr->introspection_helper_, node_ptr->fields_);

            utils::ReturnValue return_value = asyncapi_generator.generate_asyncapi();
            if (!return_value.success)
            {
                node_ptr->completion_flag_ = true; // Mark as complete even on error
                return return_value;
            }
        }

        else
        {
            node_ptr->completion_flag_ = true; // Mark as complete on error
            return { false, utils::ReturnCode::TYPE_DES_SRV_ERROR, "Failed in the typeDescription srv callback" };
        }

        node_ptr->completion_flag_ = true;
        return { true, utils::ReturnCode::OK, "Call to type description service succesfully" };
    };

    client_->async_send_request(request, callback);

    return { true, utils::ReturnCode::OK, "Request to type description service succesfully" };
}