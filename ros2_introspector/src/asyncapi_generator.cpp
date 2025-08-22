/**
 * @file asyncapi_generator.cpp
 * @author amparo.sancho-arellano (amparo.sancho-arellano@siemens.com)
 * @brief Implementation of AsyncAPIGenerator class
 * @version 0.1.0
 * @date 2025-08-20
 * 
 * Copyright (c) Siemens AG 2024 All Rights Reserved.
 * 
 */

#include "ros2_introspector/asyncapi_generator.hpp"

utils::ReturnValue AsyncAPIGenerator::update_properties(const std::vector<FieldInfo> & fields,
                                                        const std::string & type_interface,
                                                        const std::string & type_name_with_capital_letters,
                                                        const std::string & package_name,
                                                        const int depth)
{
    // Check recursion depth to prevent infinite loops
    if (depth >= Constants::MAX_RECURSION_DEPTH)
    {
        LOG_WARN(logger_,
                 "Maximum recursion depth ({}) reached while processing type '{}'. Stopping recursion.",
                 Constants::MAX_RECURSION_DEPTH,
                 type_name_with_capital_letters);
        return { false,
                 utils::ReturnCode::ASYNC_RECURSION_LIMIT_REACHED,
                 "Maximum recursion depth reached for type: " + type_name_with_capital_letters };
    }

    // Name of the AsyncAPI 3.0 Interface file
    const std::string filename_interface = Constants::Paths::OUTPUT_INTERFACE_DIR / (package_name + ".yaml");

    // Creation of the yaml node variable to save all the information following the asyncAPI standard
    YAML::Node asyncapi_interface;

    // Check if the output folder exists, create it if it doesn't
    if (!std::filesystem::exists(Constants::Paths::OUTPUT_INTERFACE_DIR))
    {
        if (!std::filesystem::create_directories(Constants::Paths::OUTPUT_INTERFACE_DIR))
        {
            return { false,
                     utils::ReturnCode::ASYNC_CREATE_FOLDER_ERROR,
                     "Failed to create output folder: " + Constants::Paths::OUTPUT_INTERFACE_DIR.string() };
        }
    }

    // Check if the AsyncAPI file already exists in the output folder
    if (std::filesystem::exists(filename_interface))
    {
        std::ifstream file_in(filename_interface);
        asyncapi_interface = YAML::Load(file_in);
        file_in.close();
    }

    else
    {
        // If file does not exist, create the base structure
        asyncapi_interface["asyncapi"] = "3.0.0";
        asyncapi_interface["info"]["title"] = package_name;
        asyncapi_interface["info"]["version"] = "1.0.0";
    }

    // Check if the component exists
    if (!asyncapi_interface["components"]["messages"][type_name_with_capital_letters])
    {
        asyncapi_interface["components"]["messages"][type_name_with_capital_letters] = YAML::Node(YAML::NodeType::Map);

        // Create an array for the "tags" field
        YAML::Node tags_array(YAML::NodeType::Sequence);
        YAML::Node tag_name;
        tag_name["name"] = type_interface;
        tags_array.push_back(tag_name);

        asyncapi_interface["components"]["messages"][type_name_with_capital_letters]["tags"] = tags_array;
        asyncapi_interface["components"]["messages"][type_name_with_capital_letters]["payload"]["type"] = "object";

        // Complete the asyncAPI file with the info of the different fields
        for (const auto & field : fields)
        {
            const std::string & field_name = field.name;
            const std::string & ros_type = field.type;

            // Check if the field type is a nested message
            if (!field.nested_fields.empty())
            {
                // Extract the last part of the type name to obtain the nested package name (before the first '/')
                size_t slash_pos = field.type_name_nested_fields.find("/");
                std::string nested_package_name;

                if (slash_pos != std::string::npos)
                {
                    nested_package_name = field.type_name_nested_fields.substr(0, slash_pos);
                }

                // Extract the last part of the type name to obtain the nested msg name
                const std::string nested_type_name = field.type_name_nested_fields;

                std::string nested_type_name_with_capital_letters = replace_undercores_with_pascal_case(nested_type_name);

                // Check if the AsyncAPI file already exists in the output folder
                if (std::filesystem::exists(filename_interface))
                {
                    std::ifstream file_in(filename_interface);
                    asyncapi_interface = YAML::Load(file_in);
                    file_in.close();
                }

                else
                {
                    // If file does not exist, create the base structure
                    asyncapi_interface["asyncapi"] = "3.0.0";
                    asyncapi_interface["info"]["title"] = package_name;
                    asyncapi_interface["info"]["version"] = "1.0.0";
                }

                if (nested_package_name != package_name)
                {
                    asyncapi_interface["components"]["messages"][type_name_with_capital_letters]["payload"]["properties"][field_name]
                                      ["$ref"] = "./" + nested_package_name + ".yaml#/components/messages/"
                                                 + nested_type_name_with_capital_letters + "/payload";
                }
                else
                {
                    asyncapi_interface["components"]["messages"][type_name_with_capital_letters]["payload"]["properties"][field_name]
                                      ["$ref"] = "#/components/messages/" + nested_type_name_with_capital_letters + "/payload";
                }

                // Save the updated Interface AsyncAPI document back to the YAML file
                std::ofstream interface_yaml_file(filename_interface);
                if (!interface_yaml_file.is_open())
                {
                    return { false, utils::ReturnCode::ASYNC_OPEN_FILE_ERROR, "Failed to open file: " + filename_interface };
                }
                interface_yaml_file << asyncapi_interface;
                interface_yaml_file.close();

                // Recursively process the nested type
                utils::ReturnValue nested_return_value = update_properties(field.nested_fields,
                                                                           type_interface,
                                                                           nested_type_name_with_capital_letters,
                                                                           nested_package_name,
                                                                           depth + 1);
                if (!nested_return_value.success)
                {
                    return nested_return_value;
                }
            }

            else
            {
                // Check if the AsyncAPI file already exists in the output folder
                if (std::filesystem::exists(filename_interface))
                {
                    std::ifstream file_in(filename_interface);
                    asyncapi_interface = YAML::Load(file_in);
                    file_in.close();
                }

                else
                {
                    // If file does not exist, create the base structure
                    asyncapi_interface["asyncapi"] = "3.0.0";
                    asyncapi_interface["info"]["title"] = package_name;
                    asyncapi_interface["info"]["version"] = "1.0.0";
                }

                // Map ROS type to AsyncAPI type
                std::string asyncapi_type = map_ros_type_to_asyncapi(ros_type);
                asyncapi_interface["components"]["messages"][type_name_with_capital_letters]["payload"]["properties"][field_name]["type"] =
                    asyncapi_type;

                // If there's a format (e.g., int8, float, etc.), add it
                auto it = type_mapping_.find(ros_type);
                if (it != type_mapping_.end() && !it->second.format.empty())
                {
                    asyncapi_interface["components"]["messages"][type_name_with_capital_letters]["payload"]["properties"][field_name]
                                      ["format"] = it->second.format;
                }

                // Save the updated Interface AsyncAPI document back to the YAML file
                std::ofstream interface_yaml_file(filename_interface);
                if (!interface_yaml_file.is_open())
                {
                    return { false, utils::ReturnCode::ASYNC_OPEN_FILE_ERROR, "Failed to open file: " + filename_interface };
                }
                interface_yaml_file << asyncapi_interface;
                interface_yaml_file.close();
            }
        }
    }
    return { true, utils::ReturnCode::OK, "Succesfully updated AsyncAPI properties" };
};

utils::ReturnValue AsyncAPIGenerator::generate_asyncapi()
{
    utils::ReturnValue return_value;

    // Name of the AsyncAPI 3.0 Head document file
    const std::string filename = Constants::Paths::OUTPUT_DIR / "Head-asyncapi.yaml";

    // Creation of the yaml node variable to save all the information following the asyncAPI standard
    YAML::Node asyncapi;

    // Remove the starting slash from the topic name
    std::string topic_name_without_slash = introspection_helper_->getData().topic_name;

    // Check if the topic name starts with '/'
    if (!introspection_helper_->getData().topic_name.empty() && introspection_helper_->getData().topic_name[0] == '/')
    {
        topic_name_without_slash.erase(0, 1); // Remove the leading '/'
    }

    std::string type_name_with_capital_letters = replace_undercores_with_pascal_case(introspection_helper_->getData().type_name);

    // Check if the output folder exists, create it if it doesn't
    if (!std::filesystem::exists(Constants::Paths::OUTPUT_DIR))
    {
        std::filesystem::create_directories(Constants::Paths::OUTPUT_DIR);
    }

    // Check if the AsyncAPI file already exists in the output folder
    if (std::filesystem::exists(filename))
    {
        std::ifstream file_in(filename);
        asyncapi = YAML::Load(file_in);
        file_in.close();
    }

    else
    {
        // If file does not exist, create the base structure
        asyncapi["asyncapi"] = "3.0.0";
        asyncapi["info"]["title"] = "Head AsyncAPI definition";
        asyncapi["info"]["version"] = "1.0.0";
    }

    // Determine the operation type
    std::string operation_type = "";

    if (introspection_helper_->getData().endpoint_type == "SUBSCRIPTION")
    {
        operation_type = "subscriber";
    }

    else if (introspection_helper_->getData().endpoint_type == "PUBLISHER")
    {
        operation_type = "publisher";
    }

    // If it is an action or a server the endpoint_type is alredy the required string
    else
        operation_type = introspection_helper_->getData().endpoint_type;

    // Creation of the channel
    // Completing the messages of the channel depending of the type inteface
    if (introspection_helper_->getData().type_interface == "msg")
    {
        asyncapi["channels"][type_name_with_capital_letters]["address"] = introspection_helper_->getData().topic_name;
        asyncapi["channels"][type_name_with_capital_letters]["messages"][type_name_with_capital_letters]["$ref"] =
            "./InterfaceLibrary/" + introspection_helper_->getData().package_name + ".yaml#/components/messages/"
            + type_name_with_capital_letters;
    }

    else if (introspection_helper_->getData().type_interface == "action" || introspection_helper_->getData().type_interface == "srv")
    {
        asyncapi["channels"][type_name_with_capital_letters + "Request"]["address"] = introspection_helper_->getData().topic_name;
        asyncapi["channels"][type_name_with_capital_letters + "Request"]["messages"][type_name_with_capital_letters + "Request"]["$ref"] =
            "./InterfaceLibrary/" + introspection_helper_->getData().package_name + ".yaml#/components/messages/"
            + type_name_with_capital_letters + "Request";

        asyncapi["channels"][type_name_with_capital_letters + "Reply"]["address"] = introspection_helper_->getData().topic_name;
        asyncapi["channels"][type_name_with_capital_letters + "Reply"]["messages"][type_name_with_capital_letters + "Result"]["$ref"] =
            "./InterfaceLibrary/" + introspection_helper_->getData().package_name + ".yaml#/components/messages/"
            + type_name_with_capital_letters + "Response";

        if (introspection_helper_->getData().type_interface == "action")
        {
            asyncapi["channels"][type_name_with_capital_letters + "Request"]["messages"][type_name_with_capital_letters + "Request"]
                    ["$ref"] = "./InterfaceLibrary/" + introspection_helper_->getData().package_name + ".yaml#/components/messages/"
                               + type_name_with_capital_letters + "Goal";
            asyncapi["channels"][type_name_with_capital_letters + "Reply"]["messages"][type_name_with_capital_letters + "Result"]["$ref"] =
                "./InterfaceLibrary/" + introspection_helper_->getData().package_name + ".yaml#/components/messages/"
                + type_name_with_capital_letters + "Result";
            asyncapi["channels"][type_name_with_capital_letters + "Reply"]["messages"][type_name_with_capital_letters + "Feedback"]
                    ["$ref"] = "./InterfaceLibrary/" + introspection_helper_->getData().package_name + ".yaml#/components/messages/"
                               + type_name_with_capital_letters + "Feedback";
        }
    }

    // Creation of the operation
    YAML::Node interface_for_operation;
    interface_for_operation["action"] =
        (operation_type == "publisher" || operation_type == "action_client" || operation_type == "service_client") ? "send" : "receive";

    // Defining only channel (msg) or channel and reply (actions)
    if (introspection_helper_->getData().type_interface == "msg")
    {
        interface_for_operation["channel"]["$ref"] = "#/channels/" + type_name_with_capital_letters;
    }

    else if (introspection_helper_->getData().type_interface == "action" || introspection_helper_->getData().type_interface == "srv")
    {
        interface_for_operation["channel"]["$ref"] = "#/channels/" + type_name_with_capital_letters + "Request";
        interface_for_operation["reply"]["channel"]["$ref"] = "#/channels/" + type_name_with_capital_letters + "Reply";
    }

    // Defining the bindings for ros2 of the operation
    interface_for_operation["bindings"]["x-ros2"]["role"] = operation_type;
    interface_for_operation["bindings"]["x-ros2"]["node"] = introspection_helper_->getData().node_name;

    asyncapi["operations"][type_name_with_capital_letters] = interface_for_operation;

    // Save the updated Head AsyncAPI document back to the YAML file
    std::ofstream yaml_file(filename);
    yaml_file << asyncapi;
    yaml_file.close();

    LOG_INFO(logger_, "Head AsyncAPI 3.0 YAML file updated: {}", filename);

    // Add or update the AsyncAPI 3.0 Interface documents
    return update_properties(fields_,
                             introspection_helper_->getData().type_interface,
                             type_name_with_capital_letters,
                             introspection_helper_->getData().package_name);
}


std::string AsyncAPIGenerator::map_ros_type_to_asyncapi(const std::string & ros_type)
{
    auto it = type_mapping_.find(ros_type);

    if (it != type_mapping_.end())
    {
        return it->second.type; // Return AsyncAPI type
    }

    else
    {
        return "string"; // Default type
    }
}

std::string AsyncAPIGenerator::replace_undercores_with_pascal_case(const std::string & type_name)
{
    std::string result;
    std::string type_name_last_part = type_name;
    bool capitalize_next = false;

    size_t last_slash_pos = type_name.find_last_of('/');

    if (last_slash_pos != std::string::npos)
    {
        type_name_last_part = type_name_last_part.substr(last_slash_pos + 1);
    }

    for (char c : type_name_last_part)
    {
        if (c == '_')
        {
            capitalize_next = true;
        }

        else
        {
            if (capitalize_next)
            {
                result += static_cast<char>(std::toupper(c));
                capitalize_next = false;
            }

            else
            {
                result += c;
            }
        }
    }

    return result;
}
