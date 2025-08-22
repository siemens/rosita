/**
 * @file introspection_helper.cpp
 * @author amparo.sancho-arellano (amparo.sancho-arellano@siemens.com)
 * @brief Implementation of the IntrospectionHelper class
 * @version 0.1.0
 * @date 2025-05-20
 *
 * Copyright (c) Siemens AG 2024 All Rights Reserved.
 *
 */

#include "ros2_introspector/introspection_helper.hpp"
#include "ros2_introspector/type_decription_client.hpp"

utils::ReturnValue IntrospectionHelper::run_command(const std::string & command) noexcept
{
    std::array<char, Constants::BUFFER_SIZE> buffer{};
    std::shared_ptr<FILE> pipe(popen(command.c_str(), "r"), pclose);

    data_.command_output.erase();
    if (!pipe)
        return { false, utils::ReturnCode::CLI_POPEN_ERROR, "The execution of the command to obtain the hash failed! Please try again" };
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    {
        data_.command_output += buffer.data();
    }
    return { true, utils::ReturnCode::OK, "Command executed correctly!" };
}

utils::ReturnValue IntrospectionHelper::get_topic_type_hash_and_node() noexcept
{
    try
    {
        // Execute the command shell to obtain the information of the topic
        const std::string command = "ros2 topic info -v " + data_.topic_name;
        if (data_.operating_mode == "all")
            data_.all_topic_name.push_back(data_.topic_name);

        utils::ReturnValue return_value = IntrospectionHelper::run_command(command);
        if (!return_value.success)
        {
            return { false, return_value.return_code, return_value.message };
        }
        std::istringstream stream(data_.command_output);
        std::string line;

        // Obtain the type, the hash and the endpoint of the topic and the package name
        while (std::getline(stream, line))
        {
            if (line.find("Type:") != std::string::npos)
            {
                data_.type_name = line.substr(line.find(":") + 2); // Name of the interface (geometry_msgs/msg/Point -> Point)
                if (data_.operating_mode == "all")
                    data_.all_type_name.push_back(data_.type_name);

                // Extract the package name (geometry_msgs/msg/Point -> geometry_msgs)
                size_t slash_pos = data_.type_name.find("/");

                if (slash_pos != std::string::npos)
                {
                    data_.package_name = data_.type_name.substr(0, slash_pos);
                    if (data_.operating_mode == "all")
                        data_.all_package_name.push_back(data_.package_name);
                }
            }

            else if (line.find("Topic type hash:") != std::string::npos)
            {
                data_.type_hash = line.substr(line.find(":") + 2);
                if (data_.operating_mode == "all")
                    data_.all_type_hash.push_back(data_.type_hash);
            }

            else if (line.find("Endpoint type:") != std::string::npos)
            {
                data_.endpoint_type = line.substr(line.find(":") + 2); // To know if it is a subscriber or a publisher
                if (data_.operating_mode == "all")
                    data_.all_endpoint_type.push_back(data_.endpoint_type);
            }

            else if (line.find("Node name:") != std::string::npos)
            {
                // Find the position right after "Node name: "
                size_t pos = line.find("Node name:") + std::string("Node name:").length();
                std::string node = line.substr(pos);

                // Trim leading spaces
                node.erase(0, node.find_first_not_of(" \t"));

                // Adding a slash to the node name since the ros2 topic info retrieves the name without slash
                data_.node_name = "/" + node;
                if (data_.operating_mode == "all")
                    data_.all_node_name.push_back(data_.node_name);
            }
        }

        if (!data_.type_name.empty() && !data_.node_name.empty() && !data_.type_hash.empty() && !data_.endpoint_type.empty())
        {
            return { true, utils::ReturnCode::OK, "Hash and type obtained succesfully" };
        }
        else
        {
            return { false, utils::ReturnCode::HASH_TYPE_ERROR, "Some values when obtaining the hash and the type are empty" };
        }
    }

    catch (const std::exception & e)
    {
        return { false, utils::ReturnCode::HASH_TYPE_ERROR, e.what() };
    }
}

utils::ReturnValue IntrospectionHelper::get_action_type_hash_and_node() noexcept
{
    try
    {
        // Run the command shell to extract the info of the required action
        const std::string command = "ros2 action info -t " + data_.topic_name;
        if (data_.operating_mode == "all")
            data_.all_topic_name.push_back(data_.topic_name);

        utils::ReturnValue return_value = IntrospectionHelper::run_command(command);
        if (!return_value.success)
        {
            return { false, return_value.return_code, return_value.message };
        }

        std::istringstream stream(data_.command_output);
        std::string line;

        // 1. Check if there is an action client or server
        bool found_action_servers = false;
        bool found_action_clients = false;

        while (std::getline(stream, line))
        {
            // Check for Action clients line
            if (line.find("Action clients:") != std::string::npos)
            {
                std::string client_count = line.substr(line.find(":") + 1);
                int action_clients = std::stoi(client_count);

                if (action_clients >= 1)
                {
                    data_.endpoint_type = "action_client";
                    if (data_.operating_mode == "all")
                        data_.all_endpoint_type.push_back(data_.endpoint_type);
                }

                found_action_clients = true;
            }

            // Check for Action servers line
            else if (line.find("Action servers:") != std::string::npos)
            {
                std::string server_count = line.substr(line.find(":") + 1);
                int action_servers = std::stoi(server_count);

                if (action_servers == 1)
                {
                    data_.endpoint_type = "action_server";
                    if (data_.operating_mode == "all")
                        data_.all_endpoint_type.push_back(data_.endpoint_type);
                }

                found_action_servers = true;
            }

            // 2. Extract the node name and interface (type_name)
            else if ((found_action_clients || found_action_servers) && line.find("/") != std::string::npos)
            {
                // Extract the node name
                data_.node_name = line.substr(line.find("/") + 0); // Extract after the "/"
                size_t space_pos = data_.node_name.find(' ');      // Find the position of the first space

                if (space_pos != std::string::npos)
                {
                    data_.node_name.erase(space_pos); // Remove everything from the first space onward
                    if (data_.operating_mode == "all")
                        data_.all_node_name.push_back(data_.node_name);
                }

                // Extract the interface name from the line (that is between [])
                size_t start = line.find("[");
                size_t end = line.find("]");

                if (start != std::string::npos && end != std::string::npos && end > start)
                {
                    data_.type_name = line.substr(start + 1, end - start - 1); // Extract content within brackets
                    if (data_.operating_mode == "all")
                        data_.all_type_name.push_back(data_.type_name);
                }

                // Extract the package name (before the first '/')
                size_t slash_pos = data_.type_name.find("/");

                if (slash_pos != std::string::npos)
                {
                    data_.package_name = data_.type_name.substr(0, slash_pos);
                    if (data_.operating_mode == "all")
                        data_.all_package_name.push_back(data_.package_name);
                }

                break; // Exit loop after processing the first match
            }
        }

        // If no action clients or servers found, throw an error
        if (!found_action_clients && !found_action_servers)
        {
            return { false, utils::ReturnCode::HASH_TYPE_ERROR, "No action clients or action servers found for " + data_.topic_name };
        }

        // 3. Obtain the Hash from the JSON data (type hash discovery not available yet for actions)
        // Construct file path for the JSON file
        std::string file_path = "./install/" + data_.package_name + "/share/" + data_.type_name + ".json";

        // Open the file
        std::ifstream file(file_path);
        nlohmann::json json_data;
        if (std::filesystem::exists(file_path))
        {
            if (!file.is_open())
            {
                return { false, utils::ReturnCode::HASH_TYPE_ERROR, "Could not open the file: " + file_path };
            }

            // Read the JSON file
            file >> json_data;
        }
        else
        {
            // Obtain the Hash from the JSON data from the default install folder for the ros installation
            file_path = get_parameter("ros2_path").as_string() + data_.type_name + ".json";
            std::ifstream file(file_path);

            if (!file.is_open())
            {
                return { false, utils::ReturnCode::HASH_TYPE_ERROR, "Could not open the file: " + file_path };
            }

            // Read the JSON file
            file >> json_data;
        }

        // Access the hash from the JSON data
        if (json_data.contains("type_hashes") && !json_data["type_hashes"].empty())
        {
            data_.type_hash = json_data["type_hashes"][0]["hash_string"];
            if (data_.operating_mode == "all")
                data_.all_type_hash.push_back(data_.type_hash);
        }

        else
        {
            return { false, utils::ReturnCode::HASH_TYPE_ERROR, "Hash not found in JSON file: " + file_path };
        }


        if (!data_.type_name.empty() && !data_.node_name.empty() && !data_.type_hash.empty() && !data_.endpoint_type.empty())
        {
            return { true, utils::ReturnCode::OK, "Hash and type obtained succesfully" };
        }
        else
        {
            return { false, utils::ReturnCode::HASH_TYPE_ERROR, "Some values when obtaining the hash and the type are empty" };
        }
    }

    catch (const std::exception & e)
    {
        return { false, utils::ReturnCode::HASH_TYPE_ERROR, e.what() };
    }
}

utils::ReturnValue IntrospectionHelper::get_srv_type_hash_and_node() noexcept
{
    try
    {
        // 1. Run the command shell to extract the info of the required service
        std::string command = "ros2 service info " + data_.topic_name;
        if (data_.operating_mode == "all")
            data_.all_topic_name.push_back(data_.topic_name);

        utils::ReturnValue return_value = IntrospectionHelper::run_command(command);
        if (!return_value.success)
        {
            return { false, return_value.return_code, return_value.message };
        }

        std::istringstream stream(data_.command_output);
        std::string line;

        bool found_srv_servers = false;
        bool found_srv_clients = false;

        while (std::getline(stream, line))
        {
            // Check for interface name
            if (line.find("Type: ") != std::string::npos)
            {
                // Extract the interface name
                data_.type_name = line.substr(line.find(":") + 2); // Extract after the "Type: "
                if (data_.operating_mode == "all")
                    data_.all_type_name.push_back(data_.type_name);

                // Extract the package name (before the first '/')
                size_t slash_pos = data_.type_name.find("/");

                if (slash_pos != std::string::npos)
                {
                    data_.package_name = data_.type_name.substr(0, slash_pos);
                    if (data_.operating_mode == "all")
                        data_.all_package_name.push_back(data_.package_name);
                }
            }

            // Check for server clients line
            if (line.find("Clients count:") != std::string::npos)
            {
                std::string client_count = line.substr(line.find(":") + 1);
                int service_clients = std::stoi(client_count);

                if (service_clients >= 1)
                {
                    data_.endpoint_type = "service_client";
                    if (data_.operating_mode == "all")
                        data_.all_endpoint_type.push_back(data_.endpoint_type);
                }

                found_srv_clients = true;
            }

            // Check for service servers line
            else if (line.find("Services count:") != std::string::npos)
            {
                std::string server_count = line.substr(line.find(":") + 1);
                int service_servers = std::stoi(server_count);

                if (service_servers == 1)
                {
                    data_.endpoint_type = "service_server";
                    if (data_.operating_mode == "all")
                        data_.all_endpoint_type.push_back(data_.endpoint_type);
                }

                found_srv_servers = true;
            }
        }

        // If no srv clients or servers found, throw an error
        if (!found_srv_clients && !found_srv_servers)
        {
            return { false, utils::ReturnCode::HASH_TYPE_ERROR, "No srv clients or srv servers found for " + data_.topic_name };
        }

        // 2. Obtain the Hash from the JSON data (type hash discovery not available yet for services)
        // Construct file path for the JSON file
        std::string file_path = "./install/" + data_.package_name + "/share/" + data_.type_name + ".json";

        // Open the file
        std::ifstream file(file_path);
        nlohmann::json json_data;
        if (std::filesystem::exists(file_path))
        {
            if (!file.is_open())
            {
                return { false, utils::ReturnCode::HASH_TYPE_ERROR, "Could not open the file: " + file_path };
            }

            // Read the JSON file
            file >> json_data;
        }
        else
        {
            // Obtain the Hash from the JSON data from the default install folder for the ros installation
            file_path = get_parameter("ros2_path").as_string() + data_.type_name + ".json";
            std::ifstream file(file_path);

            if (!file.is_open())
            {
                return { false, utils::ReturnCode::HASH_TYPE_ERROR, "Could not open the file: " + file_path };
            }

            // Read the JSON file
            file >> json_data;
        }

        // Access the hash from the JSON data
        if (json_data.contains("type_hashes") && !json_data["type_hashes"].empty())
        {
            data_.type_hash = json_data["type_hashes"][0]["hash_string"];
            if (data_.operating_mode == "all")
                data_.all_type_hash.push_back(data_.type_hash);
        }

        else
        {
            return { false, utils::ReturnCode::HASH_TYPE_ERROR, "Hash not found in JSON file: " + file_path };
        }

        // 3. Extract the node name
        auto introspection_node = rclcpp::Node::make_shared("service_introspection_node"); // Create a node for introspection
        auto node_names = introspection_node->get_node_names();                            // List all nodes
        static constexpr const char* valid_node_name_pattern = "^[a-zA-Z0-9_]+$";
        static const std::regex valid_node_name_regex(valid_node_name_pattern);            // Define valid node name convention
        bool found_service = false;                                                        // Track if the service is found

        // Iterate through all nodes to find which one provides or uses the target service
        for (const auto & node : node_names)
        {
            // Remove leading `/` to normalize the node name
            std::string normalized_node = node;

            if (!node.empty() && node[0] == '/')
            {
                normalized_node = node.substr(1);
            }

            // Check if the node name is valid
            if (!std::regex_match(normalized_node, valid_node_name_regex))
            {
                RCLCPP_WARN(introspection_node->get_logger(), "Skipping invalid node name: %s", normalized_node.c_str());
                continue; // Skip invalid node names
            }

            try
            {
                // Get services provided by the node
                auto services = introspection_node->get_service_names_and_types_by_node(normalized_node, "");

                for (const auto & service : services)
                {
                    // Check if the service matches the target
                    if (service.first == data_.topic_name)
                    {
                        data_.node_name = node; // Assing the node name
                        if (data_.operating_mode == "all")
                            data_.all_node_name.push_back(data_.node_name);
                        found_service = true;
                    }
                }
            }

            catch (const std::exception & e)
            {
                RCLCPP_ERROR(introspection_node->get_logger(),
                             "Error retrieving services for node '%s': %s",
                             normalized_node.c_str(),
                             e.what());
            }

            if (found_service)
                break; // Exit loop if service is found
        }

        if (!found_service)
        {
            RCLCPP_ERROR(introspection_node->get_logger(), "Failed to retrieve node providing the service '%s'", data_.topic_name.c_str());
        }

        if (!data_.type_name.empty() && !data_.node_name.empty() && !data_.type_hash.empty() && !data_.endpoint_type.empty())
        {
            return { true, utils::ReturnCode::OK, "Hash and type obtained succesfully" };
        }
        else
        {
            return { false, utils::ReturnCode::HASH_TYPE_ERROR, "Some values when obtaining the hash and the type are empty" };
        }
    }

    catch (const std::exception & e)
    {
        return { false, utils::ReturnCode::HASH_TYPE_ERROR, e.what() };
    }
}

utils::ReturnValue IntrospectionHelper::process_type_interface_user_input() noexcept
{
    // Check if the mode is "all"
    if (data_.operating_mode == "all")
    {
        LOG_INFO(logger_, "Running in automatic mode - processing all interfaces");

        data_.available_types.push_back("msg");
        data_.available_types.push_back("srv");
        data_.available_types.push_back("action");

        return { true, utils::ReturnCode::OK, "Successfully selected all interface types" };
    }
    // Otherwise, the mode is "select"
    else
    {
        std::string user_input;
        bool correct_input = false;

        // Check if interface type was provided via ros 2 parameter
        data_.type_interface = get_parameter("interface_type").as_string();

        if (!data_.type_interface.empty())
        {
            if (data_.type_interface == "msg" || data_.type_interface == "srv" || data_.type_interface == "action")
            {
                return { true, utils::ReturnCode::OK, "Successfully selected type interface from ros 2 parameter" };
            }
        }

        // If not provided via ros2 parameters, prompt the user
        LOG_INFO(logger_,
                 "Welcome to the Introspection tool to create your custom AsyncAPI Specification!!\n"
                 "Follow the instructions to complete the generation process.\n"
                 "Enter the interface type (msg, srv, or action) or press 'e' to exit the program.");

        std::getline(std::cin, user_input);

        while (!correct_input)
        {
            if ("e" == user_input)
            {
                return { false, utils::ReturnCode::CLI_USER_INPUT_EXIT, "User exited the program." };
            }

            correct_input = ("msg" == user_input || "action" == user_input || "srv" == user_input);
            if (correct_input)
            {
                break;
            }
            LOG_WARN(logger_, "{} is not correct. Please select a correct type or press 'e' to exit the program: ", user_input);
            std::getline(std::cin, user_input);
        }

        data_.type_interface = user_input;
        return { true, utils::ReturnCode::OK, "Successfully selected type interface" };
    }
}

utils::ReturnValue IntrospectionHelper::process_available_interfaces_user_input() noexcept
{
    utils::ReturnValue return_value;

    std::string command;

    // Print the available topics, services, or actions
    if (data_.type_interface == "msg")
    {
        command = "ros2 topic list";
    }
    else if (data_.type_interface == "srv")
    {
        command = "ros2 service list";
    }
    else if (data_.type_interface == "action")
    {
        command = "ros2 action list";
    }

    return_value = run_command(command);
    if (!return_value.success)
    {
        LOG_ERROR(logger_, "{}", return_value.message);
        return { false, utils::ReturnCode::PRINT_INTERFACE_ERROR, "Error printing available " + data_.type_interface };
    }

    std::istringstream stream(data_.command_output);
    std::string interfaces;

    while (std::getline(stream, interfaces))
    {
        // For services, filter out those from our introspection nodes
        if (data_.type_interface == "srv")
        {
            if (interfaces.find("/type_description_client/") == 0 || interfaces.find("/ros2_introspector_node/") == 0)
            {
                continue; // Skip this service
            }
        }
        data_.available_interfaces.push_back(interfaces);
    }

    return { true, utils::ReturnCode::OK, "Listed available interfaces successfully" };
}

utils::ReturnValue IntrospectionHelper::process_available_interfaces() noexcept
{
    utils::ReturnValue return_value;

    std::string command;
    for (auto const & type : data_.available_types)
    {
        data_.type_interface = type;

        // choose the right command
        if (data_.type_interface == "msg")
        {
            command = "ros2 topic list";
        }
        else if (data_.type_interface == "srv")
        {
            command = "ros2 service list";
        }
        else if (data_.type_interface == "action")
        {
            command = "ros2 action list";
        }

        return_value = run_command(command);
        if (!return_value.success)
        {
            LOG_ERROR(logger_, "{}", return_value.message);
            return { false, utils::ReturnCode::PRINT_INTERFACE_ERROR, "Error printing available" + data_.type_interface };
        }


        std::istringstream stream(data_.command_output);
        std::string interfaces = {};

        // fill the correct available_interfaces
        while (std::getline(stream, interfaces))
        {
            if (data_.type_interface == "msg")
            {
                data_.available_msg_interfaces.push_back(interfaces);
            }
            else if (data_.type_interface == "srv")
            {
                // Skip services from our introspection nodes
                if (interfaces.find("/type_description_client/") == 0 || interfaces.find("/ros2_introspector_node/") == 0)
                {
                    continue; // Skip this service
                }
                data_.available_service_interfaces.push_back(interfaces);
            }
            else if (data_.type_interface == "action")
            {
                data_.available_action_interfaces.push_back(interfaces);
            }
        }
    }

    return { true, utils::ReturnCode::OK, "Listed available topics successfully" };
}

utils::ReturnValue IntrospectionHelper::process_all_interfaces() noexcept
{
    utils::ReturnValue return_value;

    for (auto const & type : data_.available_types)
    {
        // get the right available_interfaces
        data_.type_interface = type;

        if (data_.type_interface == "msg")
        {
            data_.available_interfaces = data_.available_msg_interfaces;
        }
        else if (data_.type_interface == "srv")
        {
            data_.available_interfaces = data_.available_service_interfaces;
        }
        else if (data_.type_interface == "action")
        {
            data_.available_interfaces = data_.available_action_interfaces;
        }

        // get all information about the interface
        for (const auto & interface : data_.available_interfaces)
        {
            data_.topic_name = interface;
            if (data_.type_interface == "msg")
            {
                if (data_.topic_name == "/rosout" || data_.topic_name == "/parameter_events") // Not interested since is from ros internally
                    continue;
                else
                {
                    data_.all_type_interface.push_back(data_.type_interface);
                    return_value = get_topic_type_hash_and_node();
                }
            }
            else if (data_.type_interface == "srv")
            {
                data_.all_type_interface.push_back(data_.type_interface);
                return_value = get_srv_type_hash_and_node();
            }
            else if (data_.type_interface == "action")
            {
                data_.all_type_interface.push_back(data_.type_interface);
                return_value = get_action_type_hash_and_node();
            }
        }
    }

    // Process interfaces one by one using a sequential approach
    for (size_t i = 0; i < data_.all_type_name.size(); i++)
    {
        data_.type_interface = data_.all_type_interface[i];
        data_.topic_name = data_.all_topic_name[i];
        data_.type_name = data_.all_type_name[i];
        data_.type_hash = data_.all_type_hash[i];
        data_.node_name = data_.all_node_name[i];
        data_.endpoint_type = data_.all_endpoint_type[i];
        data_.package_name = data_.all_package_name[i];

        LOG_INFO(logger_, "Processing {}", data_.topic_name);

        // Create a new TypeDescriptionClient for this interface
        auto node = std::make_shared<TypeDescriptionClient>(shared_from_this_helper());

        // Start the client and wait for it to complete
        return_value = node->start();
        if (!return_value.success)
        {
            LOG_ERROR(logger_, "Error retrieving type description for interface {}: {}", data_.topic_name, return_value.message);
            continue; // Skip to next interface on error
        }

        // Process this interface to completion before moving to the next
        rclcpp::spin_some(node);

        // Wait for the async operation to complete
        while (rclcpp::ok() && !node->is_complete())
        {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(Constants::SLEEP_MILLISECONDS));
        }
    }

    return { true, utils::ReturnCode::OK, "All interfaces processed successfully" };
}

utils::ReturnValue IntrospectionHelper::process_select_interface_user_input() noexcept
{
    utils::ReturnValue return_value;

    // Check if interface was provided via ros 2 parameter
    data_.topic_name = get_parameter("selected_interface").as_string();

    if (!data_.topic_name.empty())
    {
        LOG_INFO(logger_, "Using selected interface from ros 2 parameters: {}", data_.topic_name);

        // Obtain the required information to create the AsyncAPI doc
        if (data_.type_interface == "msg")
        {
            return_value = get_topic_type_hash_and_node();
        }
        else if (data_.type_interface == "action")
        {
            return_value = get_action_type_hash_and_node();
        }
        else if (data_.type_interface == "srv")
        {
            return_value = get_srv_type_hash_and_node();
        }

        return return_value;
    }

    // Prompt the user to select an interface from the list
    std::string selected_interface;
    bool valid_input = false;

    while (!valid_input)
    {
        LOG_INFO(logger_, "Available {}", data_.type_interface);
        for (const auto & interface : data_.available_interfaces)
        {
            LOG_INFO(logger_, "- {}", interface);
        }
        LOG_INFO(logger_, "Enter the {} name or 'e' to exit:", data_.type_interface);

        std::getline(std::cin, selected_interface);

        if (selected_interface == "e")
        {
            return { false, utils::ReturnCode::CLI_USER_INPUT_EXIT, "User exited the program." };
        }

        // Check if the selected interface is in the list of available interfaces
        if (std::find(data_.available_interfaces.begin(), data_.available_interfaces.end(), selected_interface) != data_.available_interfaces.end())
        {
            data_.topic_name = selected_interface;
            valid_input = true;
        }
        else
        {
            LOG_WARN(logger_, "{} is not a valid {} name. Please try again.", selected_interface, data_.type_interface);
        }
    }

    // Obtain the required information to create the AsyncAPI doc
    if (data_.type_interface == "msg")
    {
        return_value = get_topic_type_hash_and_node();
    }
    else if (data_.type_interface == "action")
    {
        return_value = get_action_type_hash_and_node();
    }
    else if (data_.type_interface == "srv")
    {
        return_value = get_srv_type_hash_and_node();
    }

    return return_value;
}

utils::ReturnValue IntrospectionHelper::process_interfaces_based_on_mode() noexcept
{
    utils::ReturnValue return_value;
    if (data_.operating_mode == "all")
    {
        // Process all interfaces automatically
        return_value = process_type_interface_user_input();
        if (!return_value.success)
        {
            return return_value;
        }

        return_value = process_available_interfaces();
        if (!return_value.success)
        {
            return return_value;
        }

        return_value = process_all_interfaces();
        if (!return_value.success)
        {
            return return_value;
        }
    }
    else
    {
        // Process a specific interface selected by the user
        return_value = process_type_interface_user_input();
        if (!return_value.success)
        {
            return return_value;
        }

        return_value = process_available_interfaces_user_input();
        if (!return_value.success)
        {
            return return_value;
        }

        return_value = process_select_interface_user_input();
        if (!return_value.success)
        {
            return return_value;
        }
    }

    auto type_description_client_node = std::make_shared<TypeDescriptionClient>(shared_from_this_helper());
    return_value = type_description_client_node->start();
    if (!return_value.success)
    {
        return return_value;
    }

    // Process this interface to completion before ending the program
    rclcpp::spin_some(type_description_client_node);

    // Wait for the async operation to complete
    while (rclcpp::ok() && !type_description_client_node->is_complete())
    {
        rclcpp::spin_some(type_description_client_node);
        std::this_thread::sleep_for(std::chrono::milliseconds(Constants::SLEEP_MILLISECONDS));
    }

    return { true, utils::ReturnCode::OK, "Interfaces processed successfully" };
}