/**
 * @file return_value.hpp
 * @author amparo.sancho-arellano (amparo.sancho-arellano@siemens.com)
 * @brief Return type to give additional information back to user
 * @version 0.1.0
 * @date 2025-03-06
 *
 * Copyright (c) Siemens AG 2024 All Rights Reserved.
 *
 */
#pragma once

// std header
#include <cstdint>
#include <string>

namespace utils {

/**
 * @brief Return codes for all component functionalities that will be provided with corresponding messages
 *
 */
enum class ReturnCode : std::uint32_t
{
    // Operation was successful
    OK = 0,

    /*════════════════════════════════════╡ CLI ╞════════════════════════════════════*/

    // Command line error regarding provided arguments
    CLI_ARGUMENT_ERROR = 100,

    // Command line error regarding parsing of arguments
    CLI_PARSING_ERROR = 101,

    // Command line status if the user exits the program cleanly
    CLI_USER_INPUT_EXIT = 102,

    // Command line error if the user entered an invalid input
    CLI_USER_INPUT_ERROR = 103,

    // Poopen failed when running a command
    CLI_POPEN_ERROR = 104,

    /*════════════════════════════════════════════════════════════════════════════════*/

    // Error obtaining the hash or the type
    HASH_TYPE_ERROR = 200,

    /*════════════════════════════════════════════════════════════════════════════════*/

    // Print available interfaces error
    PRINT_INTERFACE_ERROR = 300,

    /*═════════════════════════════╡ AsyncAPI generator ╞═════════════════════════════*/

    // Error creating the output directory to save the generated asyncapi
    ASYNC_CREATE_FOLDER_ERROR = 400,

    // Error opening yaml file to save the generated asyncapi
    ASYNC_OPEN_FILE_ERROR = 401,

    // Maximum recursion depth reached when processing the asyncapi files
    ASYNC_RECURSION_LIMIT_REACHED = 402,

    /*════════════════════════════════════════════════════════════════════════════════*/

    // Error using the type description service
    TYPE_DES_SRV_ERROR = 500
};


/**
 * @brief Definition of the return value
 *
 */
struct ReturnValue
{
    // Whether the operation was successful
    bool success;

    // The corresponding return code definition
    ReturnCode return_code;

    // The corresponding message for the provided return code
    std::string message;
};

} // namespace utils
