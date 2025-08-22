/**
 * @file asyncapi_generator.hpp
 * @author amparo.sancho-arellano (amparo.sancho-arellano@siemens.com)
 * @brief This class generate the asyncAPI document for a required interface
 * @version 0.1.0
 * @date 2025-08-20
 *
 * Copyright (c) Siemens AG 2024 All Rights Reserved.
 *
 */

#pragma once

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>

// custom header
#include "ros2_introspector/introspection_helper.hpp"
#include "utils/field_info.hpp"
#include "utils/logger.hpp"
#include "utils/return_value.hpp"

class AsyncAPIGenerator
{
public:
    /**
     * @brief Deleted default constructor to prevent instantiation without parameters
     *
     */
    AsyncAPIGenerator() = delete;

    /**
     * @brief Construct a new Async A P I Generator object
     *
     * @param introspection_helper Object of the Introspection Helper class to access its members
     * @param fields Fields of the Type Description class
     */
    AsyncAPIGenerator(std::shared_ptr<IntrospectionHelper> introspection_helper, std::vector<FieldInfo> fields) :
        introspection_helper_(introspection_helper),
        fields_(std::move(fields))
    {}

    /**
     * @brief Destroy the AsyncAPI Generator object
     *
     */
    ~AsyncAPIGenerator() = default;

    /**
     * @brief Function to generate or update the AsyncAPI 3.0 Head document
     *
     * @return utils::ReturnValue The return value of the update properties function
     */
    utils::ReturnValue generate_asyncapi();

private:
    /**
     * @brief Function to map ROS message types to AsyncAPI types
     *
     * @param[in] ros_type
     * @return std::string
     */
    std::string map_ros_type_to_asyncapi(const std::string & ros_type);

    /**
     * @brief Function to change the name from slashes to capital letters
     *
     * @param[in] type_name
     * @return std::string
     */
    std::string replace_undercores_with_pascal_case(const std::string & type_name);

    /**
     * @brief Struct representing AsyncAPI type information
     *
     */
    struct AsyncAPITypeInfo
    {
        std::string type;   // The AsyncAPI type (integer, string, etc.)
        std::string format; // The format specifier (int32, float, etc.)

        AsyncAPITypeInfo(const std::string & t, const std::string & f = "") :
            type(t),
            format(f)
        {}
    };

    /**
     * @brief Struct to map the ros2 type to the asyncAPI type
     *
     */
    const std::unordered_map<std::string, AsyncAPITypeInfo> type_mapping_{
        {     "bool", AsyncAPITypeInfo("boolean",   "boolean") },
        {     "int8", AsyncAPITypeInfo("integer",      "int8") },
        {    "uint8", AsyncAPITypeInfo("integer",     "uint8") },
        {    "int16", AsyncAPITypeInfo("integer",     "int16") },
        {   "uint16", AsyncAPITypeInfo("integer",    "uint16") },
        {    "int32", AsyncAPITypeInfo("integer",     "int32") },
        {   "uint32", AsyncAPITypeInfo("integer",    "uint32") },
        {    "int64", AsyncAPITypeInfo("integer",     "int64") },
        {   "uint64", AsyncAPITypeInfo("integer",    "uint64") },
        {  "float32",  AsyncAPITypeInfo("number",     "float") },
        {  "float64",  AsyncAPITypeInfo("number",    "double") },
        {   "string",  AsyncAPITypeInfo("string",    "string") },
        {     "time",  AsyncAPITypeInfo("string", "date-time") },
        { "duration",  AsyncAPITypeInfo("string",  "duration") },
        {   "byte[]",  AsyncAPITypeInfo("string",      "byte") },
        {   "char[]",  AsyncAPITypeInfo("string",    "string") }
    };

    /**
     * @brief Function to generate or update the AsyncAPI 3.0 Interface documents recursively
                (is not possible to use the members of the introspection_helper_ because of the nested types)
     *
     * @param[in] fields Struct to save the information of the interfaces
     * @param[in] type_interface The type of the interface (msg, srv, or action)
     * @param[in] type_name_with_capital_letters The name of the interface
     * @param[in] package_name The name of the package
     * @param[in] depth Counter of recursive executions of the function
     * @return utils::ReturnValue SUCESS if the properties were updated successfully, corresponding error otherwise
     */
    utils::ReturnValue update_properties(const std::vector<FieldInfo> & fields,
                                         const std::string & type_interface,
                                         const std::string & type_name_with_capital_letters,
                                         const std::string & package_name,
                                         const int depth = 0);

    std::shared_ptr<IntrospectionHelper>
        introspection_helper_;      // Class that handles the extraction of the information of the instrospection tool
    std::vector<FieldInfo> fields_; // Struct to save the information of the interfaces
    utils::Logger logger_ = utils::Logger("AsyncAPIGenerator_logger");
};
