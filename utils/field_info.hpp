/**
 * @file field_info.hpp
 * @author amparo.sancho-arellano (amparo.sancho-arellano@siemens.com)
 * @brief Definition of the struct to save the information of the interfaces
 * @version 0.1.0
 * @date 2025-08-20
 * 
 * Copyright (c) Siemens AG 2024 All Rights Reserved.
 * 
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

/**
 * @brief Structure to hold field information
 *
 */
struct FieldInfo
{
    std::string name;
    std::string type;
    std::vector<FieldInfo> nested_fields; // For nested types
    std::string type_name_nested_fields;  // For nested types
};