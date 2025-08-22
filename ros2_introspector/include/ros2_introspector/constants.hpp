/**
 * @file constants.hpp
 * @author amparo.sancho-arellano (amparo.sancho-arellano@siemens.com)
 * @brief Definition of constants to use in the ros2_introspector files
 * @version 0.1.0
 * @date 2025-05-19
 * 
 * Copyright (c) Siemens AG 2024 All Rights Reserved.
 * 
 */

#pragma once

#include <filesystem>

namespace Constants {
    // Buffer, timing and limit constants
    constexpr size_t BUFFER_SIZE = 128;
    constexpr int SERVICE_TIMEOUT_SECONDS = 5;
    constexpr int SLEEP_MILLISECONDS = 100;
    constexpr int MAX_RECURSION_DEPTH = 100;

    // Data type constants
    namespace TypesId {
        constexpr int MESSAGE = 1;
        constexpr int INT8 = 2;
        constexpr int INT16 = 3;
        constexpr int INT32 = 4;
        constexpr int INT64 = 5;
        constexpr int UINT8 = 6;
        constexpr int UINT16 = 7;
        constexpr int UINT32 = 8;
        constexpr int UINT64 = 9;
        constexpr int FLOAT32 = 10;
        constexpr int FLOAT64 = 11;
        constexpr int STRING = 12;
    }

    // File paths
    namespace Paths {
        const std::filesystem::path OUTPUT_INTERFACE_DIR = "./output/InterfaceLibrary";
        const std::filesystem::path OUTPUT_DIR = "./output";
    }
}

