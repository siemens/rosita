/**
 * @file logger.hpp
 * @author Manuel Kreutz (manuel.kreutz.ext@siemens.com)
 * @brief Logging functionalities for Introspection tool
 * @version 0.1.0
 * @date 2024-03-22
 *
 * Copyright (c) Siemens AG 2024 All Rights Reserved.
 *
 */
#pragma once

// std header
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>

// custom header
#include "spdlog/async.h"
#include "spdlog/sinks/ansicolor_sink.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/spdlog.h"


namespace utils {


/**
 * @brief Singleton based logger that can be used in the defined logging macros for command line or file logging
 *
 */
class Logger
{
public:
    /**
     * @brief Deleted default constructor
     *
     */
    Logger() = delete;

    /**
     * @brief Constructor to create and initialize a command line logger
     *
     * @param[in] name The name of the logger
     */
    explicit Logger(const std::string & name)
    {
        auto color_sink = std::make_shared<spdlog::sinks::ansicolor_stdout_sink_mt>();

        color_sink->set_color(spdlog::level::debug, color_sink->white);
        color_sink->set_color(spdlog::level::info, color_sink->white);
        color_sink->set_color(spdlog::level::warn, color_sink->yellow);
        color_sink->set_color(spdlog::level::err, color_sink->red);
        color_sink->set_color_mode(spdlog::color_mode::automatic);

        _logger = std::make_shared<spdlog::logger>(name, color_sink);


        // Set log pattern to logs looking like and are colorized: hello world
        // Aliases defined in: https://github.com/gabime/spdlog/wiki/3.-Custom-formatting
        _logger->set_pattern("%^%v%$");

        _logger->set_level(spdlog::level::info);

        // Set flush level to ensure immediate logging
        _logger->flush_on(spdlog::level::info);
    }

    /**
     * @brief Get the command line logger object
     *
     * @return The underlaying spdlog command line logger
     */
    [[nodiscard]] inline std::shared_ptr<spdlog::logger> getBaseLogger() const noexcept
    {
        return _logger;
    }
    /**
     * @brief Log the specified message combined with additional information e.g. the log level
     *
     * @param[in] logger The logger instance to be used
     * @param[in] log_level The log level at which the message should be logged
     * @param[in] file The file from which the log was called
     * @param[in] line The line of the calling function
     * @param[in] function The function name where the log was called
     * @param[in] args The variadic fmt based argument list
     */
    template<typename... Args>
    inline void log(std::shared_ptr<spdlog::logger> logger,
                    const spdlog::level::level_enum log_level,
                    [[maybe_unused]] const std::string & file,
                    [[maybe_unused]] const std::uint32_t line,
                    [[maybe_unused]] const std::string & function,
                    Args &&... args)
    {
        if (!logger)
        {
            return;
        }

        const auto format = fmt::format("{}", fmt::format(std::forward<Args>(args)...));

        switch (log_level)
        {
        case spdlog::level::trace:
            logger->trace(format);
            break;

        case spdlog::level::debug:
            logger->debug(format);
            break;

        case spdlog::level::info:
            logger->info(format);
            break;

        case spdlog::level::warn:
            logger->warn(format);
            break;

        case spdlog::level::err:
            logger->error(format);
            break;

        case spdlog::level::critical:
            logger->critical(format);
            break;

        case spdlog::level::off:
        case spdlog::level::n_levels:
            break;
        }
    }

private:
    // The spdlog logger instance
    std::shared_ptr<spdlog::logger> _logger{ nullptr };
};


} // namespace utils


// clang-format off
#define LOG_DEBUG(logger, ...) logger.log(logger.getBaseLogger(), spdlog::level::debug, static_cast<const char *>(__FILE__), __LINE__, static_cast<const char *>(__FUNCTION__), __VA_ARGS__) // NOLINT
#define LOG_INFO(logger, ...) logger.log(logger.getBaseLogger(), spdlog::level::info, static_cast<const char *>(__FILE__), __LINE__, static_cast<const char *>(__FUNCTION__), __VA_ARGS__) // NOLINT
#define LOG_WARN(logger, ...) logger.log(logger.getBaseLogger(), spdlog::level::warn, static_cast<const char *>(__FILE__), __LINE__, static_cast<const char *>(__FUNCTION__), __VA_ARGS__) // NOLINT
#define LOG_ERROR(logger, ...) logger.log(logger.getBaseLogger(), spdlog::level::err, static_cast<const char *>(__FILE__), __LINE__, static_cast<const char *>(__FUNCTION__), __VA_ARGS__) // NOLINT
// clang-format on