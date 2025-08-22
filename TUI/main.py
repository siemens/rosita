"""asyncapi_editor.py

This script is the entry point for the Terminal UI for the AsyncAPI ROS 2 Specification Generator
"""

__author__ = "Amparo Sancho Arellano (amparo.sancho-arellano@siemens.com)"
__version__ = "0.1.0"
__date__ = "2025-08-20"
__copyright__ = "Copyright (c) Siemens AG 2024 All Rights Reserved."

import os
import curses
from asyncapi_editor import AsyncAPIEditor
from log import get_logger, log_file

# Get a logger for this module
logger = get_logger("main")

# Specify asyncapi header file location
script_dir = os.path.dirname(os.path.abspath(__file__))
asyncapi_file = os.path.join(script_dir, "../output/Head-asyncapi.yaml")


EMOJIS = {
    "ros": "🤖",  # Robot face for ROS
    "topic": "📡",  # Satellite antenna for topics/communication
    "service": "🔄",  # Revolving arrows for services (request/response)
    "action": "🎯",  # Target for actions (goal-oriented)
    "success": "✅",  # Check mark for success
    "error": "❌",  # Cross mark for errors
    "warning": "⚠️",  # Warning sign
    "info": "ℹ️",  # Information
    "delete": "🗑️",  # Wastebasket for deletion
    "select": "✏️",  # Pencil for editing
    "refresh": "🔄",  # Revolving arrows for refresh
    "save": "💾",  # Floppy disk for saving
    "exit": "🚪",  # Door for exit
    "debug": "🐞",  # Bug for debugging
    "asyncapi": "📋",  # Clipboard for AsyncAPI
    "path": "📁",  # Folder for path definition
    "all": "📚",  # Books for all selection
}


def main():
    """Main function."""
    try:
        editor = AsyncAPIEditor(
            asyncapi_file, EMOJIS
        )  # Create an instance of the AsyncAPIEditor class
        curses.wrapper(
            editor.run
        )  # Initialize the TUI environment and call the run method
    except Exception as e:
        logger.exception(f"Unhandled exception: {e}")
        print(f"Error: {e}")
        print("Check asyncapi_editor.log for details")


if __name__ == "__main__":
    main()
