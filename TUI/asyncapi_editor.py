"""asyncapi_editor.py

This script creates the Terminal UI for the AsyncAPI ROS2 Specification Generator.
"""

__author__ = "Amparo Sancho Arellano (amparo.sancho-arellano@siemens.com)"
__version__ = "0.1.0"
__date__ = "2025-08-20"
__copyright__ = "Copyright (c) Siemens AG 2024 All Rights Reserved."

import os
import yaml
import subprocess
import time
import curses
import ui_utils
from log import get_logger, log_file

logger = get_logger("AsyncAPIEditor")


class AsyncAPIEditor:
    """A curses-based AsyncAPI Editor."""

    def __init__(self, asyncapi_file, emojis):
        self.asyncapi_file = asyncapi_file
        self.emojis = emojis
        self.asyncapi_definition = (
            {}
        )  # Empty dictionary to store the AsyncAPI specification
        self.selected_index = 0  # Index of the currently selected channel in the UI
        self.channels = []  # List of channel addresses to display
        self.top_line = 0  # First visible line in the scrollable view
        self.running = True  # Boolean to control the main loop
        self.default_ros2_path = (
            "/opt/ros/jazzy/share/"  # Default tors 2 path in case none is configured
        )
        self.ros2_path = ""  # Will store the ROS2 path
        self.ros2_path_configured = False  # Track if the ROS2 path is configured

        # Variables to display status messages
        self.message = ""
        self.message_type = ""  # "info", "error", "success"
        self.message_timeout = 0

        # Load the path from configuration immediately after initialization
        self.load_initial_path()

    def check_ros2_path(self):
        """Check if ROS2 path is set and valid.

        Returns:
            tuple: A tuple containing:
                - bool: True if the path is valid and accessible, False otherwise.
                - str: The valid path or an error message if invalid.
        """

        # Check if path is set
        if not self.ros2_path:
            logger.info(f"ROS2 path not set, using default: {self.default_ros2_path}")
            self.ros2_path = self.default_ros2_path

        # Check if the path exists
        if not os.path.exists(self.ros2_path):
            logger.error(f"ROS 2 path directory does not exist: {self.ros2_path}")
            return False, f"ROS 2 path directory does not exist: {self.ros2_path}"

        # Check if the path is readable
        if not os.access(self.ros2_path, os.R_OK):
            logger.error(f"ROS 2 path directory is not readable: {self.ros2_path}")
            return False, f"ROS 2 path directory is not readable: {self.ros2_path}"

        logger.info(f"ROS2 path is valid: {self.ros2_path}")

        # Save the configuration after setting a valid path
        self.save_configuration()

        return True, self.ros2_path

    def set_ros2_path(self, path):
        """Set the ROS2 path as an instance attribute"""
        
        # If path is empty or None, use default
        if not path:
            self.ros2_path = self.default_ros2_path
        else:
            self.ros2_path = path
        logger.info(f"Set ROS 2 path to: {path}")

        return self.check_ros2_path()

    def load_asyncapi_definition(self):
        """Load the AsyncAPI definition from file.

        This method performs the following steps:
        1. Determines the path to the AsyncAPI YAML file.
        2. Checks if the file exists.
        3. If it exists, loads the YAML data into `self.asyncapi_definition`.
        4. Handles various errors that might occur during loading.
        """
        
        try:
            logger.info(f"Loading AsyncAPI definition from: {self.asyncapi_file}")

            if not os.path.exists(self.asyncapi_file):
                logger.warning(f"AsyncAPI file does not exist: {self.asyncapi_file}")
                self.asyncapi_definition = {}
                return

            # Safe YAML data in the asyncapi_definition dictionary
            with open(self.asyncapi_file, "r") as file:
                data = yaml.safe_load(file)
                logger.info(
                    f"Loaded AsyncAPI definition with {len(data.get('channels', {}))} channels"
                )
                self.asyncapi_definition = data

        except FileNotFoundError:
            logger.error(f"AsyncAPI definition file not found: {self.asyncapi_file}")
            self.show_message("AsyncAPI definition file not found.", "error")
            self.asyncapi_definition = {}

        except yaml.YAMLError as e:
            logger.error(f"Invalid AsyncAPI definition file: {e}")
            self.show_message(f"Invalid AsyncAPI definition file: {e}", "error")
            self.asyncapi_definition = {}

        except Exception as e:
            logger.exception(f"Error loading AsyncAPI definition: {e}")
            self.show_message(f"Error loading AsyncAPI definition: {e}", "error")
            self.asyncapi_definition = {}

    def save_asyncapi_definition(self):
        """Save the AsyncAPI definition to file."""
        
        try:
            logger.info(f"Saving AsyncAPI definition to: {self.asyncapi_file}")

            with open(self.asyncapi_file, "w") as file:
                yaml.dump(self.asyncapi_definition, file, default_flow_style=False)
            logger.info("AsyncAPI definition saved successfully")
            return True
        except Exception as e:
            logger.exception(f"Failed to save AsyncAPI definition: {e}")
            self.show_message(f"Failed to save AsyncAPI definition: {e}", "error")
            return False

    def get_unique_channel_addresses(self):
        """Get the unique channel addresses from the AsyncAPI definition.

        This method performs the following steps:
        1. Creates an empty set to store unique addresses.
        2. Iterates through all channels in the AsyncAPI definition.
        3. Adds each unique address to the set.

        Returns:
            set: A set of unique channel addresses.
        """
        
        channel_addresses = set()
        for channel, details in self.asyncapi_definition.get("channels", {}).items():
            if "address" in details and details["address"] not in channel_addresses:
                channel_addresses.add(details["address"])
        return channel_addresses

    def update_channels_list(self):
        """Update the channels list with the current AsyncAPI definition."""
        
        channel_addresses = self.get_unique_channel_addresses()
        if channel_addresses:
            self.channels = sorted(list(channel_addresses))
        else:
            self.channels = ["No channels found"]

        # Reset selection
        self.selected_index = 0
        self.top_line = 0

    def run_introspection_tool(
        self, mode="all", interface_type=None, selected_interface=None
    ):
        """Run the ROS2 introspection tool to generate AsyncAPI definition.

        Args:
            mode (str): "all" to get all interfaces, "select" to select a specific interface.
            interface_type (str, optional): Type of interface ("msg", "srv", "action")
                                            when mode is "select". Defaults to None.
            selected_interface (str, optional): Name of the selected interface when
                                                mode is "select". Defaults to None.

        Returns:
            bool: True if the introspection tool ran successfully, False otherwise.
        """
        
        try:
            # Check if user_ros2_path is set
            self.ros2_path_configured, message = self.check_ros2_path()
            if not self.ros2_path_configured:
                logger.error(f"Cannot run introspection tool: {message}")
                return False

            logger.info(f"Starting ROS2 introspection tool in {mode} mode")

            # Build command with ros2 parameters
            cmd = ["ros2", "run", "ros2_introspector", "ros2_introspector_node"]

            # Add parameters using ros2 args syntax
            cmd.extend(["--ros-args", "--param", f"mode:={mode}"])

            if mode == "select" and interface_type:
                cmd.extend(["--param", f"interface_type:={interface_type}"])

            if mode == "select" and selected_interface:
                cmd.extend(["--param", f"selected_interface:={selected_interface}"])

            # Add ROS2 path parameter using the instance attribute
            if self.ros2_path:
                cmd.extend(["--param", f"ros2_path:={self.ros2_path}"])

            # Run the command
            logger.info(f"Running command: {' '.join(cmd)}")
            process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
            )

            # Wait for the process to complete
            stdout, stderr = process.communicate()

            # Log the output
            logger.info(f"Process exit code: {process.returncode}")
            if stdout:
                logger.info(f"Process stdout: {stdout}")

            if process.returncode != 0:
                logger.error(f"Process failed with exit code {process.returncode}")
                return False

            logger.info("ROS2 introspection tool completed successfully")
            return True
        except Exception as e:
            logger.exception(f"Exception while running introspection tool: {e}")
            return False

    def verify_asyncapi_generation(self):
        """Verify that the AsyncAPI file was generated correctly."""
        
        try:
            # Check if file exists
            if not os.path.exists(self.asyncapi_file):
                logger.error(f"AsyncAPI file not found: {self.asyncapi_file}")
                return False

            # Check if file was recently modified (within the last minute)
            file_mod_time = os.path.getmtime(self.asyncapi_file)
            current_time = time.time()
            if current_time - file_mod_time > 60:  # 60 seconds = 1 minute
                logger.warning(
                    f"AsyncAPI file exists but wasn't recently modified: {self.asyncapi_file}"
                )
                logger.warning(f"File modification time: {time.ctime(file_mod_time)}")
                logger.warning(f"Current time: {time.ctime(current_time)}")

            # Check if file has valid content
            with open(self.asyncapi_file, "r") as file:
                content = yaml.safe_load(file)
                if not content or "channels" not in content:
                    logger.error(
                        f"AsyncAPI file exists but has invalid content: {self.asyncapi_file}"
                    )
                    return False

                logger.info(
                    f"AsyncAPI file validated successfully with {len(content.get('channels', {}))} channels"
                )
                return True
        except Exception as e:
            logger.exception(f"Exception while verifying AsyncAPI generation: {e}")
            return False

    def check_ros2_environment(self):
        """Check if ROS2 environment is properly set up."""
        
        try:
            # Check ROS_DISTRO
            ros_distro = os.environ.get("ROS_DISTRO", "Not set")
            logger.info(f"ROS_DISTRO: {ros_distro}")

            # Check ROS_DOMAIN_ID
            ros_domain_id = os.environ.get("ROS_DOMAIN_ID", "Not set")
            logger.info(f"ROS_DOMAIN_ID: {ros_domain_id}")

            # Check USER_ROS2_PATH
            user_ros2_path = os.environ.get("USER_ROS2_PATH", "Not set")
            logger.info(f"USER_ROS2_PATH: {user_ros2_path}")

            # Check if ros2_introspector package is available
            cmd = ["ros2", "pkg", "list"]
            process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
            )
            stdout, stderr = process.communicate()

            if "ros2_introspector" in stdout:
                logger.info("ros2_introspector package is available")
            else:
                logger.error(
                    "ros2_introspector package is NOT available in ros2 pkg list"
                )

            # Check if the node is executable
            cmd = ["ros2", "pkg", "executables", "ros2_introspector"]
            process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
            )
            stdout, stderr = process.communicate()

            if "ros2_introspector ros2_introspector_node" in stdout:
                logger.info("ros2_introspector_node executable is available")
            else:
                logger.error("ros2_introspector_node executable is NOT available")
                logger.info(f"Available executables: {stdout}")
                if stderr:
                    logger.error(f"Error: {stderr}")

            return True
        except Exception as e:
            logger.exception(f"Exception while checking ROS2 environment: {e}")
            return False

    def run_manual_command(self):
        """Run a manual ROS2 command for debugging."""
        
        try:
            cmd = ["ros2", "topic", "list"]
            logger.info(f"Running manual command: {' '.join(cmd)}")

            process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
            )
            stdout, stderr = process.communicate()

            if stderr:
                logger.error(f"Error running manual command: {stderr}")
                return False, stderr
            else:
                logger.info(f"Manual command output: {stdout}")
                return True, stdout
        except Exception as e:
            logger.exception(f"Exception running manual command: {e}")
            return False, str(e)

    def get_available_interfaces(self, interface_type):
        """Get a list of available interfaces of the specified type."""
        
        try:
            cmd = None
            if interface_type == "msg":
                cmd = ["ros2", "topic", "list"]
            elif interface_type == "srv":
                cmd = ["ros2", "service", "list"]
            elif interface_type == "action":
                cmd = ["ros2", "action", "list"]
            else:
                return []

            process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
            )
            stdout, stderr = process.communicate()

            if stderr:
                logger.error(
                    f"Error getting available {interface_type} interfaces: {stderr}"
                )
                return []

            # Parse the output into a list of interfaces
            interfaces = [line.strip() for line in stdout.split("\n") if line.strip()]

            # Filter out internal ROS interfaces if needed
            if interface_type == "msg":
                interfaces = [
                    i for i in interfaces if i not in ["/rosout", "/parameter_events"]
                ]

            # Filter out services from introspection nodes
            elif interface_type == "srv":
                interfaces = [
                    i
                    for i in interfaces
                    if not (
                        i.startswith("/type_description_client/")
                        or i.startswith("/ros2_introspector_node/")
                    )
                ]

            return interfaces
        except Exception as e:
            logger.exception(
                f"Exception getting available {interface_type} interfaces: {e}"
            )
            return []

    def show_message(self, message, message_type="info"):
        """Show a message in the UI."""
        
        self.message = message
        self.message_type = message_type
        self.message_timeout = time.time() + 5  # Show for 5 seconds

    def show_ros2_path_dialog(self, stdscr):
        """Show a dialog to set the ROS2 path."""
        
        # Use current instance attribute as default, or the standard default if not set
        current_path = self.ros2_path if self.ros2_path else self.default_ros2_path

        # Show input dialog
        new_path = ui_utils.show_input_dialog(
            stdscr,
            f"{self.emojis['path']} ROS2 Path Configuration",
            "Enter the path to your ROS2 installation:",
            current_path,
        )

        if new_path is None:
            # If user cancels and no path is set, use default
            if not self.ros2_path:
                self.set_ros2_path(self.default_ros2_path)
            return False  # Indicate user cancelledd

        # Set the new path
        success, message = self.set_ros2_path(new_path)

        if success:
            self.show_message(f"ROS 2 path set to: {new_path}", "success")
            self.ros2_path_configured = True
            return True
        else:
            ui_utils.show_notification(
                stdscr,
                f"Failed to set ROS 2 path: {message}\n\nPlease try again with a valid path.",
                "Error",
            )
            return False

    def show_debug_screen(self, stdscr):
        """Show debug information in a scrollable screen view."""
        
        try:
            # Save current state
            curses.curs_set(0)  # Hide cursor

            # Check ROS2 environment
            self.check_ros2_environment()

            # Run a manual ROS2 command to check if ROS2 is working
            success, output = self.run_manual_command()

            # Read the last few lines of the log file
            try:
                with open(log_file, "r") as f:
                    log_lines = f.readlines()
                    # Show last 10 lines
                    log_content = "".join(log_lines[-10:])
            except:
                log_content = "Could not read log file"

            # Check USER_ROS2_PATH
            self.ros2_path_configured, ros2_path_msg = self.check_ros2_path()

            # Prepare debug information
            message = (
                f"ROS_DISTRO: {os.environ.get('ROS_DISTRO', 'Not set')}\n"
                f"ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', 'Not set')}\n"
                f"USER_ROS2_PATH: {os.environ.get('USER_ROS2_PATH', 'Not set')} "
                f"({'Valid' if self.ros2_path_configured else 'Invalid: ' + ros2_path_msg})\n"
                f"Script directory: {os.path.dirname(os.path.abspath(__file__))}\n"
                f"Log file: {os.path.abspath(log_file)}\n\n"
                f"ROS2 topic list test: {'Success' if success else 'Failed'}\n"
                f"Topics found: {output if success else 'None'}\n\n"
                f"Recent log entries:\n{log_content}"
            )

            # Split message into lines
            lines = message.split("\n")

            # Clear screen
            stdscr.clear()
            height, width = stdscr.getmaxyx()

            # Draw title
            title = f"{self.emojis['debug']} Debug Information"
            title_x = max(0, (width - len(title)) // 2)
            ui_utils.safe_addstr(stdscr, 0, title_x, title)

            # Set up scrolling
            scroll_pos = 0
            max_visible = height - 4  # Leave space for title and footer
            max_scroll = max(0, len(lines) - max_visible)

            # Draw footer with instructions
            footer = "Up/Down: Scroll | Page Up/Down: Page | Home/End: Jump | P: Set Path | Q: Return"
            footer_x = max(0, (width - len(footer)) // 2)

            # Function to redraw content
            def redraw_content():
                # Clear content area
                for i in range(2, height - 2):
                    ui_utils.safe_addstr(stdscr, i, 0, " " * width)

                # Draw message
                visible_lines = lines[scroll_pos : scroll_pos + max_visible]
                for i, line in enumerate(visible_lines):
                    if i + 2 < height - 2:  # Leave space for title and footer
                        ui_utils.safe_addstr(stdscr, i + 2, 2, line[: width - 4])

                # Draw scroll indicators
                if scroll_pos > 0:
                    ui_utils.safe_addstr(stdscr, 2, width - 3, "^")
                if scroll_pos < max_scroll:
                    ui_utils.safe_addstr(stdscr, height - 3, width - 3, "v")

                # Draw footer
                ui_utils.safe_addstr(stdscr, height - 1, footer_x, footer)

                stdscr.refresh()

            # Initial draw
            redraw_content()

            # Handle keys for scrolling
            while True:
                key = stdscr.getch()

                if key in [ord("q"), ord("Q")]:
                    break
                elif key in [ord("p"), ord("P")]:
                    # Show dialog to set USER_ROS2_PATH
                    self.show_ros2_path_dialog(stdscr)

                    # Refresh debug information
                    self.ros2_path_configured, ros2_path_msg = self.check_ros2_path()
                    message = (
                        f"ROS_DISTRO: {os.environ.get('ROS_DISTRO', 'Not set')}\n"
                        f"ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', 'Not set')}\n"
                        f"USER_ROS2_PATH: {os.environ.get('USER_ROS2_PATH', 'Not set')} "
                        f"({'Valid' if self.ros2_path_configured else 'Invalid: ' + ros2_path_msg})\n"
                        f"Script directory: {os.path.dirname(os.path.abspath(__file__))}\n"
                        f"Log file: {os.path.abspath(log_file)}\n\n"
                        f"ROS2 topic list test: {'Success' if success else 'Failed'}\n"
                        f"Topics found: {output if success else 'None'}\n\n"
                        f"Recent log entries:\n{log_content}"
                    )
                    lines = message.split("\n")
                    max_scroll = max(0, len(lines) - max_visible)
                    redraw_content()
                elif key == curses.KEY_UP:
                    if scroll_pos > 0:
                        scroll_pos -= 1
                        redraw_content()
                elif key == curses.KEY_DOWN:
                    if scroll_pos < max_scroll:
                        scroll_pos += 1
                        redraw_content()
                elif key == curses.KEY_PPAGE:  # Page Up
                    scroll_pos = max(0, scroll_pos - max_visible)
                    redraw_content()
                elif key == curses.KEY_NPAGE:  # Page Down
                    scroll_pos = min(max_scroll, scroll_pos + max_visible)
                    redraw_content()
                elif key == curses.KEY_HOME:
                    scroll_pos = 0
                    redraw_content()
                elif key == curses.KEY_END:
                    scroll_pos = max_scroll
                    redraw_content()

        except Exception as e:
            logger.exception(f"Error in debug screen: {e}")
            # Try to show error
            try:
                stdscr.clear()
                ui_utils.safe_addstr(stdscr, 0, 0, f"Error in debug screen: {e}")
                ui_utils.safe_addstr(stdscr, 1, 0, "Press any key to continue...")
                stdscr.refresh()
                stdscr.getch()
            except:
                pass

    def handle_key(self, stdscr, key):
        """Handle key press."""
        
        try:
            if key == curses.KEY_UP:
                if self.selected_index > 0:
                    self.selected_index -= 1
                    if self.selected_index < self.top_line:
                        self.top_line = self.selected_index

            elif key == curses.KEY_DOWN:
                if self.selected_index < len(self.channels) - 1:
                    self.selected_index += 1
                    height, _ = stdscr.getmaxyx()
                    max_visible = height - 7  # Adjust based on UI layout
                    if self.selected_index >= self.top_line + max_visible:
                        self.top_line = self.selected_index - max_visible + 1

            elif key in [ord("d"), ord("D")]:
                self.delete_channel(stdscr)

            elif key in [ord("r"), ord("R")]:
                # Check if ROS 2 path is set before refreshing
                if self.ros2_path_configured:
                    self.refresh_data(stdscr)
                else:
                    ui_utils.show_notification(
                        stdscr,
                        f"ROS 2 path is not set correctly. \n\nPlease configure it before refreshing.",
                        "Error",
                    )

            elif key in [ord("i"), ord("I")]:
                self.show_debug_info(stdscr)

            elif key in [ord("p"), ord("P")]:
                self.show_ros2_path_dialog(stdscr)

            elif key in [ord("q"), ord("Q")]:
                self.running = False

        except Exception as e:
            logger.exception(f"Error handling key: {e}")

    def delete_channel(self, stdscr):
        """Delete the selected channel."""
        
        if not self.channels or self.channels[0] == "No channels found" or self.channels[0] == "No channels yet, press R":
            self.show_message("No channel selected or no channels available.", "error")
            return

        selected_channel = self.channels[self.selected_index]

        # Confirm deletion
        confirmed = ui_utils.show_confirmation(
            stdscr, f"{self.emojis['warning']}  Delete channel '{selected_channel}'?"
        )

        if confirmed:
            # Find all channels with the selected address and remove them
            channels_to_delete = []
            for channel, details in self.asyncapi_definition["channels"].items():
                if details.get("address") == selected_channel:
                    channels_to_delete.append(channel)

            # Delete the channels
            for channel in channels_to_delete:
                del self.asyncapi_definition["channels"][channel]

            # Delete associated operations
            operations_to_delete = []
            for operation, op_details in self.asyncapi_definition["operations"].items():
                channel_ref = op_details.get("channel", {}).get("$ref", "")
                if channel_ref.split("/")[-1] in channels_to_delete:
                    operations_to_delete.append(operation)
                elif "reply" in op_details and "channel" in op_details["reply"]:
                    reply_ref = op_details["reply"]["channel"].get("$ref", "")
                    if reply_ref.split("/")[-1] in channels_to_delete:
                        operations_to_delete.append(operation)

            # Delete the operations
            for operation in operations_to_delete:
                del self.asyncapi_definition["operations"][operation]

            # Save the updated AsyncAPI definition
            if self.save_asyncapi_definition():
                self.show_message(
                    f"Channel '{selected_channel}' and {len(operations_to_delete)} associated operations deleted successfully.",
                    "success",
                )

                # Update the list of channel addresses
                self.update_channels_list()

    def refresh_data(self, stdscr):
        """Refresh the AsyncAPI definition."""
        
        # Show mode selection dialog
        mode = ui_utils.show_mode_selection(stdscr)

        if mode is None:
            return  # User cancelled

        if mode == "all":
            # Confirm refresh for all interfaces
            confirmed = ui_utils.show_confirmation(
                stdscr,
                f"{self.emojis['all']}  Generate AsyncAPI for all available interfaces?",
            )

            if confirmed:
                # Show loading dialog
                loading_dialog = ui_utils.show_loading(
                    stdscr, "Running introspection tool for all interfaces..."
                )

                # Check ROS2 environment
                self.check_ros2_environment()

                # Run the introspection tool for all interfaces
                success = self.run_introspection_tool(mode="all")

                # Verify the file was generated
                verification = self.verify_asyncapi_generation()

                # Clear loading dialog
                loading_dialog.clear()
                loading_dialog.refresh()

                if success and verification:
                    # Reload the AsyncAPI definition
                    self.load_asyncapi_definition()

                    # Update the channel list
                    self.update_channels_list()

                    # Show success message
                    channel_count = len(self.get_unique_channel_addresses())
                    self.show_message(
                        f"AsyncAPI definition refreshed successfully. Found {channel_count} unique channel addresses.",
                        "success",
                    )
                else:
                    # Show error message
                    error_message = "Failed to refresh AsyncAPI definition."
                    if not success:
                        error_message += (
                            "\nThe introspection tool did not complete successfully."
                        )
                    if not verification:
                        error_message += (
                            "\nThe AsyncAPI file was not generated correctly."
                        )
                    error_message += "\nCheck the log file for more details."

                    ui_utils.show_notification(stdscr, error_message, "Error")

        elif mode == "select":
            # Gather interfaces of each type
            interfaces_by_type = {
                'msg': self.get_available_interfaces("msg"),
                'srv': self.get_available_interfaces("srv"),
                'action': self.get_available_interfaces("action")
            }
            
            # Show multi-interface selection dialog
            selected_interfaces = ui_utils.show_multi_interface_selection(stdscr, interfaces_by_type, self.emojis)

            if selected_interfaces is None or not selected_interfaces:
                self.show_message("No interfaces found.", "error")
                return  # User cancelled or no interfaces selected

            # Confirm refresh for selected interfaces
            interface_count = len(selected_interfaces)
            confirmed = ui_utils.show_confirmation(
                stdscr,
                f"{self.emojis['select']}  Generate AsyncAPI for {interface_count} selected interfaces?",
            )

            if confirmed:
                # Show loading dialog
                loading_dialog = ui_utils.show_loading(
                    stdscr,
                    f"Running introspection tool for {interface_count} interfaces...",
                )

                # Check ROS2 environment
                self.check_ros2_environment()

                # Process each selected interface
                all_success = True
                for interface_type, interface_name in selected_interfaces:
                    # Show progress
                    loading_dialog.clear()
                    loading_dialog.box()
                    ui_utils.safe_addstr(
                        loading_dialog,
                        1,
                        2,
                        f"Processing {interface_type} '{interface_name}'...",
                    )
                    ui_utils.safe_addstr(
                        loading_dialog,
                        2,
                        2,
                        f"Interface {selected_interfaces.index((interface_type, interface_name)) + 1} of {interface_count}",
                    )
                    loading_dialog.refresh()

                    # Run the introspection tool for the selected interface
                    success = self.run_introspection_tool(
                        mode="select",
                        interface_type=interface_type,
                        selected_interface=interface_name,
                    )

                    if not success:
                        all_success = False
                        logger.error(
                            f"Failed to process {interface_type} '{interface_name}'"
                        )

                # Verify the file was generated
                verification = self.verify_asyncapi_generation()

                # Clear loading dialog
                loading_dialog.clear()
                loading_dialog.refresh()

                if all_success and verification:
                    # Reload the AsyncAPI definition
                    self.load_asyncapi_definition()

                    # Update the channel list
                    self.update_channels_list()

                    # Show success message
                    self.show_message(
                        f"AsyncAPI definition generated successfully for {interface_count} interfaces.",
                        "success",
                    )
                else:
                    # Show error message
                    error_message = (
                        f"Failed to generate AsyncAPI definition for some interfaces."
                    )
                    if not all_success:
                        error_message += "\nSome interfaces could not be processed."
                    if not verification:
                        error_message += (
                            "\nThe AsyncAPI file was not generated correctly."
                        )
                    error_message += "\nCheck the log file for more details."

                    ui_utils.show_notification(stdscr, error_message, "Error")

    def show_debug_info(self, stdscr):
        """Show debug information."""
        
        # Use the scrollable debug screen
        self.show_debug_screen(stdscr)

        # Redraw the main UI after returning from debug screen
        ui_utils.draw_ui(stdscr, self)

    def run(self, stdscr):
        """Run the AsyncAPI Editor's main loop.

        This method:
        1. Sets up the curses environment (hides cursor, enables special keys).
        2. Checks if the ROS2 path is configured, and prompts to set it if not.
        3. Loads the existing AsyncAPI definition from file.
        4. Updates the list of channels to display.
        5. Enters the main loop that:
            - Draws the UI.
            - Gets key input.
            - Handles key presses.
        6. Catches and displays any errors.

        Args:
            stdscr (curses.window): The curses window object, representing the screen.
        """
        
        try:
            # Set up curses
            curses.curs_set(0)  # Hide cursor
            stdscr.keypad(True)  # Enable keypad mode for special keys

            # Check if ROS 2 path is set
            self.ros2_path_configured, message = self.check_ros2_path()

            # If ROS 2 path is not set, show dialog to set it
            if not self.ros2_path_configured:
                ui_utils.show_notification(
                    stdscr,
                    f"ROS 2 path environment variable is not set correctly:\n{message}\n\n"
                    f"This path is required for the AsyncAPI generator to work properly.\n"
                    f"Please configure it in the next dialog.",
                    "ROS2 Path Configuration Required",
                )
                self.show_ros2_path_dialog(stdscr)

            # Load AsyncAPI definition
            self.load_asyncapi_definition()

            # Update channels list
            self.update_channels_list()

            # Add instructions if no channel is found
            if not self.channels or (
                len(self.channels) == 1 and self.channels[0] == "No channels found"
            ):
                logger.debug("No channels found, wait until the user press R")
                self.channels = ["No channels yet, press R"]

            # Main loop
            while self.running:
                ui_utils.draw_ui(stdscr, self)

                # Get key input
                try:
                    key = stdscr.getch()
                    self.handle_key(stdscr, key)
                except KeyboardInterrupt:
                    self.running = False

        except Exception as e:
            logger.exception(f"Error in main run loop: {e}")
            # Try to show error message
            try:
                stdscr.clear()
                ui_utils.safe_addstr(stdscr, 0, 0, f"Error: {e}")
                ui_utils.safe_addstr(stdscr, 1, 0, "Press any key to exit")
                stdscr.refresh()
                stdscr.getch()
            except:
                pass  # If even this fails, just exit

    def save_configuration(self):
        """Save the current ROS 2 installation path to a YAML file in the script's directory."""
        
        try:
            # Get the directory where the script is located
            script_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(script_dir, "ros2_path_config.yaml")

            # Create config with comments
            with open(config_path, "w") as f:
                # Add some helpful comments at the top
                f.write(
                    "# ROS 2 installation path to be used by the AsyncAPI Introspection TUI\n"
                )
                f.write(
                    "# This file is automatically generated and updated by the AsyncAPI Introspection TUI\n\n"
                )

                # Write the path with a comment
                f.write("# Path to your ROS2 installation\n")
                f.write(f"ros2_path: {self.ros2_path}\n")

            logger.info(f"Configuration saved to {config_path}")

        except Exception as e:
            logger.exception(f"Error saving configuration: {e}")

    def load_initial_path(self):
        """Load the ROS 2 path from a YAML configuration file in the script's directory or use default."""
        
        try:
            # Get the directory where the script is located
            script_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(script_dir, "ros2_path_config.yaml")

            if os.path.exists(config_path):
                with open(config_path, "r") as f:
                    config = yaml.safe_load(f)
                    if config and "ros2_path" in config and config["ros2_path"]:
                        path = config["ros2_path"]
                        if os.path.exists(path):
                            self.ros2_path = path
                            logger.info(f"Loaded ROS 2 path from config: {path}")
                            return
                        else:
                            logger.warning(f"Saved ROS 2 path no longer exists: {path}")

            if os.path.exists(self.default_ros2_path) and os.path.isdir(
                self.default_ros2_path
            ):
                self.ros2_path = self.default_ros2_path
                logger.info(f"Using default ROS2 path: {self.default_ros2_path}")
                # Save this default path for future sessions
                self.save_configuration()
            else:
                logger.warning(f"Default ROS2 path not found: {self.default_ros2_path}")
                self.ros2_path = ""  # Empty path will trigger a prompt later

        except Exception as e:
            logger.exception(f"Error loading initial ROS2 path: {e}")
