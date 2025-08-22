##
# @file ui_utils.py
# @author Amparo Sancho Arellano (amparo.sancho-arellano@siemens.com)
# @brief UI utility functions for the AsyncAPI TUI
# @version 0.1.0
# @date 2025-05-20
#
# Copyright (c) Siemens AG 2024 All Rights Reserved.
#
##

import curses
import time
from log import get_logger

logger = get_logger("ui_utils")

def safe_addstr(window, y, x, text, attr=0):
    """Safely add a string to a window, handling any errors."""
    try:
        window.addstr(y, x, text, attr)
    except curses.error:
        # Handle the case where the string doesn't fit or other curses errors
        try:
            # Try to add a truncated version if at a valid position
            max_y, max_x = window.getmaxyx()
            if 0 <= y < max_y and 0 <= x < max_x:
                # Calculate available space
                available_space = max_x - x - 1
                if available_space > 0:
                    window.addstr(y, x, text[:available_space], attr)
        except curses.error:
            # If even that fails, just log it and continue
            logger.debug(f"Failed to add string at ({y},{x}): {text}")
            pass
        
def show_confirmation(stdscr, message):
    """Show a confirmation dialog and return True if confirmed."""
    height, width = stdscr.getmaxyx()

    # Calculate dialog dimensions
    dialog_height = 6
    dialog_width = min(max(len(message) + 4, 40), width - 4)
    dialog_y = (height - dialog_height) // 2
    dialog_x = (width - dialog_width) // 2

    # Create dialog window
    dialog = curses.newwin(dialog_height, dialog_width, dialog_y, dialog_x)
    dialog.box()

    # Show message
    safe_addstr(dialog, 1, 2, message)
    safe_addstr(dialog, 3, 2, "Yes (Y) / No (N)")
    dialog.refresh()

    # Save current state
    curses.curs_set(0)  # Hide cursor

    # Wait for input
    while True:
        key = stdscr.getch()
        if key in [ord("y"), ord("Y")]:
            return True
        elif key in [ord("n"), ord("N")]:
            return False
        
def show_input_dialog(stdscr, title, prompt, default_value=""):
    """Show an input dialog and return the entered value."""
    height, width = stdscr.getmaxyx()

    # Calculate dialog dimensions
    dialog_height = 7
    dialog_width = min(max(len(prompt) + 4, 60), width - 4)
    dialog_y = (height - dialog_height) // 2
    dialog_x = (width - dialog_width) // 2

    # Create dialog window
    dialog = curses.newwin(dialog_height, dialog_width, dialog_y, dialog_x)
    dialog.box()

    # Show title
    title_x = (dialog_width - len(title)) // 2
    safe_addstr(dialog, 0, title_x, title)

    # Show prompt
    safe_addstr(dialog, 1, 2, prompt)

    # Create input field
    input_width = dialog_width - 4
    input_field = curses.newwin(1, input_width, dialog_y + 3, dialog_x + 2)

    # Show instructions
    safe_addstr(dialog, 5, 2, "Enter: Confirm | Esc: Cancel")
    dialog.refresh()

    # Initialize input with default value
    input_value = default_value

    # Show input field
    safe_addstr(
        input_field, 0, 0, input_value + " " * (input_width - len(input_value))
    )
    input_field.refresh()

    # Enable cursor
    curses.curs_set(1)

    # Handle input
    while True:
        safe_addstr(
            input_field, 0, 0, input_value + " " * (input_width - len(input_value))
        )
        try:
            input_field.move(0, len(input_value))
        except curses.error:
            # If the cursor position is invalid, move to a safe position
            input_field.move(0, 0)
        input_field.refresh()

        key = stdscr.getch()

        if key == 27:  # Escape
            curses.curs_set(0)  # Hide cursor
            return None
        elif key == 10 or key == 13:  # Enter
            curses.curs_set(0)  # Hide cursor
            return input_value
        elif key == curses.KEY_BACKSPACE or key == 127:  # Backspace
            if input_value:
                input_value = input_value[:-1]
        elif 32 <= key <= 126:  # Printable ASCII characters
            if len(input_value) < input_width - 1:
                input_value += chr(key)

def show_notification(stdscr, message, title="Notification"):
    """Show a notification dialog."""
    height, width = stdscr.getmaxyx()

    # Calculate dialog dimensions
    lines = message.split("\n")
    dialog_height = min(len(lines) + 4, height - 4)
    dialog_width = min(
        max(max(len(line) for line in lines) + 4, len(title) + 4, 40), width - 4
    )
    dialog_y = (height - dialog_height) // 2
    dialog_x = (width - dialog_width) // 2

    # Create dialog window
    dialog = curses.newwin(dialog_height, dialog_width, dialog_y, dialog_x)
    dialog.box()

    # Show title
    title_x = (dialog_width - len(title)) // 2
    safe_addstr(dialog, 0, title_x, title)

    # Show message (with scrolling if needed)
    max_lines = dialog_height - 4
    for i, line in enumerate(lines[:max_lines]):
        safe_addstr(dialog, i + 1, 2, line[: dialog_width - 4])

    if len(lines) > max_lines:
        safe_addstr(dialog, max_lines + 1, 2, "... (more lines not shown)")

    safe_addstr(dialog, dialog_height - 2, 2, "Press any key to continue...")
    dialog.refresh()

    # Wait for input
    stdscr.getch()
    
def show_mode_selection(stdscr):
    """Show a dialog to select the refresh mode and return the selected mode."""
    height, width = stdscr.getmaxyx()

    # Calculate dialog dimensions
    dialog_height = 8
    dialog_width = min(60, width - 4)
    dialog_y = (height - dialog_height) // 2
    dialog_x = (width - dialog_width) // 2

    # Create dialog window
    dialog = curses.newwin(dialog_height, dialog_width, dialog_y, dialog_x)
    dialog.box()

    # Show title
    title = "Select Refresh Mode"
    title_x = (dialog_width - len(title)) // 2
    safe_addstr(dialog, 0, title_x, title)

    # Show options
    safe_addstr(
        dialog, 2, 2, "All interfaces (A): Automatically detect all interfaces"
    )
    safe_addstr(
        dialog, 3, 2, "Select interface (S): Choose a specific interface"
    )
    safe_addstr(dialog, 5, 2, "Press A or S to select mode, or Q to cancel")
    dialog.refresh()

    # Wait for input
    while True:
        key = stdscr.getch()
        if key in [ord("a"), ord("A")]:
            return "all"
        elif key in [ord("s"), ord("S")]:
            return "select"
        elif key in [ord("q"), ord("Q")]:
            return None
        
def show_multi_interface_selection(stdscr, interfaces_by_type, emojis):
    """Show a dialog to select multiple interfaces and return the selected interfaces."""
    # Extract all interfaces by type from the provided dict
    msg_interfaces = interfaces_by_type.get('msg', [])
    srv_interfaces = interfaces_by_type.get('srv', [])
    action_interfaces = interfaces_by_type.get('action', [])

    # Combine all interfaces with their type
    all_interfaces = []
    for interface in msg_interfaces:
        all_interfaces.append(("msg", interface))
    for interface in srv_interfaces:
        all_interfaces.append(("srv", interface))
    for interface in action_interfaces:
        all_interfaces.append(("action", interface))

    if not all_interfaces:
        return None

    height, width = stdscr.getmaxyx()

    # Calculate dialog dimensions
    max_interface_len = (
        max(len(i[1]) for i in all_interfaces) + 10
    )  # Add space for type prefix
    dialog_height = min(
        len(all_interfaces) + 12, height - 4
    )  # Add more space for legend
    dialog_width = min(max_interface_len + 12, width - 4)  # Add space for checkbox
    dialog_y = (height - dialog_height) // 2
    dialog_x = (width - dialog_width) // 2

    # Create dialog window
    dialog = curses.newwin(dialog_height, dialog_width, dialog_y, dialog_x)
    dialog.box()

    # Show title
    title = "Select Interfaces to Document"
    title_x = (dialog_width - len(title)) // 2
    safe_addstr(dialog, 0, title_x, title)

    # Set up scrolling
    selected_idx = 0
    top_idx = 0
    max_visible = (
        dialog_height - 12
    )  # Leave space for title, legend, instructions, and selected count
    selected_interfaces = []

    # Function to redraw the interface list
    def redraw_list():
        # Clear the list area
        for i in range(2, 2 + max_visible):
            safe_addstr(dialog, i, 1, " " * (dialog_width - 2))

        # Draw visible interfaces
        visible_interfaces = all_interfaces[top_idx : top_idx + max_visible]
        for i, (itype, interface) in enumerate(visible_interfaces):
            # Prepare display string with type emoji
            if itype == "msg":
                display_str = f"{emojis['topic']} {interface}"
            elif itype == "srv":
                display_str = f"{emojis['service']} {interface}"
            else:  # action
                display_str = f"{emojis['action']} {interface}"

            # Add checkbox
            is_selected = (itype, interface) in selected_interfaces
            checkbox = "[X]" if is_selected else "[ ]"

            # Highlight selected item
            if i + top_idx == selected_idx:
                dialog.attron(curses.A_REVERSE)
                safe_addstr(
                    dialog,
                    i + 2,
                    2,
                    f"{checkbox} {display_str[:dialog_width - 8]}",
                    curses.A_REVERSE,
                )
                dialog.attroff(curses.A_REVERSE)
            else:
                safe_addstr(
                    dialog, i + 2, 2, f"{checkbox} {display_str[:dialog_width - 8]}"
                )

        # Draw scroll indicators
        if top_idx > 0:
            safe_addstr(dialog, 1, dialog_width - 3, "^")
        if top_idx + max_visible < len(all_interfaces):
            safe_addstr(dialog, 2 + max_visible, dialog_width - 3, "v")

        # Draw legend for emoticons
        legend_y = max_visible + 3
        safe_addstr(dialog, legend_y, 2, "Legend:")
        safe_addstr(
            dialog, legend_y + 1, 4, f"{emojis['topic']} = Message/Topic"
        )
        safe_addstr(dialog, legend_y + 2, 4, f"{emojis['service']} = Service")
        safe_addstr(dialog, legend_y + 3, 4, f"{emojis['action']} = Action")

        # Draw selected items count
        total_count = len(all_interfaces)
        selected_count = len(selected_interfaces)
        count_str = f"Selected: {selected_count} out of {total_count}"
        safe_addstr(dialog, legend_y + 5, 2, count_str)

        # Draw instructions
        safe_addstr(
            dialog,
            dialog_height - 3,
            2,
            "Up/Down: Navigate, Space: Select/Deselect",
        )
        safe_addstr(
            dialog, dialog_height - 2, 2, "Enter: Confirm Selection, Q: Cancel"
        )

        dialog.refresh()

    # Initial draw
    redraw_list()

    # Handle input
    while True:
        key = stdscr.getch()

        if key == curses.KEY_UP:
            if selected_idx > 0:
                selected_idx -= 1
                if selected_idx < top_idx:
                    top_idx = selected_idx
                redraw_list()

        elif key == curses.KEY_DOWN:
            if selected_idx < len(all_interfaces) - 1:
                selected_idx += 1
                if selected_idx >= top_idx + max_visible:
                    top_idx = selected_idx - max_visible + 1
                redraw_list()

        elif key == ord(" "):  # Space key to toggle selection
            current_interface = all_interfaces[selected_idx]
            if current_interface in selected_interfaces:
                selected_interfaces.remove(current_interface)
            else:
                selected_interfaces.append(current_interface)
            redraw_list()

        elif key == curses.KEY_ENTER or key == 10 or key == 13:  # Enter key
            if selected_interfaces:
                return selected_interfaces
            else:
                # Show warning if no interfaces selected
                safe_addstr(
                    dialog,
                    max_visible + 8,
                    2,
                    "Please select at least one interface!",
                    curses.A_BOLD,
                )
                dialog.refresh()
                time.sleep(1.5)
                redraw_list()

        elif key in [ord("q"), ord("Q")]:
            return None

def show_loading(stdscr, message):
    """Show a loading dialog."""
    height, width = stdscr.getmaxyx()

    # Calculate dialog dimensions
    dialog_height = 5
    dialog_width = min(len(message) + 10, width - 4)
    dialog_y = (height - dialog_height) // 2
    dialog_x = (width - dialog_width) // 2

    # Create dialog window
    dialog = curses.newwin(dialog_height, dialog_width, dialog_y, dialog_x)
    dialog.box()

    # Show message
    safe_addstr(dialog, 1, 2, message)
    safe_addstr(dialog, 2, 2, "Please wait...")
    dialog.refresh()

    return dialog

def draw_ui(stdscr, editor):
    """Draw the main UI."""
    try:
        stdscr.clear()
        height, width = stdscr.getmaxyx()

        # Log screen dimensions for debugging
        logger.debug(f"Screen dimensions: {height}x{width}")

        # Draw title
        title = f"{editor.emojis['ros']} AsyncAPI Editor {editor.emojis['asyncapi']}"
        title_x = max(0, (width - len(title)) // 2)
        safe_addstr(stdscr, 0, title_x, title)

        # Show warning if ROS 2 path is not set
        if not editor.ros2_path_configured:
            warning = f"{editor.emojis['warning']} ROS 2 path not set! Press 'P' to configure."
            warning_x = max(0, (width - len(warning)) // 2)
            stdscr.attron(curses.A_BOLD)
            safe_addstr(stdscr, 1, warning_x, warning)
            stdscr.attroff(curses.A_BOLD)
        else:
            # Draw instructions
            instructions = f"{editor.emojis['info']}  Use arrow keys to navigate"
            safe_addstr(stdscr, 1, 2, instructions[: width - 4])

        # Draw channels list frame
        safe_addstr(stdscr, 2, 0, "+" + "-" * (width - 2) + "+")
        for i in range(3, height - 4):
            safe_addstr(stdscr, i, 0, "|")
            safe_addstr(stdscr, i, width - 1, "|")
        safe_addstr(stdscr, height - 4, 0, "+" + "-" * (width - 2) + "+")

        # Draw channels list title
        channels_title = "Channel Addresses"
        safe_addstr(stdscr, 2, 2, channels_title)

        # Draw channels
        max_visible = height - 7  # Adjust based on UI layout
        visible_count = min(max_visible, len(editor.channels))

        # Log visible channels info for debugging
        logger.debug(
            f"Channels: {len(editor.channels)}, Visible: {visible_count}, Top line: {editor.top_line}"
        )

        for i in range(visible_count):
            channel_idx = i + editor.top_line
            if channel_idx < len(editor.channels):
                channel = editor.channels[channel_idx]
                # Truncate channel name if too long
                display_channel = f" {channel}"[: width - 5]

                # Highlight selected item
                if channel_idx == editor.selected_index:
                    stdscr.attron(curses.A_REVERSE)
                    stdscr.addstr(i + 3, 2, display_channel)
                    stdscr.attroff(curses.A_REVERSE)
                else:
                    stdscr.addstr(i + 3, 2, display_channel)

        # Draw buttons
        button_y = height - 3
        buttons = [
            f"{editor.emojis['delete']}  Delete (D)",
            f"{editor.emojis['refresh']} Refresh (R)",
            f"{editor.emojis['debug']} Debug (I)",
            f"{editor.emojis['path']} Path (P)",
            f"{editor.emojis['exit']} Exit (Q)",
        ]

        button_width = width // len(buttons)
        for i, button in enumerate(buttons):
            x = i * button_width + 2
            safe_addstr(stdscr, button_y, x, button[: button_width - 3])

        # Draw message if there is one
        if editor.message and time.time() < editor.message_timeout:
            message_y = height - 1

            # Set prefix based on message type
            if editor.message_type == "error":
                prefix = f"{editor.emojis['error']} "
            elif editor.message_type == "success":
                prefix = f"{editor.emojis['success']} "
            else:
                prefix = f"{editor.emojis['info']} "

            # Truncate message if too long
            display_message = prefix + editor.message
            if len(display_message) > width - 4:
                display_message = display_message[: width - 7] + "..."

            safe_addstr(stdscr, message_y, 2, display_message)

        stdscr.refresh()

    except Exception as e:
        logger.exception(f"Error drawing UI: {e}")
        # Try to show a minimal UI if the main one fails
        try:
            stdscr.clear()
            safe_addstr(stdscr, 0, 0, "AsyncAPI Editor")
            safe_addstr(stdscr, 1, 0, f"Error: {e}")
            safe_addstr(stdscr, 2, 0, "Press 'Q' to quit")
            stdscr.refresh()
        except:
            pass  # If even the minimal UI fails, just continue