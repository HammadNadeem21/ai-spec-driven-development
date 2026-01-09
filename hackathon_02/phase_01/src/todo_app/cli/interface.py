"""CLI interface for the Todo application."""

import sys
from typing import Tuple
from ..commands.command_processor import CommandProcessor


class TodoCLI:
    """
    Command-Line Interface for interacting with the Todo application.

    Handles user input, processes commands through the CommandProcessor,
    and displays results to the console.
    """

    def __init__(self):
        """Initialize the CLI with a command processor."""
        self.processor = CommandProcessor()
        self.running = True

    def display_welcome(self):
        """Display welcome message and instructions."""
        welcome_msg = """
Welcome to the Console Todo App!
===============================
Type 'help' for available commands or 'exit' to quit.
        """.strip()
        print(welcome_msg)

    def display_result(self, success: bool, message: str):
        """
        Display the result of a command execution.

        Args:
            success: Whether the command was successful
            message: The result message to display
        """
        if success:
            print(message)
        else:
            print(f"Error: {message}")

    def process_user_input(self, user_input: str) -> bool:
        """
        Process user input and return whether to continue running.

        Args:
            user_input: The raw input from the user

        Returns:
            True if the application should continue running, False otherwise
        """
        # Check for exit commands
        command = user_input.strip().lower()
        if command in ['exit', 'quit', 'q']:
            return False

        # Process the command
        success, message = self.processor.process_command(user_input)
        self.display_result(success, message)

        return True  # Continue running unless exit command was given

    def run(self):
        """Run the main application loop."""
        self.display_welcome()

        while self.running:
            try:
                # Get user input
                user_input = input("\n> ").strip()

                # Process the input
                self.running = self.process_user_input(user_input)

            except KeyboardInterrupt:
                print("\n\nExiting application...")
                break
            except EOFError:
                print("\n\nExiting application...")
                break

        print("Goodbye!")