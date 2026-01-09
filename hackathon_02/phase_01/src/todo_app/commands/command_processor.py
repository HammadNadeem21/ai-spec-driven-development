"""Command processor for handling user commands."""

import re
from typing import Dict, List, Optional, Tuple
from ..models.todo import Todo
from ..services.todo_store import TodoStore


class CommandProcessor:
    """
    Processes user commands and coordinates actions with the TodoStore.

    This class maps user commands to appropriate actions and validates inputs
    before performing operations on the TodoStore.
    """

    def __init__(self):
        """Initialize the command processor with a TodoStore."""
        self.store = TodoStore()
        self.commands = {
            'add': self.add_todo,
            'view': self.view_todos,
            'complete': self.complete_todo,
            'update': self.update_todo,
            'delete': self.delete_todo,
            'help': self.show_help,
        }

    def process_command(self, user_input: str) -> Tuple[bool, str]:
        """
        Process a user command and return the result.

        Args:
            user_input: The raw command string from the user

        Returns:
            A tuple of (success: bool, message: str) indicating the result
        """
        if not user_input.strip():
            return False, "Please enter a command. Type 'help' for available commands."

        # Split the input into command and arguments
        parts = user_input.strip().split(' ', 1)
        command = parts[0].lower()
        args = parts[1] if len(parts) > 1 else ""

        # Check if the command exists
        if command not in self.commands:
            return False, f"Unknown command: '{command}'. Type 'help' for available commands."

        # Process the command
        try:
            return self.commands[command](args)
        except ValueError as e:
            return False, str(e)
        except Exception as e:
            return False, f"Error processing command: {str(e)}"

    def add_todo(self, args: str) -> Tuple[bool, str]:
        """
        Add a new todo item.

        Args:
            args: The title of the new todo item

        Returns:
            A tuple of (success: bool, message: str)
        """
        title = args.strip()
        if not title:
            return False, "Todo title cannot be empty. Please provide a description."

        try:
            todo = self.store.create(title)
            return True, f"Todo added: {todo.title} (ID: {todo.id})"
        except ValueError as e:
            return False, str(e)

    def view_todos(self, args: str = "") -> Tuple[bool, str]:
        """
        View all todo items.

        Args:
            args: Ignored for this command

        Returns:
            A tuple of (success: bool, message: str)
        """
        todos = self.store.get_all()

        if not todos:
            return True, "No todos in the list."

        # Format the todo list for display
        result_lines = ["Todo List:"]
        for todo in todos:
            status = "✓" if todo.completed else "○"
            result_lines.append(f"  {status} [{todo.id}] {todo.title}")

        return True, "\n".join(result_lines)

    def complete_todo(self, args: str) -> Tuple[bool, str]:
        """
        Mark a todo item as complete.

        Args:
            args: The ID of the todo item to mark complete

        Returns:
            A tuple of (success: bool, message: str)
        """
        if not args:
            return False, "Please provide the ID of the todo to mark complete. Usage: complete <id>"

        try:
            todo_id = int(args.strip())
        except ValueError:
            return False, f"Invalid ID: '{args}'. Please provide a valid number."

        todo = self.store.mark_complete(todo_id)
        if todo:
            return True, f"Todo {todo_id} marked as complete: {todo.title}"
        else:
            return False, f"No todo found with ID {todo_id}."

    def update_todo(self, args: str) -> Tuple[bool, str]:
        """
        Update a todo item's description.

        Args:
            args: The ID and new description of the todo item (format: "id new description")

        Returns:
            A tuple of (success: bool, message: str)
        """
        if not args:
            return False, "Please provide the ID and new description. Usage: update <id> <new description>"

        # Split args into ID and new title
        parts = args.split(' ', 1)
        if len(parts) < 2:
            return False, "Please provide both ID and new description. Usage: update <id> <new description>"

        try:
            todo_id = int(parts[0])
        except ValueError:
            return False, f"Invalid ID: '{parts[0]}'. Please provide a valid number."

        new_title = parts[1].strip()
        if not new_title:
            return False, "New description cannot be empty. Please provide a valid description."

        # Update the todo
        updated_todo = self.store.update(todo_id, title=new_title)
        if updated_todo:
            return True, f"Todo {todo_id} updated to: {updated_todo.title}"
        else:
            return False, f"No todo found with ID {todo_id}."

    def delete_todo(self, args: str) -> Tuple[bool, str]:
        """
        Delete a todo item.

        Args:
            args: The ID of the todo item to delete

        Returns:
            A tuple of (success: bool, message: str)
        """
        if not args:
            return False, "Please provide the ID of the todo to delete. Usage: delete <id>"

        try:
            todo_id = int(args.strip())
        except ValueError:
            return False, f"Invalid ID: '{args}'. Please provide a valid number."

        success = self.store.delete(todo_id)
        if success:
            return True, f"Todo {todo_id} deleted."
        else:
            return False, f"No todo found with ID {todo_id}."

    def show_help(self, args: str = "") -> Tuple[bool, str]:
        """
        Show help information.

        Args:
            args: Ignored for this command

        Returns:
            A tuple of (success: bool, message: str)
        """
        help_text = """
Available Commands:
  add <description>     - Add a new todo item
  view                  - View all todo items
  complete <id>         - Mark a todo as complete
  update <id> <desc>    - Update a todo description
  delete <id>           - Delete a todo item
  help                  - Show this help message
  exit                  - Exit the application

Examples:
  add Buy groceries
  view
  complete 1
  update 1 Buy weekly groceries
  delete 1
        """.strip()

        return True, help_text