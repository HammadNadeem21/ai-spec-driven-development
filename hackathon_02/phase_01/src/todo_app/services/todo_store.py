"""In-memory store for managing Todo items."""

from typing import Dict, List, Optional
from ..models.todo import Todo


class TodoStore:
    """
    In-memory store that manages Todo objects.

    This store handles all CRUD operations for Todo items using a dictionary
    keyed by ID for efficient access. All data exists only during application
    runtime and is not persisted.
    """

    def __init__(self):
        """Initialize an empty store with a counter for ID generation."""
        self._todos: Dict[int, Todo] = {}
        self._next_id = 1

    def create(self, title: str) -> Todo:
        """
        Add a new todo item with auto-generated ID.

        Args:
            title: The title/description of the new todo item

        Returns:
            The newly created Todo object with assigned ID

        Raises:
            ValueError: If the title is empty after trimming
        """
        # Validate the title is not empty
        title = title.strip()
        if not title:
            raise ValueError("Todo title cannot be empty or consist only of whitespace")

        # Create a new Todo with the next available ID
        todo = Todo(id=self._next_id, title=title)
        self._todos[self._next_id] = todo

        # Increment the ID counter for the next item
        self._next_id += 1

        return todo

    def get_by_id(self, todo_id: int) -> Optional[Todo]:
        """
        Retrieve a single todo item by its ID.

        Args:
            todo_id: The ID of the todo item to retrieve

        Returns:
            The Todo object if found, None otherwise
        """
        return self._todos.get(todo_id)

    def get_all(self) -> List[Todo]:
        """
        Retrieve all todo items.

        Returns:
            A list of all Todo objects in the store
        """
        return list(self._todos.values())

    def update(self, todo_id: int, title: str = None, completed: bool = None) -> Optional[Todo]:
        """
        Update an existing todo item's properties.

        Args:
            todo_id: The ID of the todo item to update
            title: New title for the todo (optional)
            completed: New completion status (optional)

        Returns:
            The updated Todo object if found, None otherwise

        Raises:
            ValueError: If a new title is provided but is empty after trimming
        """
        if todo_id not in self._todos:
            return None

        todo = self._todos[todo_id]

        if title is not None:
            title = title.strip()
            if not title:
                raise ValueError("Todo title cannot be empty or consist only of whitespace")
            todo.update_title(title)

        if completed is not None:
            todo.completed = completed

        return todo

    def delete(self, todo_id: int) -> bool:
        """
        Remove a todo item by its ID.

        Args:
            todo_id: The ID of the todo item to remove

        Returns:
            True if the item was found and removed, False otherwise
        """
        if todo_id not in self._todos:
            return False

        del self._todos[todo_id]
        return True

    def mark_complete(self, todo_id: int) -> Optional[Todo]:
        """
        Mark a specific todo item as complete.

        Args:
            todo_id: The ID of the todo item to mark complete

        Returns:
            The updated Todo object if found, None otherwise
        """
        if todo_id not in self._todos:
            return None

        todo = self._todos[todo_id]
        todo.mark_complete()
        return todo

    def clear(self) -> None:
        """Remove all todo items from the store."""
        self._todos.clear()
        self._next_id = 1

    @property
    def count(self) -> int:
        """Get the number of todo items in the store."""
        return len(self._todos)