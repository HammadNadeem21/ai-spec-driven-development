"""Todo model representing a single task."""

from dataclasses import dataclass
from datetime import datetime
from typing import Optional


@dataclass
class Todo:
    """
    Represents a single todo item with an ID, title, completion status, and creation timestamp.

    Attributes:
        id: Unique identifier for the todo item, auto-generated
        title: Title/description of the todo item (non-empty string)
        completed: Completion status of the todo item (default: False)
        created_at: Timestamp when the todo was created (ISO 8601 format)
    """

    id: int
    title: str
    completed: bool = False
    created_at: datetime = None

    def __post_init__(self):
        """Initialize the created_at timestamp if not provided."""
        if self.created_at is None:
            self.created_at = datetime.now()

        # Validate that title is not empty after trimming
        self.title = self.title.strip()
        if not self.title:
            raise ValueError("Todo title cannot be empty or consist only of whitespace")

    @property
    def status(self) -> str:
        """Return the completion status as a string ('completed' or 'pending')."""
        return "completed" if self.completed else "pending"

    def mark_complete(self) -> None:
        """Mark the todo item as complete."""
        self.completed = True

    def update_title(self, new_title: str) -> None:
        """Update the title of the todo item after validating it's not empty."""
        new_title = new_title.strip()
        if not new_title:
            raise ValueError("Todo title cannot be empty or consist only of whitespace")
        self.title = new_title

    def to_dict(self) -> dict:
        """Convert the Todo object to a dictionary representation."""
        return {
            "id": self.id,
            "title": self.title,
            "completed": self.completed,
            "created_at": self.created_at.isoformat() if self.created_at else None
        }