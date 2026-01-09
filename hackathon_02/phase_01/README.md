# Console Todo Application (Phase I)

A simple in-memory console todo application built with Python 3.13+.

## Features

- Add, view, update, delete, and mark todo items as complete
- In-memory storage (no persistence)
- Command-line interface
- Cross-platform compatibility

## Requirements

- Python 3.13 or higher

## Installation

1. Clone the repository
2. Install dependencies: `pip install -e .` (or use uv: `uv sync`)

## Usage

Run the application:

```bash
python -m main
```

Or if installed as a package:

```bash
todo-app
```

### Available Commands

- `add <description>` - Add a new todo item
- `view` - View all todo items
- `complete <id>` - Mark a todo as complete
- `update <id> <desc>` - Update a todo description
- `delete <id>` - Delete a todo item
- `help` - Show help information
- `exit` or `quit` - Exit the application

### Examples

```bash
add Buy groceries
view
complete 1
update 1 Buy weekly groceries
delete 1
```

## Architecture

The application follows a clear separation of concerns:

- **Models**: `src/todo_app/models/todo.py` - Defines the Todo data structure
- **Services**: `src/todo_app/services/todo_store.py` - Handles data operations
- **Commands**: `src/todo_app/commands/command_processor.py` - Processes user commands
- **CLI**: `src/todo_app/cli/interface.py` - Handles user interface
- **Main**: `main.py` - Application entry point

## Constraints

- In-memory only (no file or database persistence)
- Python standard library only (no external dependencies)
- Single-user console application