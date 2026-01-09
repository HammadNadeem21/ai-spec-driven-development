# Quickstart Guide: Console Todo Application (Phase I)

## Prerequisites
- Python 3.13 or higher
- UV package manager (for dependency management)

## Setup Instructions

### 1. Clone or Create the Project
```bash
# Navigate to your project directory
cd your-project-directory
```

### 2. Project Structure
Create the following directory structure:
```
src/
├── todo_app/
│   ├── __init__.py
│   ├── models/
│   │   ├── __init__.py
│   │   └── todo.py
│   ├── services/
│   │   ├── __init__.py
│   │   └── todo_store.py
│   ├── commands/
│   │   ├── __init__.py
│   │   └── command_processor.py
│   └── cli/
│       ├── __init__.py
│       └── interface.py
├── main.py
└── pyproject.toml
```

### 3. Project Configuration
Create `pyproject.toml`:
```toml
[build-system]
requires = ["setuptools>=61.0"]
build-backend = "setuptools"

[project]
name = "console-todo-app"
version = "0.1.0"
description = "A simple in-memory console todo application"
authors = [{name = "Developer", email = "dev@example.com"}]
readme = "README.md"
requires-python = ">=3.13"
classifiers = [
    "Programming Language :: Python :: 3",
    "License :: OSI Approved :: MIT License",
    "Operating System :: OS Independent",
]

[project.scripts]
todo-app = "main:main"
```

### 4. Installation and Running
```bash
# Install the package in development mode
uv sync --dev

# Run the application
python -m src.main

# Or if installed as a script
todo-app
```

## Usage Instructions

### Available Commands
- `add "task description"` - Add a new todo item
- `view` - Display all todo items
- `complete <id>` - Mark a todo item as complete
- `update <id> "new description"` - Update a todo item description
- `delete <id>` - Delete a todo item
- `help` - Show available commands
- `exit` or `quit` - Exit the application

### Example Usage
```bash
# Start the application
python -m src.main

# Add a new todo
> add "Buy groceries"

# View all todos
> view

# Mark item #1 as complete
> complete 1

# Update item #1 description
> update 1 "Buy groceries for dinner party"

# Delete item #1
> delete 1

# Exit the application
> exit
```

## Development

### Running Tests
```bash
# Run all tests
python -m unittest discover tests/

# Run specific test file
python -m unittest tests.test_todo_model
```

### Code Structure
- **Models**: Define data structures and business logic
- **Services**: Handle data operations and business rules
- **Commands**: Process user input and coordinate operations
- **CLI**: Handle user interface and display formatting
- **Main**: Application entry point and control flow

## Troubleshooting
- If you get "command not found" errors, ensure Python 3.13+ is installed and in your PATH
- For import errors, verify the directory structure matches the expected layout
- If the application crashes, check that all required Python standard library modules are available