# Data Model: Console Todo Application (Phase I)

## Overview
This document defines the data structures and relationships for the in-memory Python console todo application.

## Todo Item Entity

### Attributes
- **id** (int): Unique identifier for the todo item, auto-generated
  - Required: Yes
  - Constraints: Positive integer, unique within the application session
  - Validation: Must be greater than 0, must not conflict with existing IDs

- **title** (str): Title/description of the todo item
  - Required: Yes
  - Constraints: Non-empty string, trimmed of leading/trailing whitespace
  - Validation: Length must be at least 1 character after trimming

- **completed** (bool): Completion status of the todo item
  - Required: Yes
  - Default Value: False
  - Constraints: Boolean value only

- **created_at** (datetime): Timestamp when the todo was created
  - Required: Yes
  - Default Value: Current timestamp at creation
  - Format: ISO 8601 datetime string

### State Transitions
- **Initial State**: `completed = False`
- **Transition to Complete**: When user marks item as complete, `completed` changes from `False` to `True`
- **No reverse transition**: Once completed, items remain completed (as per typical todo app behavior)

### Business Rules
1. Title cannot be empty or consist only of whitespace
2. ID must be unique within the application session
3. Completed status can only change from False to True, not vice versa
4. Each todo item has a creation timestamp that cannot be modified

## Todo List Collection

### Characteristics
- **Type**: In-memory collection (likely implemented as a dictionary keyed by ID)
- **Capacity**: Limited by available memory
- **Persistence**: Exists only during application runtime
- **Access Pattern**: Direct access by ID for updates/deletes, iteration for viewing

### Operations Supported
- **Create**: Add new todo item with auto-generated ID
- **Read**: Retrieve single item by ID or all items
- **Update**: Modify existing item properties (title, completion status)
- **Delete**: Remove item by ID

### Validation Rules
- Duplicate titles are allowed (different items can have same title)
- Attempting to operate on non-existent ID should raise appropriate error
- All operations must maintain data integrity

## Relationships
- **Todo List** contains zero or more **Todo Items**
- Each **Todo Item** belongs to exactly one **Todo List** (within the application context)