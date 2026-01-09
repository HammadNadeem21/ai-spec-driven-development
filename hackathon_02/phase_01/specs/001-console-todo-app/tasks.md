# Tasks: Console Todo Application (Phase I)

**Feature**: Console Todo Application (Phase I)
**Created**: 2026-01-10
**Branch**: `001-console-todo-app`
**Spec**: [specs/001-console-todo-app/spec.md](specs/001-console-todo-app/spec.md)
**Plan**: [specs/001-console-todo-app/plan.md](specs/001-console-todo-app/plan.md)

## Implementation Strategy

**Approach**: Build incrementally following user story priorities, starting with MVP (User Story 1: Add Todo Item) and expanding functionality step by step. Each user story should be independently testable and deliver value.

**MVP Scope**: User Story 1 (Add Todo Item) with basic model and CLI interface.

**Delivery Order**: P1 → P1 → P2 → P2 → P3 (priorities from spec.md)

## Dependencies

- **US1 (Add)** → Base Todo model and store
- **US2 (View)** → Base Todo model and store
- **US3 (Complete)** → Base Todo model and store
- **US4 (Update)** → Base Todo model and store
- **US5 (Delete)** → Base Todo model and store

## Parallel Execution Examples

- **US1 & US2**: Can be developed in parallel after foundational components exist
- **US3, US4, US5**: Can be developed in parallel after foundational components exist
- **Unit tests**: Can be written in parallel for different components

---

## Phase 1: Setup

- [x] T001 Create project directory structure per implementation plan
- [x] T002 Create pyproject.toml with Python 3.13+ requirement
- [x] T003 Initialize src/todo_app package structure
- [x] T004 Initialize tests directory structure
- [x] T005 Set up basic application entry point (main.py)

## Phase 2: Foundational Components

- [x] T010 [P] Create Todo model in src/todo_app/models/todo.py
- [x] T011 [P] Create in-memory Todo store in src/todo_app/services/todo_store.py
- [x] T012 [P] Create command processor interface in src/todo_app/commands/command_processor.py
- [x] T013 [P] Create CLI interface base in src/todo_app/cli/interface.py

## Phase 3: User Story 1 - Add Todo Item (Priority: P1)

**Goal**: Enable users to add new todo items to their list with unique IDs and confirmation

**Independent Test**: Can be fully tested by running the add command with a sample todo description and verifying the item appears in the list.

**Tasks**:
- [x] T020 [US1] Implement add command logic in command processor
- [x] T021 [US1] Add validation for empty todo descriptions
- [x] T022 [US1] Add CLI handler for add command in interface
- [x] T023 [US1] Test add functionality with valid input
- [x] T024 [US1] Test add functionality with empty description (error case)

## Phase 4: User Story 2 - View Todo Items (Priority: P1)

**Goal**: Allow users to see all their todo items listed with IDs and completion status

**Independent Test**: Can be fully tested by adding sample items and then viewing the list to confirm all items appear correctly with their status.

**Tasks**:
- [x] T030 [US2] Implement view command logic in command processor
- [x] T031 [US2] Add CLI handler for view command in interface
- [x] T032 [US2] Format display of todo items with ID, title, and status
- [x] T033 [US2] Handle case when no todo items exist
- [x] T034 [US2] Test view functionality with multiple items
- [x] T035 [US2] Test view functionality with no items

## Phase 5: User Story 3 - Mark Todo Complete (Priority: P2)

**Goal**: Enable users to mark specific todo items as complete by referencing their ID

**Independent Test**: Can be fully tested by adding an item, marking it complete, and then viewing to confirm the status changed.

**Tasks**:
- [x] T040 [US3] Implement complete command logic in command processor
- [x] T041 [US3] Add validation for valid ID existence
- [x] T042 [US3] Add CLI handler for complete command in interface
- [x] T043 [US3] Update todo item completion status
- [x] T044 [US3] Test complete functionality with valid ID
- [x] T045 [US3] Test complete functionality with invalid ID (error case)

## Phase 6: User Story 4 - Update Todo Description (Priority: P2)

**Goal**: Allow users to change the description of existing todo items by referencing their ID

**Independent Test**: Can be fully tested by adding an item, updating its description, and then viewing to confirm the change.

**Tasks**:
- [x] T050 [US4] Implement update command logic in command processor
- [x] T051 [US4] Add validation for valid ID existence
- [x] T052 [US4] Add CLI handler for update command in interface
- [x] T053 [US4] Update todo item description
- [x] T054 [US4] Test update functionality with valid ID and description
- [x] T055 [US4] Test update functionality with invalid ID (error case)

## Phase 7: User Story 5 - Delete Todo Item (Priority: P3)

**Goal**: Enable users to remove specific todo items from their list by referencing their ID

**Independent Test**: Can be fully tested by adding an item, deleting it, and then viewing to confirm it's no longer in the list.

**Tasks**:
- [x] T060 [US5] Implement delete command logic in command processor
- [x] T061 [US5] Add validation for valid ID existence
- [x] T062 [US5] Add CLI handler for delete command in interface
- [x] T063 [US5] Remove todo item from store
- [x] T064 [US5] Test delete functionality with valid ID
- [x] T065 [US5] Test delete functionality with invalid ID (error case)

## Phase 8: Polish & Cross-Cutting Concerns

- [x] T070 Implement error handling for invalid commands
- [x] T071 Add help command functionality
- [x] T072 Improve user interface formatting and messages
- [x] T073 Handle edge cases for commands with extra spaces or special characters
- [x] T074 Add comprehensive error messages for all failure cases
- [x] T075 Implement main application loop in main.py
- [x] T076 Test complete workflow with all commands
- [x] T077 Document usage in README
- [x] T078 Verify all constraints met (in-memory only, stdlib only)