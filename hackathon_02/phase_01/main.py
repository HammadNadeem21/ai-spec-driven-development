#!/usr/bin/env python3
"""
Console Todo Application - Phase I
A simple in-memory todo application with CLI interface.
"""

import sys
import os

# Add the src directory to the Python path to allow imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from todo_app.cli.interface import TodoCLI


def main():
    """Application entry point."""
    cli = TodoCLI()
    cli.run()


if __name__ == "__main__":
    main()