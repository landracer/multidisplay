#!/usr/bin/env python3
"""
Documentation generator for MultiDisplay project
"""

import os
import re
from pathlib import Path

def extract_functions(file_path):
    """Extract function signatures from a C++ file"""
    functions = []
    try:
        with open(file_path, 'r') as f:
            content = f.read()
        
        # Look for function definitions
        # This regex looks for function signatures
        func_pattern = r'(?:\w+(?:\s*\*\s*)?\w+\s*\([^;]*\)\s*(?:const)?\s*(?:noexcept)?\s*(?:\w+)?\s*;|void\s+\w+\s*\([^;]*\)\s*;|class\s+\w+|struct\s+\w+)'
        
        # More specific pattern for function definitions
        lines = content.split('\n')
        current_function = None
        
        for line in lines:
            line = line.strip()
            
            # Look for function definitions
            if re.match(r'^\w+(?:\s*\*\s*)?\w+\s*\([^)]*\)\s*(?:const)?\s*(?:noexcept)?\s*(?:\w+)?\s*{', line):
                # This is a function definition
                func_name = re.search(r'^\w+(?:\s*\*\s*)?(\w+)\s*\([^)]*\)', line)
                if func_name:
                    functions.append({
                        'name': func_name.group(1),
                        'file': os.path.basename(file_path),
                        'signature': line
                    })
            elif re.match(r'^void\s+\w+\s*\([^)]*\)\s*{', line):
                # This is a void function definition
                func_name = re.search(r'^void\s+(\w+)\s*\([^)]*\)', line)
                if func_name:
                    functions.append({
                        'name': func_name.group(1),
                        'file': os.path.basename(file_path),
                        'signature': line
                    })
                    
    except Exception as e:
        print(f"Error reading {file_path}: {e}")
        
    return functions

def generate_api_docs(project_path):
    """Generate API documentation for the project"""
    
    print("MultiDisplay API Documentation")
    print("=" * 50)
    
    # Get all source files
    source_files = []
    for root, dirs, files in os.walk(project_path):
        for file in files:
            if file.endswith(('.cpp', '.h', '.hpp')):
                source_files.append(os.path.join(root, file))
    
    # Group by file
    file_functions = {}
    for file_path in source_files:
        functions = extract_functions(file_path)
        if functions:
            file_functions[os.path.basename(file_path)] = functions
    
    # Print documentation
    for filename, functions in file_functions.items():
        print(f"\n{filename}")
        print("-" * len(filename))
        for func in functions:
            print(f"  {func['signature']}")
    
    print("\n" + "=" * 50)
    print("Documentation generation complete!")

if __name__ == "__main__":
    project_path = "/home/sysadmin/Documents/multidisplay-firmware/multidisplay"
    generate_api_docs(project_path)