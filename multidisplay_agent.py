#!/usr/bin/env python3
"""
MultiDisplay Agent - A code assistant for the MultiDisplay automotive data logging system

This agent helps with understanding, documenting, and working with the MultiDisplay firmware codebase.
"""

import os
import re
from typing import Dict, List, Optional, Tuple

class MultiDisplayAgent:
    """Agent for analyzing and documenting the MultiDisplay firmware"""
    
    def __init__(self, project_path: str):
        self.project_path = project_path
        self.files = self._get_project_files()
        
    def _get_project_files(self) -> List[str]:
        """Get all source files in the project"""
        source_files = []
        for root, dirs, files in os.walk(self.project_path):
            for file in files:
                if file.endswith(('.cpp', '.h', '.hpp')):
                    source_files.append(os.path.join(root, file))
        return source_files
    
    def analyze_project_structure(self) -> Dict:
        """Analyze the overall project structure"""
        analysis = {
            'files': len(self.files),
            'components': [],
            'key_features': [],
            'hardware_config': [],
            'sensor_types': []
        }
        
        # Analyze key files
        for file_path in self.files:
            filename = os.path.basename(file_path)
            if 'main' in filename.lower():
                analysis['components'].append('Main entry point')
            elif 'controller' in filename.lower():
                analysis['components'].append(f'Controller: {filename}')
            elif 'sensor' in filename.lower():
                analysis['components'].append(f'Sensor handler: {filename}')
            elif 'display' in filename.lower() or 'lcd' in filename.lower():
                analysis['components'].append(f'Display handler: {filename}')
            elif 'boost' in filename.lower():
                analysis['components'].append(f'Boost control: {filename}')
            elif 'define' in filename.lower():
                analysis['components'].append(f'Configuration: {filename}')
                
        return analysis
    
    def extract_function_signatures(self, file_path: str) -> List[str]:
        """Extract function signatures from a C++ file"""
        signatures = []
        try:
            with open(file_path, 'r') as f:
                content = f.read()
                
            # Look for function definitions
            func_pattern = r'(?:\w+(?:\s*\*\s*)?\w+\s*\([^;]*\)\s*(?:const)?\s*(?:noexcept)?\s*(?:\w+)?\s*;|void\s+\w+\s*\([^;]*\)\s*;|class\s+\w+|struct\s+\w+)'
            lines = content.split('\n')
            
            for line in lines:
                line = line.strip()
                if line.startswith('void ') or line.startswith('int ') or line.startswith('float ') or line.startswith('uint8_t ') or line.startswith('bool '):
                    # Extract function signature
                    if '(' in line and ';' in line:
                        signatures.append(line)
                        
        except Exception as e:
            print(f"Error reading {file_path}: {e}")
            
        return signatures
    
    def get_project_summary(self) -> str:
        """Generate a summary of the project"""
        analysis = self.analyze_project_structure()
        
        summary = f"""
# MultiDisplay Project Summary

## Project Overview
This is an automotive data logging and boost control system running on Arduino hardware.

## Project Structure
- Total source files: {analysis['files']}
- Key components: {', '.join(analysis['components']) if analysis['components'] else 'None identified'}

## Key Features
- Real-time automotive data monitoring
- Boost control with PID regulation
- Multi-sensor data acquisition
- LCD display output
- PC communication interface
- Safety protection features

## Hardware Configuration
The system is designed for Seeeduino Mega with ATmega1280/2560 MCU and supports:
- 20x4 LCD display
- Multiple analog sensors (Type K thermocouples, VDO sensors, etc.)
- Boost control with N75 valve
- K-line communication with ECUs
        """
        
        return summary

def main():
    """Main function to run the agent"""
    project_path = "/home/sysadmin/Documents/multidisplay-firmware/multidisplay"
    
    if not os.path.exists(project_path):
        print(f"Project path {project_path} does not exist")
        return
        
    agent = MultiDisplayAgent(project_path)
    summary = agent.get_project_summary()
    
    print(summary)
    
    # Show some key function signatures
    print("\n=== Key Function Signatures ===")
    for file_path in agent.files:
        if 'MultidisplayController' in file_path or 'SensorData' in file_path:
            signatures = agent.extract_function_signatures(file_path)
            if signatures:
                print(f"\n{os.path.basename(file_path)}:")
                for sig in signatures[:5]:  # Show first 5 signatures
                    print(f"  {sig}")

if __name__ == "__main__":
    main()