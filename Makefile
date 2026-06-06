# Makefile for MultiDisplay Project
# Simplified build and documentation generation

.PHONY: docs clean

# Default target
all: docs

# Generate documentation
docs:
	@echo "Generating project documentation..."
	@echo "Project: MultiDisplay Automotive Data Logger"
	@echo "=========================================="
	@echo "Source files:"
	@find . -name "*.cpp" -o -name "*.h" | head -20
	@echo "..."
	@echo ""
	@echo "Key components:"
	@echo "- main.cpp: Main entry point"
	@echo "- MultidisplayController: Main system controller"
	@echo "- SensorData: Sensor data handling"
	@echo "- RPMBoostController: Boost control logic"
	@echo "- LCDController: Display management"
	@echo "- MultidisplayDefines.h: Configuration options"
	@echo ""
	@echo "Features:"
	@echo "- Real-time automotive data monitoring"
	@echo "- Boost control with PID regulation"
	@echo "- Multi-sensor data acquisition"
	@echo "- LCD display output"
	@echo "- PC communication interface"
	@echo "- Safety protection features"
	@echo ""
	@echo "Documentation files:"
	@echo "- PROJECT_DOCUMENTATION.md: Complete project overview"
	@echo "- CODE_ANNOTATIONS.md: Detailed code annotations"
	@echo "- multidisplay_agent.py: Code analysis agent"

# Clean build artifacts (if any)
clean:
	@echo "Cleaning build artifacts..."
	@rm -f *.o *.out

# Run documentation generator
generate-docs:
	@echo "Running documentation generator..."
	@python3 generate_docs.py

# Run the agent
run-agent:
	@echo "Running MultiDisplay agent..."
	@python3 multidisplay_agent.py

# Help target
help:
	@echo "MultiDisplay Project Makefile"
	@echo "============================"
	@echo "Available targets:"
	@echo "  all         - Generate documentation (default)"
	@echo "  docs        - Generate documentation"
	@echo "  clean       - Clean build artifacts"
	@echo "  generate-docs - Run documentation generator"
	@echo "  run-agent   - Run the code analysis agent"
	@echo "  help        - Show this help"