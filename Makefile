.PHONY: build clean rebuild help

# Build using build.sh script (handles colcon + libexec setup)
build:
	@bash build.sh .

# Clean build artifacts
clean:
	rm -rf build install log

# Clean and rebuild everything
rebuild: clean build

# Help text
help:
	@echo "ENGR489 Robot Platform - ROS2 Build Targets"
	@echo ""
	@echo "Standard ROS2 build process (recommended):"
	@echo "  colcon build $(COLCON_ARGS)"
	@echo "  python3 src/eeen489_robot/setup_libexec.py install/eeen489_robot"
	@echo ""
	@echo "Convenience targets (run both steps):"
	@echo "  make build        - Run colcon build and setup libexec symlinks"
	@echo "  make rebuild      - Clean build artifacts and rebuild everything"
	@echo ""
	@echo "Individual steps:"
	@echo "  make colcon-build - Run colcon build only"
	@echo "  make setup-libexec - Setup libexec symlinks (run after colcon build)"
	@echo "  make clean        - Remove build artifacts (build/, install/, log/)"
