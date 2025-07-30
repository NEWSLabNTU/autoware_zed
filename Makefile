.PHONY: help build clean test run format

help:
	@echo "Autoware ZED Package Makefile"
	@echo "============================"
	@echo "Available targets:"
	@echo "  make build  - Build autoware_zed package"
	@echo "  make clean  - Clean build artifacts"
	@echo "  make test   - Run package tests"
	@echo "  make run    - Run the transformer node"
	@echo "  make format - Format source code"

build:
	@echo "Building autoware_zed package..."
	. ../zed/install/setup.sh && \
	colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

clean:
	@echo "Cleaning autoware_zed build artifacts..."
	rm -rf build install log

test:
	@echo "Running autoware_zed tests..."
	@cd ../.. && . install/setup.sh && \
	colcon test && \
	colcon test-result --verbose

run:
	@echo "Running autoware_zed node..."
	@cd ../.. && . install/setup.sh && \
	ros2 run autoware_zed autoware_zed_node

format:
	@echo "Formatting autoware_zed source code..."
	@find src include -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i
	@find scripts -name "*.py" | xargs autopep8 --in-place --aggressive --aggressive || true
