BUILD_DIR := build
INSTALL_DIR := install
LOG_DIR := log

.PHONY: run
run: build 

.PHONY: clean
clean:
	@echo "Cleaning build, install, and log directories..."
	rm -rf $(BUILD_DIR) $(INSTALL_DIR) $(LOG_DIR)
	@echo "Clean complete."

.PHONY: build
build:
	@echo "Building workspace using colcon..."
	colcon build 
	@echo "Build complete."