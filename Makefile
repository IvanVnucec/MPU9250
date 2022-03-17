BUILD_FOLDER = build

.PHONY: all install_deps setup_cmake build clean

all: build

install_deps:
	sudo apt-get install -y gcc cmake ninja-build

setup_cmake: 
	rm -rf $(BUILD_FOLDER)
	mkdir $(BUILD_FOLDER)
	cd $(BUILD_FOLDER) && \
	cmake -GNinja ..

build:
	cd $(BUILD_FOLDER) && ninja

clean:
	rm -rf $(BUILD_FOLDER)

help:
	@echo all install_deps setup_cmake build clean