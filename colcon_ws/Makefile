mode=Release
.SILENT: all clean
.PHONY: all clean

all: 
		if [ -z "$(pkg)" ]; then \
			echo "building all"; \
			colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=$(mode); \
		else \
			echo "building $(pkg)"; \
			colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=$(mode) --packages-select $(pkg);\
		fi

clean: 
	if [ -z "$(pkg)" ]; then \
		echo "cleaning all"; \
		rm -r build install; \
  else \
    echo "cleaning $(pkg)"; \
    rm -r build/$(pkg) install/$(pkg); \
  fi

