.PHONY: build
build :
	rm -rf build
	mkdir build
	sh -c "cd build && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .."
	make -C build -j8

install: build
	sudo make -C build install
