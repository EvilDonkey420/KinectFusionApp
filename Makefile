nproc=$(shell python3 -c 'import multiprocessing; print( max(multiprocessing.cpu_count() - 1, 1))')

CXX := clang++-10
CC := clang-10

.PHONY: plugin.opt.so
plugin.opt.so: build/Makefile
	make -C build "-j$(nproc)" && \
	rm -f $@ && \
	ln -s build/KinectFusionApp/libplugin.so plugin.opt.so && \
	true

build/Makefile:
	mkdir -p build && \
	cd build && \
	cmake -DCMAKE_BUILD_TYPE=Release .. && \
	true

tests/run:
tests/gdb:

.PHONY: clean
clean:
	touch build && rm -rf build *.so
