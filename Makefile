nproc=$(shell python3 -c 'import multiprocessing; print( max(multiprocessing.cpu_count() - 1, 1))')

# don't use clang-10, CUDA requires clang-9 or lower.
# CXX := clang++-10
# CC := clang-10

.PHONY: plugin.dbg.so
plugin.dbg.so: build/Debug/Makefile
	make -C build/Debug "-j$(nproc)" && \
	rm -f $@ && \
	ln -s build/Debug/KinectFusionApp/libplugin.so plugin.dbg.so && \
	true

.PHONY: plugin.opt.so
plugin.opt.so: build/Release/Makefile
	make -C build/Release "-j$(nproc)" && \
	rm -f $@ && \
	ln -s build/Release/KinectFusionApp/libplugin.so plugin.opt.so && \
	true

build/Debug/Makefile:
	mkdir -p build/Debug && \
	cd build/Debug && \
	cmake -DCMAKE_BUILD_TYPE=Debug ../.. && \
	true

build/Release/Makefile:
	mkdir -p build/Release && \
	cd build/Release && \
	cmake -DCMAKE_BUILD_TYPE=Release ../.. && \
	true

tests/run:
tests/gdb:

.PHONY: clean
clean:
	touch build && rm -rf build *.so
