all:
	./format.sh
	cd build && make -j && ./rasterizer
