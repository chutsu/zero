include config.mk

all: dirs
	@make -s -C src

dirs:
	@mkdir -p build

clean:
	@rm -rf build
