BUILD_DIR  	:=$(CURDIR)/build

all: at

at:
	$(MAKE) -C target/415dk BUILD_DIR=$(BUILD_DIR)/415dk

at-program:
	$(MAKE) -C target/415dk BUILD_DIR=$(BUILD_DIR)/415dk program

clean:
	rm -rf $(BUILD_DIR)