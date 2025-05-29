# Toolchain prefix: adjust if needed for Zephyr or ESP-IDF toolchain
CC = xtensa-esp32-elf-gcc
AR = xtensa-esp32-elf-ar

# Compiler flags (adjust for your project)
CFLAGS = -Os -g -mlongcalls -ffunction-sections -fdata-sections -std=gnu11

# Include directories (add ESP-IDF headers & your includes)
INCLUDES = -I/home/atoth/EspCam4011/esp-idf/components \
           -I/home/atoth/EspCam4011/cam_streaming/components/include \
           -I/home/atoth/EspCam4011/cam_streaming/main/include \
           -I/home/atoth/EspCam4011/cam_streaming/managed_components/espressif__esp32-camera/driver/include

# Source files - adjust or add paths as needed
SRCS = $(wildcard main/*.c) $(wildcard components/**/*.c)

# Object files
OBJS = $(SRCS:.c=.o)

# Output directory for objects and lib
BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/objs

LIB_NAME = libcam_streaming.a
LIB_PATH = $(BUILD_DIR)/$(LIB_NAME)

.PHONY: all clean

all: $(LIB_PATH)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

# Compile each .c to .o in OBJ_DIR preserving relative paths
$(OBJ_DIR)/%.o: %.c | $(OBJ_DIR)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Link all .o files into static lib
$(LIB_PATH): $(OBJS:%.c=$(OBJ_DIR)/%.o)
	$(AR) rcs $@ $^

clean:
	rm -rf $(BUILD_DIR)