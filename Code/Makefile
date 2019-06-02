SHELL=bash

# System configuration
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
RM = rm -rf
INC = -Iinc -Ilib/STM32F4xx_HAL_Driver/Inc -Ilib/STM32F4xx_HAL_Driver/Inc/Legacy -Ilib/CMSIS/ -Ilib/etl/include/etl -Ilib/etl/include
BIN_NAME = Toast
DEBUG_DIR = debugbin
RELEASE_DIR = bin
LDSCRIPT = "STM32F411RE_Linker.ld"
MCU = "STM32F411xE"

#Debug Settings
ifeq ($(Debug),1)

	# Assembler, Compiler and Linker flags and linker script settings
	LDFLAGS=-lm -mthumb -mhard-float -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -Wl,--no-wchar-size-warning -Wl,--gc-sections -T$(LDSCRIPT) -static  -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group -specs=nano.specs -specs=nosys.specs  -Wl,-cref "-Wl,-Map=$(BIN_DIR)/$(BIN_NAME).map" -Wl,--defsym=malloc_getpagesize_P=0x1000
	ASMFLAGS=-c -g -Os -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mthumb -mhard-float -specs=nano.specs -D"$(MCU)"  -x assembler-with-cpp $(INC)
	CFLAGS=-c -g -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -Os -Wall -ffunction-sections -fdata-sections -mthumb -mhard-float -specs=nano.specs -D"$(MCU)" $(INC)
	CXXFLAGS=-fno-threadsafe-statics -c -g -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -std=c++14 -Os -Wall -ffunction-sections -fdata-sections -mthumb -mhard-float -specs=nano.specs -fno-exceptions -fno-rtti -D"$(MCU)" $(INC)

	# Define output directory
	OBJECT_DIR = $(DEBUG_DIR)
	BIN_DIR = $(OBJECT_DIR)

#Release Settings
else 

	# Assembler, Compiler and Linker flags and linker script settings
	LDFLAGS=-lm -mthumb -mhard-float -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -Wl,--no-wchar-size-warning -Wl,--gc-sections -T$(LDSCRIPT) -static  -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group -specs=nano.specs -specs=nosys.specs  -Wl,-cref "-Wl,-Map=$(BIN_DIR)/$(BIN_NAME).map" -Wl,--defsym=malloc_getpagesize_P=0x1000
	ASMFLAGS=-c -Os -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mthumb -mhard-float -specs=nano.specs -D"$(MCU)"  -x assembler-with-cpp $(INC)
	CFLAGS=-c -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -Os -Wall -ffunction-sections -fdata-sections -mthumb -mhard-float -specs=nano.specs -D"$(MCU)" $(INC)
	CXXFLAGS=-fno-threadsafe-statics -c -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -std=c++14 -Os -Wall -ffunction-sections -fdata-sections -mthumb -mhard-float -specs=nano.specs -fno-exceptions -fno-rtti -D"$(MCU)" $(INC)

	# Define output directory
	OBJECT_DIR = $(RELEASE_DIR)
	BIN_DIR = $(OBJECT_DIR)

endif
# Define sources and objects
SRC := $(wildcard */*/*/*/*/*/*/*.c) \
	$(wildcard */*/*/*/*/*/*.c) \
	$(wildcard */*/*/*/*/*.c) \
	$(wildcard */*/*/*/*.c) \
	$(wildcard */*/*/*.c) \
	$(wildcard */*/*.c) \
	$(wildcard */*.c) \
	$(wildcard */*/*/*/*/*/*/*.cpp) \
	$(wildcard */*/*/*/*/*/*.cpp) \
	$(wildcard */*/*/*/*/*.cpp) \
	$(wildcard */*/*/*/*.cpp) \
	$(wildcard */*/*/*.cpp) \
	$(wildcard */*/*.cpp) \
	$(wildcard */*.cpp)
SRCSASM := $(wildcard */*/*/*/*/*/*/*/*.s) \
	$(wildcard */*/*/*/*/*/*/*.s) \
	$(wildcard */*/*/*/*/*/*.s) \
	$(wildcard */*/*/*/*/*.s) \
	$(wildcard */*/*/*/*.s) \
	$(wildcard */*/*/*.s) \
	$(wildcard */*/*.s) \
	$(wildcard */*.s)

HEADERS := $(wildcard */*/*/*/*/*/*/*/*.h) \
	$(wildcard */*/*/*/*/*/*/*.h) \
	$(wildcard */*/*/*/*/*/*.h) \
	$(wildcard */*/*/*/*/*.h) \
	$(wildcard */*/*/*/*.h) \
	$(wildcard */*/*/*.h) \
	$(wildcard */*/*.h) \
	$(wildcard */*.h)
HEADERS += $(wildcard */*/*/*/*/*/*/*/*.hpp) \
	$(wildcard */*/*/*/*/*/*/*.hpp) \
	$(wildcard */*/*/*/*/*/*.hpp) \
	$(wildcard */*/*/*/*/*.hpp) \
	$(wildcard */*/*/*/*.hpp) \
	$(wildcard */*/*/*.hpp) \
	$(wildcard */*/*.hpp) \
	$(wildcard */*.hpp)

OBJS := $(SRC:%.c=$(OBJECT_DIR)/%.o) $(SRCSASM:%.s=$(OBJECT_DIR)/%.o)
OBJS := $(OBJS:%.S=$(OBJECT_DIR)/%.o)  
OBJS := $(OBJS:%.cpp=$(OBJECT_DIR)/%.o)


###############
# Build project
# Major targets
###############
all: $(OBJS) buildelf objdump buildbin
	@echo -e "Build Complete"

buildelf: $(OBJS)
	@echo -e "Linking..."
	$(CXX) -o "$(BIN_DIR)/$(BIN_NAME).elf" $(OBJS) $(LDFLAGS)
	@echo -e ""

buildbin: buildelf
	@echo -e "Copying Object..."
	$(PREFIX)objcopy -O binary $(BIN_DIR)/$(BIN_NAME).elf $(BIN_DIR)/$(BIN_NAME).bin
	@echo -e ""

objdump: $(OBJS) buildelf
	@echo -e "Generating Object Dump..."
	$(PREFIX)objdump -xCd "$(BIN_DIR)/$(BIN_NAME).elf" > "$(BIN_DIR)/$(BIN_NAME).txt"
	@echo -e ""

$(OBJS): $(HEADERS)

clean:
	$(RM) $(RELEASE_DIR) $(DEBUG_DIR)


##################
# Specific targets
##################
$(OBJECT_DIR)/src/main.o: src/main.cpp
	mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) src/main.cpp -o $(OBJECT_DIR)/src/main.o
	@echo -e "" 


##################
# Implicit targets
##################
$(OBJECT_DIR)/%.o: %.c
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $< -o $@
	@echo -e ""

$(OBJECT_DIR)/%.o: %.cpp
	mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $< -o $@
	@echo -e ""

$(OBJECT_DIR)/%.o: %.s
	mkdir -p $(dir $@)
	$(CC) $(ASMFLAGS) $< -o $@
	@echo -e ""
	
$(OBJECT_DIR)/%.o: %.S
	mkdir -p $(dir $@)
	$(CC) $(ASMFLAGS) $< -o $@
	@echo -e ""