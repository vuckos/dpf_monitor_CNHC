##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##

PROJECT = stm32-can-dpf
BUILD_DIR = bin


U8G2SRCS = $(wildcard u8g2/csrc/u8*.c)
#OBJS += $(U8G2_SRCS:%.c=%.o)

#DEFS += -I./u8g2/csrc
#CFILES = usart.c utils.c stm32-slcan.c
CFILES = main.c  usart.c utils.c can.c
#U8G2SRCS = u8g2/csrc/u8g2_box.c u8g2/csrc/u8g2_buffer.c u8g2/csrc/u8g2_circle.c	u8g2/csrc/u8g2_cleardisplay.c u8g2/csrc/u8g2_d_memory.c u8g2/csrc/u8g2_d_setup.c u8g2/csrc/u8g2_font.c


#FP_FLAGS	?= -msoft-float
#ARCH_FLAGS	= -mthumb -mcpu=cortex-m3 $(FP_FLAGS) -mfix-cortex-m3-ldrd

#ARCH_FLAGS	= -mthumb -mcpu=cortex-m3 -msoft-float -mfix-cortex-m3-ldrd
LDFLAGS += -u _printf_float

DEVICE=stm32f103c8
# OOCD_FILE = board/stm32f4discovery.cfg

# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR = ./libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ./mk/rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
