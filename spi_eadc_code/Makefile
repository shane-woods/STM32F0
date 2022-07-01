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


#Use all the fancy Makefile setup in libopencm3-examples

BINARY = spi_eadc

OBJS = uart.o

LDSCRIPT = ../libopencm3-examples/examples/stm32/f0/stm32f0-discovery/stm32f0-discovery.ld

LIBNAME		= opencm3_stm32f0
DEFS		+= -DSTM32F0

FP_FLAGS	?= -msoft-float
ARCH_FLAGS	= -mthumb -mcpu=cortex-m0 $(FP_FLAGS)


################################################################################
# OpenOCD specific variables

OOCD		?= openocd
OOCD_INTERFACE	?= stlink-v2-1
OOCD_TARGET	?= stm32f0x

################################################################################
# texane/stlink specific variables
#STLINK_PORT	?= :4242

OPENCM3_DIR = ../libopencm3

include ../libopencm3-examples/examples/rules.mk

# BINARY = spi_eadc

# OBJS = ../uart.o

# LDSCRIPT = ../libopencm3-examples/examples/stm32/f0/stm32f0-discovery/stm32f0-discovery.ld

# include ../libopencm3-examples/examples/stm32/f0/Makefile.include

