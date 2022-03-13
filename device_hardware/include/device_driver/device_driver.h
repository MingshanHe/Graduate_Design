#ifndef DEVICE_DRIVER_H
#define DEVICE_DRIVER_H

#include "device_driver/eci/EciDemo113.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>

#include <vector>

void init_hardware();

std::vector<double> read_position();

std::vector<double> read_velocity();

std::vector<double> read_torque();

void write_position(std::vector<double> data);


#endif