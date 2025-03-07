#pragma once

#include <stdint.h>

int getPhase();
int getPreamble();

void setupPhoto();
int receive();

int receivedFrames();
void getFrames(uint16_t *frames);
