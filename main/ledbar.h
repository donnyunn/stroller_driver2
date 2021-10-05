#ifndef __LEDBAR_H
#define __LEDBAR_H

#include "main.h"

void ledbar_toggle(void);
void ledbar_on(void);
void ledbar_off(void);
void ledbar_blink_start(int num);
void ledbar_init(void);

#endif /* __LEDBAR_H */