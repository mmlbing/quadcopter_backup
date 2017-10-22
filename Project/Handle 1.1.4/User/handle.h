#include "stm32f10x.h"
#include "nRF24L01.h"

#define LOCKED 		0
#define UNLOCKED	1

void TxConrtolData(void);
void TxConrtolCommandLock(void);
void TxConrtolCommand(void);

static void dataPrepare(void);
