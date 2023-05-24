///
// system Fault handlers
//

#include "main.h"

void     NMI_Handler(void)
{
  DEBUG_BREAK();

  exit(ERROR);
}

void HardFault_Handler(void)
{
  DEBUG_BREAK();

  exit(ERROR);
}

