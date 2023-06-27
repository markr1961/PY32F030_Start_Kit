///
// system Fault handlers
//

#include "main.h"

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers         */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  DEBUG_BREAK();
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  DEBUG_BREAK();

  exit(ERROR);
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  DEBUG_BREAK();
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  DEBUG_BREAK();
}

