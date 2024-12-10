/******************************************************************************
* File Name: main.c
*
* Description: This is the source code that demonstrates the operation of the
* EZI2C resource in deepsleep mode.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cybsp.h"
#include "cy_pdl.h"
#include "stdio.h"
#include "mtb_hal.h"
#include "cy_retarget_io.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define DS_I2C_BUFFER_SIZE                   (6UL)
#define DS_I2C_SUCCESS                       (0UL)
#define DS_I2C_FAILURE                       (1UL)
/* The priority should always less than or equal to 253 */
#define CALLBACK_PRIORITY                    (253)
#define I2C_PRIORITY                         (3U)
#define DELAY                                (2000)

/*******************************************************************************
* Variable Definitions
*******************************************************************************/
uint8_t ds_i2c_buffer[DS_I2C_BUFFER_SIZE] = {0};
cy_stc_scb_i2c_context_t ds_i2c_context;
cy_stc_syspm_callback_params_t syspm_deep_sleep_params;

/* For the Retarget -IO (Debug UART) usage */
static cy_stc_scb_uart_context_t    DEBUG_UART_context;           /** UART context */
static mtb_hal_uart_t               DEBUG_UART_hal_obj;           /** Debug UART HAL object  */

cy_stc_syspm_callback_t syspm_deep_sleep_cb_handler =
{
    Cy_SCB_I2C_DeepSleepCallback,
    CY_SYSPM_DEEPSLEEP,
    0u,
    &syspm_deep_sleep_params,
    NULL,
    NULL,
    CALLBACK_PRIORITY
};

cy_stc_sysint_t ds_i2c_irq_config =
{
    .intrSrc = (IRQn_Type)DS_I2C_IRQ,
    .intrPriority = I2C_PRIORITY
};

/*******************************************************************************
*        Function Prototypes
*******************************************************************************/
void handle_error(uint32_t status);
void i2c_interrupt_handler(void);
uint32_t i2c_initializeSlave(void);

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  uint32_t status - status indicates success or failure
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(uint32_t status)
{
    if (status != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
}

/*******************************************************************************
* Function Name: InterruptHandler
****************************************************************************//**
* Summary:
* This function executes interrupt service routine.
*
* Parameters:
* void
*
* Return:
* void
*
*******************************************************************************/
void i2c_interrupt_handler(void)
{
    Cy_SCB_I2C_Interrupt(DS_I2C_HW, &ds_i2c_context);
}

/*******************************************************************************
* Function Name: initializeSlave
********************************************************************************
* Summary:
* This function initiates and enables the slave SCB
*
* Parameters:
* void
*
* Return:
* Status
*
*******************************************************************************/
uint32_t i2c_initializeSlave(void)
{
    cy_en_scb_i2c_status_t dsI2Cstatus;
    cy_en_sysint_status_t sysDsI2Cstatus;

    /*Initialize I2C slave.*/
    dsI2Cstatus =Cy_SCB_I2C_Init(DS_I2C_HW, &DS_I2C_config, &ds_i2c_context);
    if(dsI2Cstatus!=CY_SCB_I2C_SUCCESS)
    {
        return DS_I2C_FAILURE;
    }
    sysDsI2Cstatus = Cy_SysInt_Init(&ds_i2c_irq_config, &i2c_interrupt_handler);
    if(sysDsI2Cstatus!=CY_SYSINT_SUCCESS)
    {
        return DS_I2C_FAILURE;
    }

    NVIC_EnableIRQ((IRQn_Type)ds_i2c_irq_config.intrSrc);
    /* Configure buffer for communication with master. */
    Cy_SCB_I2C_SlaveConfigWriteBuf(DS_I2C_HW, ds_i2c_buffer, DS_I2C_BUFFER_SIZE,
                                      &ds_i2c_context);

    /* Enable SCB for the I2C operation. */
    Cy_SCB_I2C_Enable(DS_I2C_HW);
    return DS_I2C_SUCCESS;
}


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function and entry point to the application.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    uint32_t status = DS_I2C_SUCCESS;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    handle_error(result);

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = (cy_rslt_t)Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);

    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);

    /* HAL UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);

    /* HAL retarget_io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    printf("******************* I2C Deep Sleep **********************\r\n");

    /*Initiate and enable Slave SCB*/
    status = i2c_initializeSlave();
    if(status == DS_I2C_FAILURE)
    {
        CY_ASSERT(0);
    }
    printf("Slave initialization done.\r\n");

    syspm_deep_sleep_params.base = DS_I2C_HW;
    syspm_deep_sleep_params.context = &ds_i2c_context;

    Cy_SysPm_RegisterCallback(&syspm_deep_sleep_cb_handler);
    Cy_SysPm_SetDeepSleepMode(CY_SYSPM_MODE_DEEPSLEEP);

    for (;;)
    {
        printf("Entering into Deep Sleep\r\n");
        Cy_SysLib_Delay(DELAY);
        Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
        printf("WakeUp from DeepSleep\r\n");
        Cy_SysLib_Delay(DELAY);
    }
}

/* [] END OF FILE */
