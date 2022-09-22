/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PWM Square Wave code example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2019-2022, Cypress Semiconductor Corporation (an Infineon company) or
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


#include "cybsp.h"
#include "cyhal.h"
#include "cy_retarget_io.h"


/*******************************************************************************
* Macros
*******************************************************************************/
/* PWM Frequency = 2Hz */
#define PWM_FREQUENCY (2u)
/* PWM Duty-cycle = 50% */
#define PWM_DUTY_CYCLE (50.0f)


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for the CPU. It configures the PWM and puts the CPU
* in Sleep mode to save power.
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
    /* PWM object */
    cyhal_pwm_t pwm_led_control;
    /* API return code */
    cy_rslt_t result;

#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;
    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (CY_RSLT_SUCCESS != result)
    {
        /* Halt the CPU while debugging */
        CY_ASSERT(false);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize the retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    if (CY_RSLT_SUCCESS != result)
    {
        /* Halt the CPU while debugging */
        CY_ASSERT(false);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("****************** HAL Code: PWM Square Wave ******************\r\n\n");

    /* In this example, PWM output is routed to the user LED on the kit.
       See HAL API Reference document for API details. */

    /* Initialize the PWM */
    result = cyhal_pwm_init(&pwm_led_control, CYBSP_USER_LED, NULL);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("API cyhal_pwm_init failed with error code: %lu\r\n", (unsigned long) result);
        CY_ASSERT(false);
    }
    /* Set the PWM output frequency and duty cycle */
    result = cyhal_pwm_set_duty_cycle(&pwm_led_control, PWM_DUTY_CYCLE, PWM_FREQUENCY);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("API cyhal_pwm_set_duty_cycle failed with error code: %lu\r\n", (unsigned long) result);
        CY_ASSERT(false);
    }
    /* Start the PWM */
    result = cyhal_pwm_start(&pwm_led_control);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("API cyhal_pwm_start failed with error code: %lu\r\n", (unsigned long) result);
        CY_ASSERT(false);
    }

    printf("PWM started successfully. Entering the sleep mode...\r\n");

    /* Loop infinitely */
    for (;;)
    {
        /* Put the CPU into sleep mode to save power */
        cyhal_syspm_sleep();
    }
}


/* [] END OF FILE */
