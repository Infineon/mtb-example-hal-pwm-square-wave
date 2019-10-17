/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the TCPWM Square Wave code example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*******************************************************************************
* (c) 2019, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/


#include "cybsp.h"
#include "cyhal.h"


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
* This is the main function for CM4 CPU. It configures the TCPWM block to
* operate as a PWM and puts the CPU in Sleep mode to save power.
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

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        /* Halt the CPU while debugging */
        CY_ASSERT(false);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Configure the TCPWM resource for PWM operation.
    In this example, PWM output is routed to the user LED on the kit.
    See PSoC 6 HAL API Reference document for API details. */

    /* Initialize the TCPWM resource for PWM operation */
    result = cyhal_pwm_init(&pwm_led_control, (cyhal_gpio_t)CYBSP_USER_LED, NULL);
    if(result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(false);
    }
    /* Set the PWM output frequency and duty cycle */
    result = cyhal_pwm_set_duty_cycle(&pwm_led_control, PWM_DUTY_CYCLE, PWM_FREQUENCY);
    if(result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(false);
    }
    /* Start the PWM */
    result = cyhal_pwm_start(&pwm_led_control);
    if(result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(false);
    }

    /* Loop infinitely */
    for(;;)
    {
        /* Put the CPU into Sleep mode to save power */
        while(cyhal_system_sleep() != CY_RSLT_SUCCESS);
    }
}


/* [] END OF FILE */
