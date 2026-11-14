/*
 * Passerelle Wi-Fi 6E des PAMI d'attaque : Ezurio Veda IF913 (Infineon
 * CYW55913), reliée en UART à l'ESP32-S3 du PAMI. Voir README.md.
 */
#include <stdio.h>

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "bridge.h"

int main(void)
{
    cy_rslt_t result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* printf -> UART de debug du kit (USB, 115200 bauds) */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                 CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    printf("\n=== Passerelle Wi-Fi 6E PAMI (Veda IF913) ===\n");

    result = bridge_start();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    return 0;  /* comme l'exemple Infineon : les tâches continuent seules */
}
