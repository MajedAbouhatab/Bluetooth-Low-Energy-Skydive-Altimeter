//-----------------------------------------------------------------------------
// Copyright (c) 2018 Semiconductor Components Industries LLC
// (d/b/a "ON Semiconductor").  All rights reserved.
// This software and/or documentation is licensed by ON Semiconductor under
// limited terms and conditions.  The terms and conditions pertaining to the
// software and/or documentation are available at
// http://www.onsemi.com/site/pdf/ONSEMI_T&C.pdf ("ON Semiconductor Standard
// Terms and Conditions of Sale, Section 8 Software") and if applicable the
// software license agreement.  Do not use this software and/or documentation
// unless you have carefully read and you agree to the limited terms and
// conditions.  By using this software and/or documentation, you agree to the
// limited terms and conditions.
//-----------------------------------------------------------------------------
#include <stdio.h>

#include <BDK.h>
#include <BSP_Components.h>
#include <BLE_Components.h>

#include <ics/CS.h>
#include <ics/CS_Nodes.h>
#include <BitBanging.h>

int main(void)
{
    /* Initialize all needed BDK components. */
    BDK_Initialize();

    /* Initialize all LEDs. */
    LED_Initialize(LED_RED);
    LED_Initialize(LED_GREEN);
    LED_Initialize(LED_BLUE);

    /* Indication - Initialization started. */
    LED_On(LED_BLUE);

    /* Initialize CS protocol, start Peripheral Server, Add Custom Service
     * Profile to it.
     */
    CS_Init();

    /* Optionally set advertising interval to be 200 to 250 ms long. */
    BDK_BLE_SetAdvertisementInterval(320, 400);

    /* Optionally set custom device name.
     * The name needs to contain one of 'IDK', 'BDK', 'RSL10' or 'BLE_Terminal'
     * patterns to be recognized by RSL10 Sense & control mobile application.
     * Default: 'HB_BLE_Terminal'
     */
     BDK_BLE_SetLocalName("HB_BLE_Terminal");

    /* Also add battery service if its RTE component is enabled. */
#if defined (RTE_BDK_BLE_PERIPHERAL_SERVER_BASS)
    BLE_BASS_Initialize(1000, 16);
    BLE_BASS_SetVoltageRange(CALC_VBAT_MEASURED(2.4f), CALC_VBAT_MEASURED(3.0f));
    // BLE_BASS_SetBattLevelInd(BattLevelChangeCallback);
#endif /* RTE_BDK_BLE_BASS_PRESENT */

    // Change default bus speed to 400kHz
    HAL_I2C_SetBusSpeed(HAL_I2C_BUS_SPEED_FAST);

    /* Add all enabled Custom Service Nodes. */
    ASSERT_ALWAYS(CSN_ENV_CheckAvailability() == true);
    ASSERT_ALWAYS(CS_RegisterNode(CSN_ENV_Create()) == CS_OK);

    BitBanging_Init();
    /* Indication - Initialization complete. */
    LED_Off(LED_BLUE);

    CS_SYS_Info("Entering main loop.");
    while (1)
    {
        /* Execute any events that have occurred and refresh Watchdog. */
        BDK_Schedule();

        SendAltitudeToDisplay();

        /* Enter sleep mode until an interrupt occurs. */
        SYS_WAIT_FOR_INTERRUPT;
    }

    return 0;
}
