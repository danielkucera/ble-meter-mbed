/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <mbed_events.h>
#include "mbed.h"
#include "ble/BLE.h"

DigitalOut led1(P0_21, 1);
DigitalOut btn_pwr(P0_22, 1);
InterruptIn pulse(P0_23);

//#define DEBOUCE_MS 10
#define DEBOUCE_MS 0
//#define COUNT_FALLS 1
#define SEND_EACH_NTH 1

const static char     DEVICE_NAME[]        = "SmartMeter";
long i = 0;
char mfgData[30] = "{}";
int mfgDataLen = sizeof(mfgData);
int debouncing = 0;

static EventQueue eventQueue(/* event count */ 16 * EVENTS_EVENT_SIZE);
int stopEvent = 0;

void my_analogin_init(void)
{
    NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
    NRF_ADC->CONFIG = (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) |
                      (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                      (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
                      (ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos) |
                      (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
}

uint16_t my_analogin_read_u16(void)
{
    NRF_ADC->CONFIG     &= ~ADC_CONFIG_PSEL_Msk;
    NRF_ADC->CONFIG     |= ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos;
    NRF_ADC->TASKS_START = 1;
    while (((NRF_ADC->BUSY & ADC_BUSY_BUSY_Msk) >> ADC_BUSY_BUSY_Pos) == ADC_BUSY_BUSY_Busy) {};
    return (uint16_t)NRF_ADC->RESULT; // 10 bit
}

void onBleInitError(BLE &ble, ble_error_t error)
{
   /* Initialization error handling should go here */
}

void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{
    BLE&        ble   = params->ble;
    ble_error_t error = params->error;

    if (error != BLE_ERROR_NONE) {
        onBleInitError(ble, error);
        return;
    }

    if (ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {
        return;
    }

    /* setup advertising */
    ble.gap().setAdvertisingInterval(100); /* ms */
    //ble.gap().startAdvertising();
}

void scheduleBleEventsProcessing(BLE::OnEventsToProcessCallbackContext* context) {
    BLE &ble = BLE::Instance();
    eventQueue.call(Callback<void()>(&ble, &BLE::processEvents));
}

void stopAdvertising(){
  stopEvent = 0;
  BLE &ble = BLE::Instance();

  ble.gap().stopAdvertising();
  ble.gap().clearAdvertisingPayload();
  led1.write(0);
}

void pulseHandler(void)
{
    if (debouncing) {
      return;
    } else {
      debouncing = 1;
      wait(DEBOUCE_MS * 0.001);
      debouncing = 0;
    }

    // increase pulse counter
    i++;

    if (i % SEND_EACH_NTH != 0)
      return;

    BLE &ble = BLE::Instance();

    ble.gap().setTxPower(4);

    float batV = ((float)my_analogin_read_u16() * 3.6) / 1024.0;

    mfgDataLen = sprintf(mfgData, "{ \"d\": %ld, \"v\": %d }", i, (int)(batV*1000));

    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::MANUFACTURER_SPECIFIC_DATA, (uint8_t *)mfgData, mfgDataLen );
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));

    ble.gap().startAdvertising();
    led1.write(1);

    if (stopEvent){
        eventQueue.cancel(stopEvent);
    }
    stopEvent = eventQueue.call_in(1000, stopAdvertising);
}

int main()
{
    //eventQueue.call_every(4000, periodicCallback);

    pulse.mode(PullDown);
    wait(.001);
    pulse.fall(&pulseHandler);
#ifdef COUNT_FALLS
    pulse.rise(&pulseHandler); //it's easier to debunce than making RC filter
#endif

    my_analogin_init();

    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(scheduleBleEventsProcessing);
    ble.init(bleInitComplete);

    eventQueue.dispatch_forever();

    return 0;
}
