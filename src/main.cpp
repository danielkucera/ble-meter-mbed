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
#include <rtos.h>
#include "mbed.h"
#include "ble/BLE.h"
#include "ble/services/HealthThermometerService.h"

DigitalOut led1(P0_21, 1);
DigitalOut btn_pwr(P0_22, 1);
InterruptIn pulse(P0_23);

const static char     DEVICE_NAME[]        = "GasMeter";
long i = 0;
char mfgData[30] = "{}";
int mfgDataLen = sizeof(mfgData);
int debouncing = 0;

static EventQueue eventQueue(/* event count */ 16 * EVENTS_EVENT_SIZE);
int stopEvent = 0;

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
      wait(10 * 0.001);
      debouncing = 0;
    }

    BLE &ble = BLE::Instance();

    i++;
    mfgDataLen = sprintf(mfgData, "{ data: %ld }", i);

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
    pulse.rise(&pulseHandler); //it's easier to debunce than making RC filter

    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(scheduleBleEventsProcessing);
    ble.init(bleInitComplete);

    eventQueue.dispatch_forever();

    return 0;
}
