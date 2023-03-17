#include <AD5592R.h>

const int syncPin = 10;

AD5592R DAC(syncPin);

void setup() {

    Serial.begin(115200);

    DAC.init();
    DAC.reset();
    DAC.setup_source_ref_set(AD5592R_VREF_SOURCE_INTERNAL);
    DAC.setup_double_vref_set(true);
    DAC.setup_supply_voltage_mV(3300);
    
}

void loop() {

    size_t iteration_size = 50;

    double temp[iteration_size];
    double temp_sum = 0;
    double temp_avg;

    for (size_t i = 0; i < iteration_size; i++) {

        temp[i] = DAC.temperature_get_degC();
        temp_sum += temp[i];

    }

    temp_avg = temp_sum / iteration_size;

    Serial.println(temp_avg, DEC);

    delay(100);

}