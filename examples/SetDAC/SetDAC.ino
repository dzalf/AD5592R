
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

    DAC.setup_dac_all_pins(AD5592R_IO(0));
    DAC.setup_ldac(AD5592R_LDAC_MODE_IMMEDIATELY);

}

void loop() {

    if (Serial.available() > 0) {

        double voltage_mV_set = Serial.parseInt();

        Serial.print("You send: ");
        Serial.print(voltage_mV_set);
        Serial.println();
        
        DAC.dac_voltage_set_mV(AD5592R_IO(0), voltage_mV_set);

    }
}