/**
 * @file AD5592.h
 * @author Daniel (dzalf) based on /github.com/Metaln00b version
 * @brief Class abstraction layer for the AD5592R 12-bit ADC/DAC/Port Expander
 * @version and
 * @date 0.1-alpha  May 2020 - Converted implementation into a Class
 *       0.2        Sep 2020 - Added more applicability examples
 *       0.3        Jan 2021 - Tested on Teensy 3.6
 *       0.4        ??       - Long hiatus
 *       0.5        Mar 2023 - Updated, clenaed it and commited to GH
 *
 * @copyright Copyright (c) 2023
 */

#pragma once

#ifndef AD5592R_H
#define AD5592R_H

#include "definitions.h"
#include <Arduino.h>
#include <SPI.h>

// Members
typedef uint16_t                    AD5592R_word;

/*******************************************************************************
 * Datatypes                                                                   *
 ******************************************************************************/

/* Data registers content */
typedef enum {
    AD5592R_GPIO_DIRECTION_IN,
    AD5592R_GPIO_DIRECTION_OUT
} AD5592R_gpio_direction_t;

/* Pinout types */
typedef enum {
    AD5592R_IO_TYPE_ADC,
    AD5592R_IO_TYPE_DAC,
    AD5592R_IO_TYPE_GPIO
} AD5592R_io_type_t;

/* GPIO types */
typedef enum {
    AD5592R_GPIO_TYPE_PUSH_PULL,
    AD5592R_GPIO_TYPE_OPEN_DRAIN
} AD5592R_gpio_type_t;

/* GPIO states */
typedef enum {
    AD5592R_GPIO_LOW,
    AD5592R_GPIO_HIGH,
    AD5592R_GPIO_Z
} AD5592R_gpio_state_t;

/* Source of reference-voltage */
typedef enum {
    AD5592R_VREF_SOURCE_INTERNAL,
    AD5592R_VREF_SOURCE_EXTERNAL
} AD5592R_vref_source_t;


/* LDAC modes */
typedef enum {
    /* Data written to an input register is immediately copied to a
     * DAC register and the DAC output is updated (default). */
    AD5592R_LDAC_MODE_IMMEDIATELY,

    /* Data written to an input register is not copied to a DAC register.
     * The DAC output is not updated. */
    AD5592R_LDAC_MODE_ADD,

    /* The data in the input registers are copied to the corresponding DAC registers.
     * After transferring the data, the DAC outputs are updated simultaneously. */
    AD5592R_LDAC_MODE_TRANSFER
} AD5592R_ldac_mode_t;

/* PIN configuration */
typedef struct {
    /* OTHER */
    AD5592R_io_type_t type;

    /* GPIO */
    struct {
        AD5592R_gpio_type_t type;
        AD5592R_gpio_state_t state;
        AD5592R_gpio_direction_t direction;
        bool pull_down;
    } gpio;

    /* DAC */
    struct {
        uint16_t sample;
    } dac;
    
} AD5592R_io_setting_t;


class AD5592R
{
    public:

        AD5592R(uint8_t pinSS);	// Constructor

        /*******************************************************************************
         * Initialization / De-initialization                                          *
         *******************************************************************************/

        /**
         * AD5592R-object initialization
         * @param obj: Pointer to object
         */
        void init();

        /**
         * Reset
         * @param obj: Pointer to object
         */
        void reset();


        /*******************************************************************************
         * Primary configuration                                                       *
         ******************************************************************************/

        /**
         * Stores the voltage (voltage_mV) as input voltage, which may be maximum 5500mV.
         * @param obj: Pointer to object
         * @param voltage_mV: Voltage applied to (Vdd)
         * @return true if successful, otherwise false
         */
        bool setup_supply_voltage_mV(double voltage_mV);

        /**
         * If the internal reference voltage is selected, this will be applied to pin (Vref).
         * If the external reference voltage is selected, an external reference voltage should be applied to pin (Vref).
         * Important: As soon as the internal reference is selected, 2500mV will be used as
         * reference voltage, no matter what input voltage is applied to the AD5592R!
         * @param obj: Pointer to object
         * @param source: Reference voltage source (INTERNAL, EXTERNAL)
         */
        void setup_source_ref_set(AD5592R_vref_source_t source);

        /**
         * Stores the voltage (voltage_mV) as external reference voltage.
         * @param obj: Pointer to object
         * @param voltage_mV: Reference voltage (EXTERNAL)
         * @return true if successful, otherwise false
         */
        bool setup_external_ref_voltage_mV(double voltage_mV);

        /**
         * Returns the source of the reference voltage.
         * @param obj: Pointer to object
         * @return Reference voltage source (INTERNAL, EXTERNAL)
         */
        AD5592R_vref_source_t setup_source_ref_get();

        /**
         * Sets the output value range of the ADC and DAC to twice the reference voltage (0 - 2xVref).
         * @param obj: Pointer to object
         * @param double_vref: State double reference voltage (false=0V to Vref;true=0V to 2xVref)
         */
        void setup_double_vref_set(bool double_vref);

        /**
         * Returns the state of the double reference voltage of the ADC.
         * @param obj: Pointer to object
         * @return true if (0 - 2xVref), false if (0 - Vref)
         */
        bool setup_double_vref_adc_get();

        /**
         * Returns the state of the double reference voltage of the DAC.
         * @param obj: Pointer to object
         * @return true if (0 - 2xVref), false if (0 - Vref)
         */
        bool setup_double_vref_dac_get();


        /*******************************************************************************
         * PIN configuration                                                           *
         ******************************************************************************/

        /**
         * Determines how data written to an input register of a DAC is written/handled.
         * @param obj: Pointer to object
         * @param ldac_mode: LDAC-mode (IMMEDIATELY, ADD, TRANSFER)
         */
        void setup_ldac(AD5592R_ldac_mode_t ldac_mode);

        /**
         * Defines (pins) as DAC.
         * @param obj: Pointer to object
         * @param pins: Use MACRO AD5592R_IO(pin)
         */
        void setup_dac_all_pins(uint8_t pins);

        /**
         * Defines (pin) as DAC.
         * @param obj: Pointer to object
         * @param pin: Use MACRO AD5592R_IO(pin)
         */
        void setup_dac(uint8_t pin);

        /**
         * Defines (pins) as ADC.
         * @param obj: Pointer to object
         * @param pins: Use MACRO AD5592R_IO(pin)
         */
        void setup_adc_all( uint8_t pins);

        /**
         * Defines (pin) as ADC.
         * @param obj: Pointer to object
         * @param pin: Use MACRO AD5592R_IO(pin)
         */
        void setup_adc(uint8_t pin);

        /**
         * Defines the (type) of (pins).
         * @param obj: Pointer to object
         * @param pins: Use MACRO AD5592R_IO(pin)
         * @param type: Type (AD5592R_GPIO_TYPE_PUSH_PULL, AD5592R_GPIO_TYPE_OPEN_DRAIN)
         */
        void gpio_setup_type_all(uint8_t pins, AD5592R_gpio_type_t type);

        /**
         * Defines the (type) of (pins).
         * @param obj: Pointer to object
         * @param pin: Use MACRO AD5592R_IO(pin)
         * @param type: Type (AD5592R_GPIO_TYPE_PUSH_PULL, AD5592R_GPIO_TYPE_OPEN_DRAIN)
         */
        void gpio_setup_type(uint8_t pin, AD5592R_gpio_type_t type);

        /**
         * Defines (pins) as pulldown pins.
         * @param obj: Pointer to object
         * @param pins: Use MACRO AD5592R_IO(pin)
         * @param state: true if pulldown, false if not
         */
        void gpio_pulldown_all_set(uint8_t pins, bool state);

        /**
         * Defines (pin) as pulldown pin.
         * @param obj: Pointer to object
         * @param pin: Use MACRO AD5592R_IO(pin)
         * @param state: true if pulldown, false if not
         */
        void gpio_pulldown_set(uint8_t pin, bool state);

        /**
         * Defines the direction of (pins).
         * @param obj: Pointer to object
         * @param pins: Use MACRO AD5592R_IO(pin)
         * @param direction: Direction (AD5592R_GPIO_DIRECTION_IN, AD5592R_GPIO_DIRECTION_OUT)
         */
        void gpio_direction_all_set(uint8_t pins, AD5592R_gpio_direction_t direction);

        /**
         * Defines the direction of (pin).
         * @param obj: Pointer to object
         * @param pin: Use MACRO AD5592R_IO(pin)
         * @param direction: Direction (AD5592R_GPIO_DIRECTION_IN, AD5592R_GPIO_DIRECTION_OUT)
         */
        void gpio_direction_set(uint8_t pin, AD5592R_gpio_direction_t direction);


        /**
         * Sets the state from (pins) to (state).
         * FIXME: Do not set I/O0 as tri-state, otherwise it is no longer possible
         * to read out the ADC register reliably.
         * @param obj: Pointer to object
         * @param pins: Use MACRO AD5592R_IO(pin)
         * @param state: State of the digital output (LOW, HIGH, Z)
         */
        void gpio_state_all_set(uint8_t pins, AD5592R_gpio_state_t state);

        /**
         * Sets the state from (pin) to (state).
         * FIXME: Do not set I/O0 as tri-state, otherwise it is no longer possible
         * to read out the ADC register reliably.
         * @param obj: Pointer to object
         * @param pin: Use MACRO AD5592R_IO(pin)
         * @param state: State of the digital output (LOW, HIGH, Z)
         */
        void gpio_state_set(uint8_t pin, AD5592R_gpio_state_t state);

        /**
         * Returns the input state of (pins) as a sample.
         * @param obj: Pointer to object
         * @param pins: Use MACRO AD5592R_IO(pin)
         * @return 1 if HIGH, 0 if LOW
         */
        uint8_t gpio_input_state_get(uint8_t pins);

        /**
         * Returns the configured output-side state of (pin).
         * @param obj: Pointer to object
         * @param pin: Use MACRO AD5592R_IO(pin)
         * @return 1 if HIGH, 0 if LOW
         */
        uint8_t gpio_output_state_get(uint8_t pin);

        /**
         * Returns the sample from (pin) of the ADC.
         * @param obj: Pointer to object
         * @param pin: Use MACRO AD5592R_IO(pin)
         * @return Sample of the ADC
         */
        uint16_t adc_sample_get(uint8_t pin);

        /**
         * Returns the analog value (voltage_mV) of (pin) of the ADC.
         * @param obj: Pointer to object
         * @param pin: Use MACRO AD5592R_IO(pin)
         * @return Analog value (voltage_mV) of the ADC
         */
        double adc_voltage_get_mV(uint8_t pin);

        /**
         * Returns the percentage value of the voltage of (pin) of the ADC,
         * depending on the input voltage (Vdd).
         * @param obj: Pointer to object
         * @param pin: Use MACRO AD5592R_IO(pin)
         * @return Percentage value
         */
        unsigned int adc_percentage_get(uint8_t pin);

        /**
         * Sets to (pin) the sample (sample).
         * @param obj: Pointer to object
         * @param pin: Use MACRO AD5592R_IO(pin)
         * @param sample: 12-Bit sample
         * @return true if successful, otherwise false
         */
        bool dac_sample_set(uint8_t pin, uint16_t sample);

        /**
         * Applies the voltage (voltage_mV) to (pin).
         * @param obj: Pointer to object
         * @param pin: Use MACRO AD5592R_IO(pin)
         * @param voltage_mV: Voltage in mV
         * @return true if successful, otherwise false
         */
        bool dac_voltage_set_mV(uint8_t pin, double voltage_mV);

        /**
         * Specifies the voltage in percent (percentage) at (pin).
         * The voltage is applied as a percentage of the input voltage.
         * @param obj: Pointer to object
         * @param pin: Use MACRO AD5592R_IO(pin)
         * @param percantage: Percentage
         * @return true if successful, otherwise false
         */
        bool dac_percentage_set(uint8_t pin, unsigned int percentage);

        /**
         * Returns the last set sample from (pin) of the DAC from the shared memory.
         * @param obj: Pointer to object
         * @param pin: Use MACRO AD5592R_IO(pin)
         * @return Sample of the DAC
         */
        uint16_t dac_sample_get(uint8_t pin);


        /*******************************************************************************
         * Miscellaneous functions                                                     *
         ******************************************************************************/

        /**
         * Returns the temperature of the internal temperature sensor from the AD5592R.
         * @param obj: Pointer to object
         * @return Temperature in degrees Celsius
         */
        double temperature_get_degC();

        /**
         * Returns the temperature of the internal temperature sensor from the AD5592R.
         * @param obj: Pointer to object
         * @return Temperature as sample
         */
        uint16_t temperature_sample_get();

        /**
         * Converts the AD5592R typical sample of temperature to degC.
         * @param obj: Pointer to object
         * @param temperature_sample: Temperature as sample
         * @return Temperature in degrees Celsius
         */
        double sample_to_temperature_degC(uint16_t temperature_sample);

    private:

        // Members
        uint8_t _syncPin; // Added the option to set your own SS pin --> github.com/dzalf  On May 2020
    
        AD5592R_vref_source_t _vref_source;     // Internal value
        bool _double_vref_adc;
        bool _double_vref_dac;


        AD5592R_ldac_mode_t _ldac_mode;

        /* Voltage applied to pin (Vdd) */
        double _supply_voltage_mV;

        /* Voltage applied to pin (Vref) when an external reference voltage is used. */
        double _external_ref_voltage_mV;

        AD5592R_io_setting_t _io_setting[AD5592R_CHANNEL_COUNT];

        /* Register readback addresses */
        typedef enum {
            AD5592_REG_READBACK_NOP,
            AD5592_REG_READBACK_DAC_READBACK,
            AD5592_REG_READBACK_ADC_SEQ,
            AD5592_REG_READBACK_GPIO_CONF,
            AD5592_REG_READBACK_ADC_PIN_CONF,
            AD5592_REG_READBACK_DAC_PIN_CONF,
            AD5592_REG_READBACK_PULL_DOWN_CONF,
            AD5592_REG_READBACK_LDAC_CONF,
            AD5592_REG_READBACK_GPIO_WRITE_CONF,
            AD5592_REG_READBACK_GPIO_WRITE_DATA,
            AD5592_REG_READBACK_GPIO_READ_CONF,
            AD5592_REG_READBACK_PWR_DWN_AND_REF_CNTRL,
            AD5592_REG_READBACK_OPEN_DRAIN_CONF,
            AD5592_REG_READBACK_TRI_STAT_CONF,
            AD5592_REG_READBACK_RESERVED,
            AD5592_REG_READBACK_SW_RESET
        } AD5592R_reg_readback_t;
        
        /*******************************************************************************
         * Primary configuration                                                       *
         ******************************************************************************/

        /**
         * Reads out the register (reg). Helpful to ensure correct configuration.
         * @param reg: Register
         * @return Value of the register
         */
        uint8_t register_readback(AD5592R_reg_readback_t reg);

        /*******************************************************************************
         * Communication                                                               *
         ******************************************************************************/

        /**
         * Sends the data (sixteen_bits) and references the data received on it to (rx_data).
         * @param obj: Pointer to object
         * @param sixteen_bits: 16-Bit word
         * @param rx_data: Pointer to receive data
         * @return true if successful, otherwise false
         */
        bool comm(AD5592R_word sixteen_bits, uint8_t *rx_data);

        /*******************************************************************************
         * Auxiliary functions                                                         *
         ******************************************************************************/

        /**
         * Converts an analog value to a digital value and returns it.
         * @param obj: Pointer to object
         * @param voltage_mV: Voltage in mV
         * @param double_vref: true if calculated with double reference voltage,
         * false if once.
         * @return Decimal value 0-4095
         */
        uint16_t analog_2_digital(double voltage_mV, bool double_vref);

        /**
         * Converts a digital value to an analog value and returns it.
         * @param obj: Pointer to object
         * @param count: Decimal value 0-4095
         * @param double_vref: true if calculated with double reference voltage,
         * false if once.
         * @return Voltage in mV
         */
        double digital_2_analog(uint16_t sample, bool double_vref);

        /**
         * Converts a percentage value to a digital value and returns it.
         * @param percentage: Percentage
         * @return Decimal value 0-4095
         */
        uint16_t percentage_2_digital(unsigned int percentage);

        /**
         * Converts a percentage value to an analog value (mV) and returns it.
         * The return value depends on the configured input voltage (Vdd).
         * @param percentage: Percentage
         * @return Analog value 0-Vdd in mV
         */
        double percentage_2_analog(unsigned int percentage);

        /**
         * Converts the analog value (voltage_mV) into a percentage value and returns it.
         * The return value depends on the configured input voltage (Vdd).
         * @param voltage_mV: Voltage in mV
         * @return Percentage
         */
        unsigned int analog_2_percentage(double voltage_mV);

        /**
         * Converts the 16 bit word (sixteen_bits) into two 8 bit chunks (eight_bits[]).
         * @param eight_bits[]: 8-bit buffer
         * @param sixteen_bits: 16-bit AD5592R word
         * @param arr_size: Size of (eight_bits[])
         */
        void split_word(uint8_t eight_bits[], AD5592R_word sixteen_bits, size_t arr_size);

        /**
         * Converts the value of the pin macro into a pin number.
         * @param macro: Use MACRO AD5592R_IO(pin)
         * @return Pin (0-7)
         */
        uint8_t macro_2_pin(uint8_t macro);
};

#endif