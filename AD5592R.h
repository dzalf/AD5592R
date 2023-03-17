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
 * Datatypes                                                               *
 ******************************************************************************/

/* Inhalt Datenregister */

typedef enum {
    AD5592R_GPIO_DIRECTION_IN,
    AD5592R_GPIO_DIRECTION_OUT
} AD5592R_gpio_direction_t;

/* Definitionen der Pinout-Typen */
typedef enum {
    AD5592R_IO_TYPE_ADC,
    AD5592R_IO_TYPE_DAC,
    AD5592R_IO_TYPE_GPIO
} AD5592R_io_type_t;

/* Definitionen der GPIO-Typen */
typedef enum {
    AD5592R_GPIO_TYPE_PUSH_PULL,
    AD5592R_GPIO_TYPE_OPEN_DRAIN
} AD5592R_gpio_type_t;

/* Definitionen der GPIO-States */
typedef enum {
    AD5592R_GPIO_LOW,
    AD5592R_GPIO_HIGH,
    AD5592R_GPIO_Z
} AD5592R_gpio_state_t;

/* Definitionen von Quellen der Referenzspannung*/
typedef enum {
    AD5592R_VREF_SOURCE_INTERNAL,
    AD5592R_VREF_SOURCE_EXTERNAL
} AD5592R_vref_source_t;


/* Definitoion of LDAC modes */
typedef enum {
    /* Daten, die in ein Eingangsregister geschrieben werden, werden sofort in
     * ein DAC-Register kopiert und der DAC Ausgang wird aktualisiert (default). */
    AD5592R_LDAC_MODE_IMMEDIATELY,

    /* Daten, die in ein Eingangsregister geschrieben werden, werden nicht in ein DAC-Register kopiert.
     * Der DAC-Ausgang wird nicht aktualisiert. */
    AD5592R_LDAC_MODE_ADD,

    /* Die Daten in den Eingangsregistern werden in die entsprechenden DAC-Register kopiert.
     * Nach der Übertragung der Daten werden die DAC-Ausgänge gleichzeitig aktualisiert. */
    AD5592R_LDAC_MODE_TRANSFER
} AD5592R_ldac_mode_t;

/* Datentyp für Pin Einstellungen */
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


class AD5592R{
	
	public:
	
	AD5592R(uint8_t pinSS);	//Constructor
	

	/*******************************************************************************
 * Initialisierung / Deinitialisierung                                         *
 ******************************************************************************/

/**
 * Initialisierung von AD5592R-Objekt
 * @param obj: Zeiger auf Objekt
 */
void init();

/**
 * Zurücksetzen
 * @param obj: Zeiger auf Objekt
 */
void reset();


/*******************************************************************************
 * Allgemeine Konfiguration                                                    *
 ******************************************************************************/

/**
 * Speichert die Spannung (voltage_mV) als Eingangspannung, welche maximal 5500mV betragen darf.
 * @param obj: Zeiger auf Objekt
 * @param voltage_mV: Spannung welche an (Vdd) anliegt
 * @return true wenn erfolgreich, sonst false
 */
bool setup_supply_voltage_mV(double voltage_mV);

/**
 * Wird die interne Referenzspannung gewählt, wird diese an Pin (Vref) anliegen.
 * Wird die externe Referenzspannung gewählt, sollte eine externe Referenzspannung an Pin (Vref) anliegen.
 * Wichtig: Sobald die interne Referenz gewählt wird, werden 2500mV als
 * Referenzspannung verwendet, unabhängig davon, welche Eingangspannung am AD5592R anliegt!
 * @param obj: Zeiger auf Objekt
 * @param source: Quelle der Referenzspannung (INTERNAL, EXTERNAL)
 */
void setup_source_ref_set(AD5592R_vref_source_t source);

/**
 * Speichert die Spannung (voltage_mV) als externe Referenzsspannung.
 * @param obj: Zeiger auf Objekt
 * @param voltage_mV: Referenzspannung (EXTERNAL)
 * @return true wenn erfolgreich, sonst false
 */
bool setup_external_ref_voltage_mV(double voltage_mV);

/**
 * Gibt die Quelle der Referenzspannung zurück.
 * @param obj: Zeiger auf Objekt
 * @return Quelle der Referenzspannung (INTERNAL, EXTERNAL)
 */
AD5592R_vref_source_t setup_source_ref_get();

/**
 * Setzt den Ausgangswerte-Bereich des ADC und DAC auf die doppelte Referenzspannung (0 - 2xVref).
 * @param obj: Zeiger auf Objekt
 * @param double_vref: Zustand doppelte Referenzspannung (false=0V to Vref;true= 0V to 2xVref)
 */
void setup_double_vref_set(bool double_vref);

/**
 * Gibt den Zustand der doppelten Referenzspannung des ADC zurück.
 * @param obj: Zeiger auf Objekt
 * @return true wenn (0 - 2xVref), false wenn (0 - Vref)
 */
bool setup_double_vref_adc_get();

/**
 * Gibt den Zustand der doppelten Referenzspannung des DAC zurück.
 * @param obj: Zeiger auf Objekt
 * @return true wenn (0 - 2xVref), false wenn (0 - Vref)
 */
bool setup_double_vref_dac_get();


/*******************************************************************************
 * Pin Konfiguration                                                           *
 ******************************************************************************/

/**
 * Legt fest, wie Daten, die in ein Eingangsregister eines DAC geschrieben/behandelt werden.
 * @param obj: Zeiger auf Objekt
 * @param ldac_mode: LDAC-Modus (IMMEDIATELY, ADD, TRANSFER)
 */
void setup_ldac(AD5592R_ldac_mode_t ldac_mode);

/**
 * Definiert (pins) als DAC.
 * @param obj: Zeiger auf Objekt
 * @param pins: Verwendung von MACRO AD5592R_IO(pin)
 */
void setup_dac_all_pins(uint8_t pins);

/**
 * Definiert (pin) als DAC.
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 */
void setup_dac(uint8_t pin);

/**
 * Definiert (pins) als ADC.
 * @param obj: Zeiger auf Objekt
 * @param pins: Verwendung von MACRO AD5592R_IO(pin)
 */
void setup_adc_all( uint8_t pins);

/**
 * Definiert (pin) als ADC.
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 */
void setup_adc(uint8_t pin);

/**
 * Definiert den Typ (type) von (pins).
 * @param obj: Zeiger auf Objekt
 * @param pins: Verwendung von MACRO AD5592R_IO(pin)
 * @param type: Typ (AD5592R_GPIO_TYPE_PUSH_PULL, AD5592R_GPIO_TYPE_OPEN_DRAIN)
 */
void gpio_setup_type_all(uint8_t pins, AD5592R_gpio_type_t type);

/**
 * Definiert den Typ (type) von (pins).
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @param type: Typ (AD5592R_GPIO_TYPE_PUSH_PULL, AD5592R_GPIO_TYPE_OPEN_DRAIN)
 */
void gpio_setup_type(uint8_t pin, AD5592R_gpio_type_t type);

/**
 * Definiert (pins) als Pulldown-Pins.
 * @param obj: Zeiger auf Objekt
 * @param pins: Verwendung von MACRO AD5592R_IO(pin)
 * @param state: true wenn pulldown, false wenn nicht
 */
void gpio_pulldown_all_set(uint8_t pins, bool state);

/**
 * Definiert (pin) als Pulldown-Pins.
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @param state: true wenn pulldown, false wenn nicht
 */
void gpio_pulldown_set(uint8_t pin, bool state);

/**
 * Definiert die Richtung (direction) von (pins).
 * @param obj: Zeiger auf Objekt
 * @param pins: Verwendung von MACRO AD5592R_IO(pin)
 * @param direction: Richtung (AD5592R_GPIO_DIRECTION_IN, AD5592R_GPIO_DIRECTION_OUT)
 */
void gpio_direction_all_set(uint8_t pins, AD5592R_gpio_direction_t direction);

/**
 * Definiert die Richtung (direction) von (pin).
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @param direction: Richtung (AD5592R_GPIO_DIRECTION_IN, AD5592R_GPIO_DIRECTION_OUT)
 */
void gpio_direction_set(uint8_t pin, AD5592R_gpio_direction_t direction);


/**
 * Setzt den Zustand von (pins) auf (state).
 * FIXME I/O0 nicht als Tri-State setzen, sonst ist es nicht mehr möglich
 * das ADC-Register zuverlässig auszulesen.
 * @param obj: Zeiger auf Objekt
 * @param pins: Verwendung von MACRO AD5592R_IO(pin)
 * @param state: Zustand des Digitalausgangs (LOW, HIGH, Z)
 */
void gpio_state_all_set(uint8_t pins, AD5592R_gpio_state_t state);

/**
 * Setzt den Zustand von (pin) auf (state).
 * FIXME I/O0 nicht als Tri-State setzen, sonst ist es nicht mehr möglich
 * das ADC-Register zuverlässig auszulesen.
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @param state: Zustand des Digitalausgangs (LOW, HIGH, Z)
 */
void gpio_state_set(uint8_t pin, AD5592R_gpio_state_t state);

/**
 * Gibt den eingangsseitigen Zustand von (pins) als Sample zurück.
 * @param obj: Zeiger auf Objekt
 * @param pins: Verwendung von MACRO AD5592R_IO(pin)
 * @return 1 wenn HIGH, 0 wenn LOW
 */
uint8_t gpio_input_state_get(uint8_t pins);

/**
 * Gibt den konfigurierten, ausgangseitigen Zustand von (pin) zurück.
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @return 1 wenn HIGH, 0 wenn LOW
 */
uint8_t gpio_output_state_get(uint8_t pin);

/**
 * Gibt das Sample von (pin) des ADC zurück.
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @return Sample des ADC
 */
uint16_t adc_sample_get(uint8_t pin);

/**
 * Gibt den analogen Wert (voltage_mV) von (pin) des ADC zurück.
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @return Analogwert (voltage_mV) des ADC
 */
double adc_voltage_get_mV(uint8_t pin);

/**
 * Gibt den prozentualen Wert der Spannung von (pin) des ADC, in Abhängigkeit der
 * Eingangsspannung (Vdd) zurück.
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @return Prozentwert
 */
unsigned int adc_percentage_get(uint8_t pin);

/**
 * Setzt an (pin) das Sample (sample).
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @param sample: 12-Bit Sample
 * @return true wenn erfolgreich, sonst false
 */
bool dac_sample_set(uint8_t pin, uint16_t sample);

/**
 * Legt an (pin) die Spannung (voltage_mV) an.
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @param voltage_mV: Spannung in mV
 * @return true wenn erfolgreich, sonst false
 */
bool dac_voltage_set_mV(uint8_t pin, double voltage_mV);

/**
 * Legt an (pin) die Spannung in Prozent (percentage) an.
 * Die Spannung wird prozentual zur Eingangspannung angelegt.
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @param percantage: Prozentangabe
 * @return true wenn erfolgreich, sonst false
 */
bool dac_percentage_set(uint8_t pin, unsigned int percentage);

/**
 * Gibt den zuletzt eingestellten Sample von (pin) des DAC aus dem Shared-Memory zurück.
 * @param obj: Zeiger auf Objekt
 * @param pin: Verwendung von MACRO AD5592R_IO(pin)
 * @return Sample des DAC
 */
uint16_t dac_sample_get(uint8_t pin);


/*******************************************************************************
 * Sonstige                                                                    *
 ******************************************************************************/

/**
 * Gibt die Temperatur des internen Temperatursensors vom AD5592R zurück.
 * @param obj: Zeiger auf Objekt
 * @return Temperatur in Grad Celsius
 */
double temperature_get_degC();

/**
 * Gibt die Temperatur des internen Temperatursensors vom AD5592R zurück.
 * @param obj: Zeiger auf Objekt
 * @return Temperatur als sample
 */
uint16_t temperature_sample_get();

/**
 * Wandelt das AD5592R typische sample der Temperatur in degC um.
 * @param obj: Zeiger auf Objekt
 * @param temperature_sample: Temperatur als sample
 * @return Temperatur in Grad Celsius
 */
double sample_to_temperature_degC(uint16_t temperature_sample);

	
	
	private:

        // Members
        uint8_t _syncPin; // Added the option to set your own SS pin --> github.com/dzalf  On May 2020
    
        AD5592R_vref_source_t _vref_source;     // Internal value
        bool _double_vref_adc;
    bool _double_vref_dac;


    AD5592R_ldac_mode_t _ldac_mode;

    /* Spannung welche an Pin (Vdd) anliegt */
    double _supply_voltage_mV;

    /* Spannung welche an Pin (Vref) anliegt, wenn eine externe Referenzspannung verwendet wird */
    double _external_ref_voltage_mV;

    AD5592R_io_setting_t _io_setting[AD5592R_CHANNEL_COUNT];

    /* Definitionen der Register-Readback-Adressen */
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
	
	/**
 * Liest das Register (reg) aus. Hilfreich um korrekte Konfiguration sicherzustellen.
 * @param reg: Register
 * @return Wert des Registers
 */
uint8_t register_readback(AD5592R_reg_readback_t reg);

/**
 * Sendet die Daten (sixteen_bits) und verweist die darauf empfangenen Daten an (rx_data).
 * @param obj: Zeiger auf Objekt
 * @param sixteen_bits: 16-Bit Wort
 * @param rx_data: Zeiger auf Empfangs-Daten
 * @return true wenn erfolgreich, sonst false
 */
bool comm(AD5592R_word sixteen_bits, uint8_t *rx_data);

/**
 * Wandelt einen analogen Wert in einen digitalen Wert um und gibt diesen zurück.
 * @param obj: Zeiger auf Objekt
 * @param voltage_mV: Spannung in mV
 * @param double_vref: true, wenn mit doppelter Referenzspannung gerechnet werden soll,
 * false wenn einmal.
 * @return Dezimalwert 0-4095
 */
uint16_t analog_2_digital(double voltage_mV, bool double_vref);

/**
 * Wandelt einen digitalen Wert in einen analogen Wert um und gibt diesen zurück.
 * @param obj: Zeiger auf Objekt
 * @param count: Dezimalwert 0-4095
 * @param double_vref: true, wenn mit doppelter Referenzspannung gerechnet werden soll,
 * false wenn einmal.
 * @return Spannung in mV
 */
double digital_2_analog(uint16_t sample, bool double_vref);

/**
 * Wandelt einen prozentualen Wert in einen digitalen Wert um und gibt diesen zurück.
 * @param percentage: Prozentangabe
 * @return Dezimalwert 0-4095
 */
uint16_t percentage_2_digital(unsigned int percentage);

/**
 * Wandelt einen prozentualen Wert in einen analogen Wert (mV) um und gibt diesen zurück.
 * Der Rückgabewert ist abhängig von der konfigurierten Eingangspannung (Vdd).
 * @param percentage: Prozentangabe
 * @return Analogwert 0-Vdd in mV
 */
double percentage_2_analog(unsigned int percentage);

/**
 * Wandelt den analogen Wert (voltage_mV) in einen Prozentwert um und gibt diesen zurück.
 * Der Rückgabewert ist abhängig von der konfigurierten Eingangspannung (Vdd).
 * @param voltage_mV: Spannung in mV
 * @return Prozentwert
 */
unsigned int analog_2_percentage(double voltage_mV);

/**
 * Wandelt das 16 bit Wort (sixteen_bits) in zwei 8 bit Chunks (eight_bits[]).
 * @param eight_bits[]: 8-bit buffer
 * @param sixteen_bits: 16-bit AD5592R Wort
 */
void split_word(uint8_t eight_bits[], AD5592R_word sixteen_bits, size_t arr_size);

/**
 * Wandelt den Wert des Pin-Makros in eine Pin-Nummer um.
 * @param macro: Verwendung von MACRO AD5592R_IO(pin)
 * @return Pin (0-7)
 */
uint8_t macro_2_pin(uint8_t macro);
	
	

} ;



#endif