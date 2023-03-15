
#include "AD5592R.h"

AD5592R::AD5592R(uint8_t pinSS)     // Constructor
{
    _syncPin = pinSS;
} 

void AD5592R::init()
{
    SPI.begin();

    // Teensy can go faster. The DAC is comfy at 10 MHz
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1)); 
    pinMode(_syncPin, OUTPUT);      // Worked on tempe Readin example!
}

void AD5592R::reset()
{
    uint8_t receive[2];
    comm(AD5592R_CMD_SW_RESET, receive);
}


/*******************************************************************************
 * Auxiliary functions                                                         *
 ******************************************************************************/

uint16_t AD5592R::analog_2_digital(double voltage_mV, bool double_vref)
{
    uint16_t sample;
    double precalc;
    uint16_t ref_voltage_mV;


    if (setup_source_ref_get() == AD5592R_VREF_SOURCE_INTERNAL)
    {
        ref_voltage_mV = AD5592R_INT_REF_mV;
    }
    else
    {
        ref_voltage_mV = _external_ref_voltage_mV;
    }

    if (double_vref == true)
    {
        ref_voltage_mV = 2*ref_voltage_mV;
    }

    precalc = (AD5592R_SAMPLE_CODE_MAX / 1.00) / (ref_voltage_mV / 1.00);

    sample = voltage_mV * precalc;

    return sample;
}

double AD5592R::digital_2_analog(uint16_t sample, bool double_vref)
{
    double voltage_mV;
    double precalc;
    double ref_voltage_mV;

    if (setup_source_ref_get() == AD5592R_VREF_SOURCE_INTERNAL)
    {
        ref_voltage_mV = AD5592R_INT_REF_mV;
    }
    else
    {
        ref_voltage_mV = _external_ref_voltage_mV;
    }

    if (double_vref == true)
    {
        ref_voltage_mV = 2*ref_voltage_mV;
    }

    precalc = (AD5592R_SAMPLE_CODE_MAX / 1.00) / (ref_voltage_mV / 1.00);

    voltage_mV = sample / precalc;

    return voltage_mV;
}

uint16_t AD5592R::percentage_2_digital(unsigned int percentage)
{
    uint16_t sample = percentage * (AD5592R_SAMPLE_CODE_MAX / 100);
    return sample;
}

double AD5592R::percentage_2_analog(unsigned int percentage)
{
    double voltage_mV;

    voltage_mV = percentage * (_supply_voltage_mV / 100);

    return voltage_mV;
}

unsigned int AD5592R::analog_2_percentage(double voltage_mV)
{
    unsigned int percentage;

    percentage = voltage_mV / (_supply_voltage_mV / 100);

    return percentage;
}

void AD5592R::split_word(uint8_t eight_bits[], AD5592R_word sixteen_bits, size_t arr_size)
{
    memset(eight_bits, 0x00, arr_size);
    eight_bits[0] = (sixteen_bits & 0xFF00) >> 8;
    eight_bits[1] = sixteen_bits & 0xFF;
}

uint8_t AD5592R::macro_2_pin(uint8_t macro)
{
    uint8_t pin;

    switch(macro)
    {
        case   1: pin = 0; break;
        case   2: pin = 1; break;
        case   4: pin = 2; break;
        case   8: pin = 3; break;
        case  16: pin = 4; break;
        case  32: pin = 5; break;
        case  64: pin = 6; break;
        case 128: pin = 7; break;
        default:  pin = 0; break;
    }
    return pin;
}

/*******************************************************************************
 * Communication                                                               *
 ******************************************************************************/

bool AD5592R::comm(AD5592R_word sixteen_bits, uint8_t *rx_data)
{
    uint8_t send[2];
    split_word(send, sixteen_bits, sizeof(send));

    
    #ifdef CORE_TEENSY
        digitalWrite(_syncPin, LOW);
    #else
        digitalWrite(SS,LOW);
    #endif

    for (size_t i = 0; i < sizeof(send); i++)
    {
        rx_data[i] = SPI.transfer(send[i]);
    }
#if DAC_DEBUG == true
    Serial.println("SEND:");
    Serial.println(sixteen_bits, HEX);
    Serial.println("REC:");
    Serial.println(rx_data[0], HEX);
    Serial.println(rx_data[1], HEX);
#endif

#ifdef CORE_TEENSY
    digitalWrite(_syncPin, HIGH);  // SS
#else
    digitalWrite(SS,HIGH);
#endif
    return true;
}

/*******************************************************************************
 * Primary configuration                                                       *
 ******************************************************************************/

uint8_t AD5592R::register_readback(AD5592R_reg_readback_t reg){

    uint8_t receive[2];
    comm(AD5592R_CMD_CNTRL_REG_READBACK | 0x41 | (reg << 2), receive);
    comm(AD5592R_CMD_NOP, receive);

    setup_ldac(_ldac_mode);   /* Restore old LDAC-mode */

    return receive[1];
}

bool AD5592R::setup_supply_voltage_mV(double voltage_mV)
{
    if ( (voltage_mV >= AD5592R_MIN_VDD_mV) && (voltage_mV <= AD5592R_MAX_VDD_mV) )
    {
        _supply_voltage_mV = voltage_mV;
        return true;
    }
    else
    {
        return false;
    }
}

void AD5592R::setup_source_ref_set(AD5592R_vref_source_t source)
{
    uint8_t receive[2];

    if (source == AD5592R_VREF_SOURCE_INTERNAL) {

        _vref_source = AD5592R_VREF_SOURCE_INTERNAL;

        comm(AD5592R_CMD_POWER_DWN_REF_CNTRL | (0x2 << 8), receive);
    }
    if (source == AD5592R_VREF_SOURCE_EXTERNAL) {

        _vref_source = AD5592R_VREF_SOURCE_EXTERNAL;

        comm(AD5592R_CMD_POWER_DWN_REF_CNTRL, receive);
    }
}

bool AD5592R::setup_external_ref_voltage_mV(double voltage_mV)
{
    double new_vdd_voltage_mV;

    new_vdd_voltage_mV = _supply_voltage_mV;

    if ( (voltage_mV >= AD5592R_MIN_EXT_REF_mV) && (voltage_mV <= new_vdd_voltage_mV) )
    {
        _external_ref_voltage_mV = voltage_mV;
        return true;
    }
    else
    {
        return false;
    }
}

AD5592R_vref_source_t AD5592R::setup_source_ref_get()
{
    AD5592R_vref_source_t result;

    result = _vref_source;

    return result;
}

void AD5592R::setup_double_vref_set(bool double_vref)
{
    uint8_t receive[2];

    if (double_vref == true)
    {
        _double_vref_adc = true;
        _double_vref_dac = true;

        comm(AD5592R_CMD_GP_CNTRL | AD5592R_ADC_TT_VREF | AD5592R_DAC_TT_VREF, receive);
    }
    if (double_vref == false)
    {
        _double_vref_adc = false;
        _double_vref_dac = false;

        comm(AD5592R_CMD_GP_CNTRL, receive);
    }
}

bool AD5592R::setup_double_vref_adc_get()
{
    bool result;

    result = _double_vref_adc;

    return result;
}

bool AD5592R::setup_double_vref_dac_get()
{
    bool result;

    result = _double_vref_dac;

    return result;
}


/*******************************************************************************
 * PIN configuration                                                           *
 ******************************************************************************/

void AD5592R::setup_ldac(AD5592R_ldac_mode_t ldac_mode)
{
    uint8_t receive[2];

    _ldac_mode = ldac_mode;
    comm(AD5592R_CMD_CNTRL_REG_READBACK | ldac_mode, receive);
}

void AD5592R::setup_dac_all_pins(uint8_t pins)
{
    uint8_t receive[2];

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++) {
        if ( (pins & AD5592R_IO(i)) == AD5592R_IO(i))
        {
            _io_setting[i].type = AD5592R_IO_TYPE_DAC;
        }
    }
    comm(AD5592R_CMD_DAC_PIN_SELECT | pins, receive);
}

void AD5592R::setup_dac(uint8_t pin)
{
    uint8_t receive[2];
    uint8_t pins = 0x0;

    pin = macro_2_pin(pin);

    _io_setting[pin].type = AD5592R_IO_TYPE_DAC;

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++)
    {
        if (_io_setting[i].type == AD5592R_IO_TYPE_DAC)
        {
            pins |= (1 << i);
        }
    }
    comm(AD5592R_CMD_DAC_PIN_SELECT | pins, receive);
}

void AD5592R::setup_adc_all(uint8_t pins)
{
    uint8_t receive[2];

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++) {
        if ( (pins & AD5592R_IO(i)) == AD5592R_IO(i))
        {
            _io_setting[i].type = AD5592R_IO_TYPE_ADC;
        }
    }
    comm(AD5592R_CMD_ADC_PIN_SELECT | pins, receive);
}

void AD5592R::setup_adc(uint8_t pin)
{
    uint8_t receive[2];
    uint8_t pins = 0x0;

    pin = macro_2_pin(pin);

    _io_setting[pin].type = AD5592R_IO_TYPE_ADC;

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++)
    {
        if (_io_setting[i].type == AD5592R_IO_TYPE_ADC)
        {
            pins |= (1 << i);
        }
    }
    comm(AD5592R_CMD_ADC_PIN_SELECT | pins, receive);
}

void AD5592R::gpio_setup_type_all(uint8_t pins, AD5592R_gpio_type_t type)
{
    uint8_t receive[2];
    uint8_t types = 0x0;

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++) {
        if ( (pins & AD5592R_IO(i)) == AD5592R_IO(i))
        {
            _io_setting[i].gpio.type = type;
            if (_io_setting[i].gpio.type == AD5592R_GPIO_TYPE_OPEN_DRAIN)
            {
                types |= (1 << i);
            }
        }
    }
    comm(AD5592R_CMD_GPIO_DRAIN_CONFIG | types, receive);
}

void AD5592R::gpio_setup_type(uint8_t pin, AD5592R_gpio_type_t type)
{
    uint8_t receive[2];
    uint8_t types = 0x0;

    pin = macro_2_pin(pin);

    _io_setting[pin].gpio.type = type;

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++)
    {
        if (_io_setting[i].gpio.type == AD5592R_GPIO_TYPE_OPEN_DRAIN)
        {
            types |= (1 << i);
        }
    }
    comm(AD5592R_CMD_GPIO_DRAIN_CONFIG | types, receive);
}

void AD5592R::gpio_pulldown_all_set(uint8_t pins, bool state)
{
    uint8_t receive[2];

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++) {
        if ( (pins & AD5592R_IO(i)) == AD5592R_IO(i))
        {
            _io_setting[i].gpio.pull_down = state;
            if (_io_setting[i].gpio.pull_down == true)
            {
                pins |= (1 << i);
            }
        }
    }
    comm(AD5592R_CMD_PULL_DOWN_SET | pins, receive);
}

void AD5592R::gpio_pulldown_set(uint8_t pin, bool state){

    uint8_t receive[2];
    uint8_t pins = 0x0;

    pin = macro_2_pin(pin);

    _io_setting[pin].gpio.pull_down = state;

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++)
    {
        if (_io_setting[i].gpio.pull_down == true)
        {
            pins |= (1 << i);
        }
    }

    comm(AD5592R_CMD_PULL_DOWN_SET | pins, receive);
}

void AD5592R::gpio_direction_all_set(uint8_t pins, AD5592R_gpio_direction_t direction)
{
    uint8_t receive[2];
    uint8_t directions = 0x0;

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++) {
        if ( (pins & AD5592R_IO(i)) == AD5592R_IO(i))
        {
            _io_setting[i].type = AD5592R_IO_TYPE_GPIO;
            _io_setting[i].gpio.direction = direction;
            directions |= (1 << i);
        }
    }

    if (direction == AD5592R_GPIO_DIRECTION_IN)
    {
        comm(AD5592R_CMD_GPIO_READ_CONFIG | directions, receive);
    }
    if (direction == AD5592R_GPIO_DIRECTION_OUT)
    {
        comm(AD5592R_CMD_GPIO_WRITE_CONFIG | directions, receive);
    }
}

void AD5592R::gpio_direction_set(uint8_t pin, AD5592R_gpio_direction_t direction)
{
    uint8_t receive[2];
    uint8_t directions = 0x0;

    pin = macro_2_pin(pin);

    _io_setting[pin].gpio.direction = direction;
    _io_setting[pin].type = AD5592R_IO_TYPE_GPIO;

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++)
    {
        if (_io_setting[i].gpio.direction == direction)
        {
            directions |= (1 << i);
        }
    }

    if (direction == AD5592R_GPIO_DIRECTION_IN)
    {
        comm(AD5592R_CMD_GPIO_READ_CONFIG | directions, receive);
    }
    if (direction == AD5592R_GPIO_DIRECTION_OUT)
    {
        comm(AD5592R_CMD_GPIO_WRITE_CONFIG | directions, receive);
    }
}

void AD5592R::gpio_state_all_set( uint8_t pins, AD5592R_gpio_state_t state)
{
    uint8_t receive[2];
    uint8_t pp_states = 0x0;
    uint8_t z_states = 0x0;

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++) {
        if ( (pins & AD5592R_IO(i)) == AD5592R_IO(i))
        {
            _io_setting[i].gpio.state = state;
            if (_io_setting[i].gpio.state == AD5592R_GPIO_HIGH)
            {
                pp_states |= (1 << i);
            }
            if (_io_setting[i].gpio.state == AD5592R_GPIO_Z)
            {
                z_states |= (1 << i);
            }
        }
    }

    comm(AD5592R_CMD_GPIO_WRITE_DATA | pp_states, receive);
    comm(AD5592R_CMD_THREE_STATE_CONFIG | z_states, receive);

}

void AD5592R::gpio_state_set(uint8_t pin, AD5592R_gpio_state_t state)
{
    uint8_t receive[2];
    uint8_t pp_states = 0x0;
    uint8_t z_states = 0x0;

    pin = macro_2_pin(pin);

    _io_setting[pin].gpio.state = state;

    for (int i = AD5592R_CHANNEL_MIN; i <= AD5592R_CHANNEL_MAX; i++)
    {
        if (_io_setting[i].gpio.state == AD5592R_GPIO_HIGH)
        {
            pp_states |= (1 << i);
        }
        if (_io_setting[i].gpio.state == AD5592R_GPIO_Z)
        {
            z_states |= (1 << i);
        }
    }

    comm(AD5592R_CMD_GPIO_WRITE_DATA | pp_states, receive);
    comm(AD5592R_CMD_THREE_STATE_CONFIG | z_states, receive);
}

uint8_t AD5592R::gpio_input_state_get(uint8_t pins)
{
    uint8_t receive[2];

    comm(AD5592R_CMD_GPIO_READ_INPUT | pins, receive);
    comm(AD5592R_CMD_NOP, receive);

    return receive[1];
}

uint8_t AD5592R::gpio_output_state_get(uint8_t pin)
{
    uint8_t result;

    pin = macro_2_pin(pin);

    result = _io_setting[pin].gpio.state;

    return result;
}

uint16_t AD5592R::adc_sample_get(uint8_t pin)
{
    uint8_t receive[2];

    pin = macro_2_pin(pin);

    comm(AD5592R_CMD_ADC_READ | (0x1 << pin), receive);
    comm(AD5592R_CMD_NOP, receive);
    comm(AD5592R_CMD_NOP, receive);

    /* Discard the first nibble */
    uint16_t result = ((receive[0] << 8) & 0x0F00) | (receive[1] & 0xFF);

    return result;
}

double AD5592R::adc_voltage_get_mV(uint8_t pin)
{
    uint8_t receive[2];

    pin = macro_2_pin(pin);

    comm(AD5592R_CMD_ADC_READ | (0x1 << pin), receive);
    comm(AD5592R_CMD_NOP, receive);
    comm(AD5592R_CMD_NOP, receive);

    /* Discard the first nibble */
    uint16_t result = ((receive[0] << 8) & 0x0F00) | (receive[1] & 0xFF);

    return digital_2_analog(result, setup_double_vref_adc_get());
}

unsigned int AD5592R::adc_percentage_get(uint8_t pin)
{
    return analog_2_percentage(adc_voltage_get_mV(pin) );
}

bool AD5592R::dac_sample_set(uint8_t pin, uint16_t sample)
{
    uint8_t receive[2];

    pin = macro_2_pin(pin);

    if (sample <= AD5592R_SAMPLE_CODE_MAX)
    {
        comm(AD5592R_DAC_WRITE_MASK |                           /* DAC write command */
            ( (pin << 12) & AD5592R_DAC_ADDRESS_MASK ) |        /* DAC-PIN (address) & DAC addressmask */
            sample,                                             /* Digital value */
        receive);

        _io_setting[pin].dac.sample = sample;

        return true;
    }
    else
    {
        return false;
    }
}

bool AD5592R::dac_voltage_set_mV(uint8_t pin, double voltage_mV)
{
    uint8_t receive[2];
    uint16_t result;
    double supply_voltage_mV;

    pin = macro_2_pin(pin);

    supply_voltage_mV = _supply_voltage_mV;

    if (voltage_mV <= supply_voltage_mV)
    {
        result = analog_2_digital(voltage_mV, setup_double_vref_dac_get());
        comm(AD5592R_DAC_WRITE_MASK |                           /* DAC write command */
            ( (pin << 12) & AD5592R_DAC_ADDRESS_MASK ) |        /* DAC-PIN (address) & DAC addressmask */
            result,                                             /* Digital value */
            receive);

        _io_setting[pin].dac.sample = result;

        #if DAC_DEBUG == true
            Serial.println("Sent!");
        #endif  

        return true;
    }
    else
    {
        return false;
    }
}

bool AD5592R::dac_percentage_set(uint8_t pin, unsigned int percentage)
{
    if (percentage <= 100)
    {
        double voltage_mV = percentage_2_analog(percentage);
        dac_voltage_set_mV(pin, voltage_mV);

        return true;
    }
    else
    {
        return false;
    }
}

uint16_t AD5592R::dac_sample_get(uint8_t pin)
{
    uint16_t result;

    pin = macro_2_pin(pin);

    result = _io_setting[pin].dac.sample;

    return result;
}


/*******************************************************************************
 * Miscellaneous functions                                                     *
 ******************************************************************************/

double AD5592R::temperature_get_degC()
{
    uint8_t receive[2];
    double result;
    double vref_mV = 0;
    AD5592R_vref_source_t new_vref_source;

    comm(AD5592R_CMD_ADC_READ | (0x1 << 8), receive);
    delayMicroseconds(25);
    comm(AD5592R_CMD_NOP, receive);
    comm(AD5592R_CMD_NOP, receive);

    /* Discard the first nibble */
    uint16_t temp_raw = ((receive[0] << 8) & 0x0F00) | (receive[1] & 0xFF);

    new_vref_source = _vref_source;

    if (new_vref_source == AD5592R_VREF_SOURCE_EXTERNAL)
    {
        vref_mV = _external_ref_voltage_mV;
    }
    if (new_vref_source == AD5592R_VREF_SOURCE_INTERNAL)
    {
        vref_mV = AD5592R_INT_REF_mV;
    }

    if (setup_double_vref_adc_get() == true)
    {
        result = AD5592R_TEMPERATURE_FORMULA_GAIN2(temp_raw, vref_mV);
    }
    else
    {
        result = AD5592R_TEMPERATURE_FORMULA_GAIN1(temp_raw, vref_mV);
    }

    return result;
}

uint16_t AD5592R::temperature_sample_get()
{
    uint8_t receive[2];

    comm(AD5592R_CMD_ADC_READ | (0x1 << 8), receive);
    comm(AD5592R_CMD_NOP, receive);
    comm(AD5592R_CMD_NOP, receive);

    /* Discard the first nibble */
    uint16_t result = ((receive[0] << 8) & 0x0F00) | (receive[1] & 0xFF);

    return result;
}

double AD5592R::sample_to_temperature_degC(uint16_t temperature_sample)
{
    double result;
    double vref_mV = 0;
    AD5592R_vref_source_t new_vref_source;

    new_vref_source = _vref_source;

    if (new_vref_source == AD5592R_VREF_SOURCE_EXTERNAL)
    {
        vref_mV = _external_ref_voltage_mV;
    }
    if (new_vref_source == AD5592R_VREF_SOURCE_INTERNAL)
    {
        vref_mV = AD5592R_INT_REF_mV;
    }

    if (setup_double_vref_adc_get() == true)
    {
        result = AD5592R_TEMPERATURE_FORMULA_GAIN2(temperature_sample, vref_mV);
    }
    else
    {
        result = AD5592R_TEMPERATURE_FORMULA_GAIN1(temperature_sample, vref_mV);
    }

    return result;
}
