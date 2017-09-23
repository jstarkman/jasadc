#! /bin/sh
# Source: https://bbs.nextthing.co/t/chips-internal-adc/2136/59
function enable_2v_adc_on_gpio0 {
    # docs: http://dl.linux-sunxi.org/AXP/AXP209_Datasheet_v1.0en.pdf
    #REG 83H: ADC Enable 2
    #bit 3 : GPIO0 ADC function is enabled , default 80H

    #REG 85H: ADC Input Range  default X0H
    #bit 0 : GPIO0 ADC input range: 1 : 0.7v - 2.7475v
    #                               0 : 0v - 2.0475v

    #REG 90H: GPIO0 function settings : default 07H
    #    bit 0,1,2 : 0b100 : ADC input
    /usr/sbin/i2cset -f -y 0 0x34 0x83 0x80 # disable ADC input on GPIO0
    /usr/sbin/i2cset -f -y 0 0x34 0x90 0x04 # set GPIO0 to 12 bit ADC input
    # /usr/sbin/i2cset -f -y 0 0x34 0x85 0x01 # set ADC input range to 0.7-2.7475v
    /usr/sbin/i2cset -f -y 0 0x34 0x85 0x00 # set ADC input range to 0-2v
    /usr/sbin/i2cset -f -y 0 0x34 0x83 0x88 # enable ADC input on GPIO0
}

function reset_gpio0_to_vcc1v8 {
    /usr/sbin/i2cset -f -y 0 0x34 0x83 0x80 # disable ADC input on GPIO0
    /usr/sbin/i2cset -f -y 0 0x34 0x90 0x03
    /usr/sbin/i2cset -f -y 0 0x34 0x91 0x00 #calculation is output voltage = 1.8v + high_4_bit(0x91) * 0.1v
}


function read_gpio0_adc {
    # read GPIO0 as 12-bit ADC:
    # get the ADC input range: 0.7v offset from register 85H
    p=$(i2cget -f -y 0 0x34 0x85 b)
    rhigh=$(i2cget -f -y 0 0x34 0x64 b)
    rlow=$(expr substr "$(i2cget -f -y 0 0x34 0x65 b)" 4 1)
    r="${rhigh}${rlow}"
    echo $(( (((r *10000)/4096)*2)+p*7000 )) # bash can perform HEX arithmatics automatically
}

