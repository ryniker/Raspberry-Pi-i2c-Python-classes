#   File:  i2c_device.py

'''Classes to support i2c devices.

Uses the smbus2 package:  https://pypi.python.org/pypi/smbus2/0.2.0

Developed and tested on Raspberry Pi machines using Fedora and
Raspbian systems.

i2c buses are accessed via character special files such as:
    /dev/i2c-0         (i2c bus 0)
    /dev/i2c-1         (i2c bus 1)

It may be necessary to load a kernel module:
    modprobe  i2c_dev
in order to create these special files and furnish the kernel with code
to translate file operations using them into i2c bus operations.

User applications have to have the necessary permission to access the
i2c bus files.  For a single-user system, it may suffice to use a
command like:
    chmod a+rw /dev/i2c-1
to give all users access to i2c bus 1.

If the special file can be used by a specific group (such as group
i2c), then user xyz may be added to that group by:
    usermod --groups i2c xyz
User xyz executes:
    newgrp i2c
to set his current group id to i2c, and thereby gain access to an i2c bus.


An instance of this class records the i2c bus and device address for a
specific device, which simplifies subsequent calls to manipulate that
device.

The base class i2c_device provides some common data and methods to
interface with the i2c bus operations available in the smbus2 package.

Several subclasses are defined for specific i2c devices.  These
subclasses provide data and methods to exploit those particular
devices These subclasses provide examples for how additional
subclasses might be crafted to support other devices.

The kernel provides low-level serialization to insure one i2c bus
transaction is complete before another transaction is started.

Applications that use i2c devices must address any issues that arise
from shared use of the i2c bus and i2c devices.

There is no mechanism to prevent two processes that have permission to
use an i2c bus from separate creation of i2c_device instances and, by
their use, interfere with the behavior each process expects.  For
example, a device that might be configured either for input or for
output might be configured for read operations by application A, then
changed by application B for write activity.  A's logic then fails
when it tries to read from an output device.

Problems may occur if one application uses multiple threads
that try to share one i2c_device instance.  The application must
insure i2c_device methods are not re-entered.
________________________________________________________________________________

MIT License.

Copyright 2018 Richard Ryniker      <ryniker@alum.mit.edu>

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
________________________________________________________________________________

'''

from smbus2 import SMBus, i2c_msg
import sys, time

DEFAULT_I2C_BUS = 1

byte_swap = False
if sys.byteorder != 'big':
    byte_swap = True
    

def get_bits(v, len, lsb):
    '''Return the <len> bits from integer <v> with least significant
    bit at offset <lsb> measured from the right.
    For example, get_bits(0b10011100, 3, 2) returns 7.'''
    return (v >> lsb) & ( (-1 << len) ^ -1)


class i2c_error(Exception):
    def __init__(self, message):
        self.message = message


class i2c_device:
    # Bit masks.
    bit0    = 0x01
    bit1    = 0x02
    bit2    = 0x04
    bit3    = 0x08
    bit4    = 0x10
    bit5    = 0x20
    bit6    = 0x40
    bit7    = 0x80

    def __init__(self, address = None, bus = None):
        self.address = address  # "address" is the device address on the i2c bus.
        self.bus = bus
        self.DEBUG = 0
        if self.bus == None:
            self.bus = DEFAULT_I2C_BUS
            # "bus" is the Raspberry Pi external i2c bus, where the MCP23017 is connected.
            # This may vary accross different Raspberry Pi models.  0 is used by the
            # original Raspbery Pi model B (256 MB memory, 26-pin GPIO connector).
            # 1 is used by recent Raspberry Pi devices.
        self.device = SMBus(self.bus)
        return


    def bus_reset(self):
        '''Caution:  bus reset is a general call operation.  It
is not device-specific.  All devices on the bus are affected.
In systems with multiple i2c busses, only one bus (the bus associated
with this device) is reset.'''
        self.device.write_byte(0, 6)
        return
    
    
    def wbyte_only(self, value):
        # Write a single byte to device, without any register address.
        self.device.write_byte(self.address, value)

    def rbyte_only(self):
        # Read a single byte from device, without any register address.
        return self.device.read_byte(self.address)
    
    def wbyte(self, reg, value):
        # Write a single byte <value> to register <reg>.
        if self.DEBUG >= 32:
            sys.stdout.write('  write  x{:02X} to chip x{:02X} register {}\n'.format(value, self.address, reg))
            sys.flush()
        self.device.write_byte_data(self.address, reg, value)
        return

    def rbyte(self, reg):
        # Read one byte from register <reg>.
        return self.device.read_byte_data(self.address, reg)

    def wword(self, reg, value):
        '''Write the integer <value> (two bytes) to register <reg>.'''
        if byte_swap:
            value = ((value & 0xff) << 8) | ((value >> 8) & 0xff)
        self.device.write_word_data(self.address, reg, value)
        return

    def rword(self, reg):
        value = self.device.read_word_data(self.address, reg)
        if byte_swap:
            value = ((value & 0xff) << 8) | ((value >> 8) & 0xff)
        return value

    
######################################################################
    
class MCP23017(i2c_device):
    ''' Microchip 16-bit i2c I/O expander.
'''
    # Define MCP23017 register names (when IOCON BANK bit = 1).
    IODIR      = 0    # I/O direction:  bit = 1 for input; bit = 0 for output.
    IPOL       = 1    # Input polarity  bit = 1 for active low; bit = 0 for active high.
                      # If polarity bit is 1, zero input is reported as 1; high input is reported as 1.

    GPINTEN      = 2    # Enable interrupt-on-change:  bit = 1 to enable interrupt for a pin; bit = 0 for no interrupt.
    DEFVAL     = 3    # Default (comparison) value for interrupt-on-change: interrupt may occur when
                      # pin input value is different from the corresponding default value.
    INTCON     = 4    # Interrupt-on-change control:  bit = 1:  pin input value is compared with the
                      # corresponding bit in DEFVAL;  bit = 0:  pin input value is compared with the
                      # previous pin input value.  In either case, if the values compare unequal, an
                      # interrupt will be generated.

    IOCON      = 5    # Device configuration register.
    PULLUP     = 6    # Pull-up resistor control:  bit = 1 to activate pull-up resistor when pin is
                      # configured as an input.
    IFLAG      = 7    # Interrupt flag register (read-only).  Bit is set to 1 when interrupt condition is
                      # detected on the corresponding pin.
    INTCAP     = 8    # Interrupt capture register.
    PORT       = 9    # GPIO port register.  Read returns the current port value.  Write sets the output latch register.
    LATCH      = 10   # Output latch register.  Read returns value of this register, not port value.

    PORTB      = 0x10 # Add (or logical-or) this value to the above register name values to address
                      # port B registers instead of port A registers.  There is only one IOCON register
                      # in the device; two addresses select the same register.
    PORTA      = 0    # Only for code clarity.  This may be added (or logical-ored) to register name
                      # values if some explicit symbol to identify port A is desired.  The zero value
                      # does not change the "default" register value that addresses port A.

    # Configuration register bits:

    BANK       = i2c_device.bit7 # In IOCON: defines register address mode.  Set to 1 before using the address
                      # definitions above.  Power-on value is 0 (this denotes interleaved register
                      #  address scheme).
    SEQOP      = i2c_device.bit5 # Set to 1 to disable sequential operation mode.
    ODR        = i2c_device.bit2 # Configure interrupt outputs as open drain circuits to allow wire-or connection.
    INTPOL     = i2c_device.bit1 # Interupt polarity: 1 = active high; 0 = active low.


    def __init__(self, address = 0x20, bus = None):
        # Default address for MCP23017 (all 3 address pins are zero).
        # Other valid addresses are 0x21 through 0x27.
        i2c_device.__init__(self, address = address, bus = bus)

        # Set address mode.  If bank is 0 (the power-on value), this will set it to 1.
        # If bank is already 1, this will not change it because register 11 (0x0b) does not
        # address any register when bank is 1.  The user may change this later,
        # but the symbolic register names defined above are valid only when the bank mode
        # bit is 1.
        self.wbyte(11, self.BANK)   

        
######################################################################
        
class ADS1115(i2c_device):
    '''Texas Instrumenets 16-bit analog-to-digital converter.
The ADS1113 and ADS1114 parts are very similar, and
support subsets of the function of the ADS1115:
  ADS1113 has two inputs, no comparator.
  ADS1114 has two inputs.

There is no explicit indication when an input voltage exceeds
the range permitted by the programmable amplifier.  For example,
if no range is selected using the <pga> argument to configure(),
the default range of 2.048 volts (pga=2) is used.  If an input voltage
of 3 volts is measured, its value will be indicated as 
2.048 volts (raw value = 0x7fff).  In this example, 
configure(pga=PGA_4V) could be used to set a pga value of 1
(4.096 volts full scale) and a correct reading of 3.0 will be obtained
for the input voltage.
'''

    # Symbolic names for the four device registers:
    R_value      = 0     # Last conversion value.
    R_config     = 1     # Configuration.
    R_low        = 2     # Comparator low threshold
    R_high       = 3     # Comparator high threshold.

    # Names for some configuration values:
    MUX_A0       = 0b100
    MUX_A1       = 0b101
    MUX_A2       = 0b110
    MUX_A3       = 0b111
    MUX_A0_A1    = 0b000
    MUX_A0_A3    = 0b001
    MUX_A1_A3    = 0b010
    MUX_A2_A3    = 0b011
    PGA_6V       = 0b000    #  6.144 volts full scale.
    PGA_4V       = 0b001    #  4.096 volts full scale.
    PGA_2V       = 0b010    #  2.948 volts full scale.
    PGA_1V       = 0b011    #  1.024 volts full scale.
    PGA_F5V      = 0b100    #  0.512 volts full scale.
    PGA_F25V     = 0b101    #  0.256 volts full scale.
    # PGA values 0b110 and 0b111 also denote 0.256 volts full scale.
    RATE_8       = 0b000    #    8 samples per second.
    RATE_16      = 0b000    #   16 samples per second.
    RATE_32      = 0b000    #   32 samples per second.
    RATE_64      = 0b000    #   64 samples per second.
    RATE_128     = 0b000    #  128 samples per second.
    RATE_250     = 0b000    #  250 samples per second.
    RATE_475     = 0b000    #  475 samples per second.
    RATE_860     = 0b000    #  860 samples per second.
    
    # lsb voltage values for pga values (0, 1, 2, 3, 4, 5, 6, 7)
    pga_lsb = (187.5E-6, 125E-6, 62.5E-6, 31.25E-6, 15.625E-6, 7.8125E-6, 7.8125E-6, 7.8125E-6)

    # Sample rate associated with each configuration data rate value
    # (conversions per second):
    rate_frequency = (8, 16, 32, 64, 128, 250, 475, 860)
    
    def __init__(self, address = 0x48, bus = None):
        ''' Default address is for ADS1115 with address pin grounded.
Other valid addresses are 0x41 through 0x43.
'''        
        
        i2c_device.__init__(self, address = address, bus = bus)
        self.delay_trigger = False
        self.delay_trigger_time = 0.0

        # These variables must exist before configure is called:
        self.os = None
        self.mux = None
        self.pga = None
        self.mode = None
        self.rate = None

        self.configure(defaults=True)
        

    def write(self, reg, value):
        return self.wword(reg, value)


    def read(self, reg = None):
        '''Specify register to read raw value.
Default reg to read the conversion register (R_value)
and convert it to a floating point voltage value.

If there has been a recent configuration change that might
require a delay before the conversion register has valid data,
implement the appropriate delay.
'''
        if reg == None:
            if self.delay_trigger:
                self.delay()
            return self.raw_to_volts(self.rword(self.R_value))
        else:
            if reg == self.R_value:
                self.delay()
            return self.rword(reg)

        
    def raw_to_volts(self, raw):
        '''Convert a <raw> value (read earlier from register R_value) to volts.
This is the conversion performed by read()
with argument None (or no argument).'''
        if raw >= 0x8000:
            raw = - ((raw ^ 0xffff) + 1)
        return raw * self.pga_lsb[self.pga]


    def configure(self, defaults=False, os=None, mux=None, pga=None, mode=None, rate=None, comp_mode = None, comp_polarity = None, comp_latch = None, comp_que = None):
        '''Set device configuration.  The default values are what
power-on or reset establish.

If the <defaults> argument is False, only the specified configuration
parameters are changed; parameters not specified will continue to have
the values saved from the last configuration call that provided values
for them.

If the <defaults> argument is True, all configuration parameters that
are specified in this call will be set to the specified values;
all other configuration parameters will be set to their default values.

  os       = operational status:
               1  means start a single conversion.
               0  has no effect.
  mux      = input multiplexor configuration (3 bits)
               000  = differential  A0 - A1
               001  = differential  A0 - A3
               010  = differential  A1 - A3
               011  = differential  A2 - A3
               100  = A0 - ground
               101  = A1 - ground
               110  = A3 - ground
               111  = A4 - ground
  pga      = programmable gain amplifier (3 bits)
  mode     = device operating mode:
               0  = continuous conversion mode.
               1  = single conversion mode.
  rate     = data rate (number of conversions per second) (3 bits)
               000  =   8 samples per second.
               001  =  16 samples per second.
               010  =  32 samples per second.
               011  =  64 samples per second.
               100  = 128 samples per second.  Default value.
               101  = 250 samples per second.
               110  = 475 samples per second.
               111  = 860 samples per second.
                       
   comp_que = comparator que and disable (2 bits)
                11  = disable comparator.

Of course, a user can simply write the desired configuration into
R_config.  The purpose of this function is to put desired values
into the appropriate bit fields, check parameeters for valid values,
and be more understandable than an equivalent integer value.'''

        if defaults:       # Start with the default values.
            w_os = 1
            w_mux = 0
            w_pga = 2
            w_mode = 1
            w_rate = 4
            w_comp_mode = 0
            w_comp_polarity = 0
            w_comp_latch = 0
            w_comp_que = 3
        else:              # Start with saved values from earlier configuration.
            w_os = self.os
            w_mux = self.mux
            w_pga = self.pga
            w_mode = self.mode
            w_rate = self.rate
            w_comp_mode = self.comp_mode
            w_comp_polarity = self.comp_polarity
            w_comp_latch = self.comp_latch
            w_comp_que = self.comp_que

        if os != None:
            w_os = int(os)
        if mux != None:
            w_mux = int(mux)
        if pga != None:
            w_pga = int(pga)
        if mode != None:
            w_mode = int(mode)
        if rate != None:
            w_rate = int(rate)
        if comp_mode != None:
            w_comp_mode = int(comp_mode)
        if comp_polarity != None:
            w_comp_polarity = int(comp_polarity)
        if comp_latch != None:
            w_comp_latch = int(comp_latch)
        if comp_que != None:
            w_comp_que = int(comp_que)

        if w_os != 0 and w_os != 1:
            raise i2c_error('Invalid os argument value:  {}'.format(w_os))
        if w_mux < 0 or w_mux > 7:
            raise i2c_error('Invalid mux argument value:  {}'.format(w_mux))
        if w_pga < 0 or w_pga > 7:
            raise i2c_error('Invalid pga argument value:  {}'.format(w_pga))
        if w_mode != 0 and w_mode != 1:
            raise i2c_error('Invalid mode argument value:  {}'.format(w_mode))
        if w_rate < 0 or w_rate > 7:
            raise i2c_error('Invalid rate argument value:  {}'.format(w_rate))
        if w_comp_mode != 0 and w_comp_mode != 1:
            raise i2c_error('Invalid comp_mode argument value:  {}'.format(w_comp_mode))
        if w_comp_polarity != 0 and w_comp_polarity != 1:
            raise i2c_error('Invalid comp_polarity argument value:  {}'.format(w_comp_polarity))
        if w_comp_latch != 0 and w_comp_latch != 1:
            raise i2c_error('Invalid comp_latch argument value:  {}'.format(w_comp_latch))
        if w_comp_que < 0 or w_comp_que > 3:
            raise i2c_error('Invalid comp_que argument value:  {}'.format(w_comp_que))

        # If configuration change means time must elapse until valid configuration
        # data is available, set a trigger to delay for an appropriate time.
        if (w_mux != self.mux) or (w_pga != self.pga) or (w_rate != self.rate) or (w_os == 1) or (w_mode == 0):
            self.delay_trigger = True
            self.delay_trigger_time = time.time()

        
        ''' Remember the configuration values.
        
        If a user sets the device configuration directly, by a write
        to R_config, the values saved here may no longer be correct.
        Use the read_config method to update the class attributes with
        the current device values.
        '''
        self.os = w_os
        self.mux = w_mux
        self.pga = w_pga
        self.mode = w_mode
        self.rate = w_rate
        self.comp_mode = w_comp_mode
        self.comp_polarity = w_comp_polarity
        self.comp_latch = w_comp_latch
        self.comp_que = w_comp_que

        # Build the desired config register value.
        value = (self.os << 15) | (self.mux << 12) | (self.pga << 9) | (self.mode << 8) | (self.rate << 5) | (self.comp_mode << 4) | (self.comp_polarity << 3) | (self.comp_latch << 2) | self.comp_que

        '''
        # The configuration value just computed is the big-endian format
        # used by the device.  Swap bytes if the local host is little-endian,
        # because then, when the word is written to the device, it will be
        # byte-swapped again, back to the device format.

        if byte_swap:
            value = ((value & 0xff) << 8) | ((value >> 8) & 0xff)
        '''

        if self.DEBUG != 0:
            sys.stderr.write('ADS1115 config value = 0x{:04x}  0b{:016b}\n'.format(value, value))
            sys.stderr.flush()

        self.write(self.R_config, value)
        return

    
    def read_config(self):
        '''Read actual configuration from the device, and update the class
        configuration attributes.'''
        v = self.read(self.R_config)
        self.os              = get_bits(v, 1, 15)
        self.mux             = get_bits(v, 3, 12)
        self.pga             = get_bits(v, 3,  9)
        self.mode            = get_bits(v, 1,  8)
        self.rate            = get_bits(v, 3,  5)
        self.comp_mode       = get_bits(v, 1,  4)
        self.comp_polarity   = get_bits(v, 1,  3)
        self.comp_latch      = get_bits(v, 1,  2)
        self.comp_que        = get_bits(v, 2,  0)
    
    def delay(self):
        '''Delay for enough time to allow one conversion at the current
data rate.  After changing the input to measue (the multiplexor value),
data rate, or starting cconversion activity, it will require a period of
time (that depends on the data rate) for a conversion to complete and
valid date to be available in the conversion register.  This functions
waits long enough for one conversion at the current data rate before it returns.

This is called internally by the read method after a configuration change
(only needed when R_value is read) to insure valid data is returned.
'''
        delay = (1.0 / self.rate_frequency[self.rate]) - (time.time() - self.delay_trigger_time)
        if delay > 0:
            time.sleep(delay)
        self.delay_trigger = False
        return

######################################################################
        
class MCP4725(i2c_device):
    '''Microchip MCP4725 digital-to-analog converter.
    '''

    def __init__(self, address = 0x62, bus = None):
        '''The bus address of this device is 0x60 plus
three address bits.  The low-order bit (A0) depends on the
value of the address input pin, while address bits A1 and A2
are set by the manufacturer.  Therefore, when A0 is zero (ground),
the actual address may be 0x60, 0x62, 0x64, or 0x66.
When A0 is one, the actual accress may be  0x61, 0x63, 0x65, or 0x67.

The device used during code development had A1 programmed by
the manufacturer to be 1, which is why the default <address>
value is set to 0x62.
'''
        i2c_device.__init__(self, address = address, bus = bus)
        self.m_read = i2c_msg.read(self.address, 3)
        self.power = 0

    def read(self):
        '''Read the current configuration and value data from the device.
Returns an integer value with 24 bits of data:  CCDDDD'''
        self.device.i2c_rdwr(self.m_read)
        return (ord(self.m_read.buf[0]) << 16) | (ord(self.m_read.buf[1]) << 8) | ord(self.m_read.buf[2])

    
    def write(self, data, power = None, EEPROM = None):
        '''Write <data> to the device to set the desired
output voltage:  0 <= <data> <= 4095.

<power> is optional, and defaults to normal operation.
    0 = normal operation (output is driven to the value
        specified by <data>.
    1 = output off;    1 Kohm to ground.
    2 - output off;  100 Kohm to ground.
    3 - output off;  500 Kohm to ground.

If EEPROM is True, data is also saved to flash memory.

If EEPROM is not specified, a "fast write" (only two blytes)
is performed to set current power and data values.

If EEPROM is specified as False, a three-byte
write is performed that does not modify EEPROM data.
'''
        if power != None:
            if (not isinstance(power, int)) or power < 0 or power > 3:
                raise i2c_error('Invalid power argument value.')
        else:
            power = 0     # Default value is normal operation.
        data &= 0xfff     # Only 12 data bits can be used.

        if EEPROM == None:
            # Perform a fast write (only two bytes instead of three.)
            cmd = (power << 4) | (data >> 8)
            self.wbyte(cmd, data & 0xff)
        else:    
            cmd = 0x40        # Normal write.
            if EEPROM:
                cmd = 0x60
            cmd |= (power << 1)

            data = data << 4
            self.wword(cmd, data)
        return
    

######################################################################
        
class PCF8574(i2c_device):
    '''Texas Instruments PCF8574 I/O expander.

The PCF8574 device documentation specifies a maximum 100 kHz clock signal,
though I have seen it operate at nearly 400 kHz.
    '''

    def __init__(self, address = 0x38, bus = None):
        '''The TI data sheet for this part says it uses bus address 0x20,
but the devices I have used respond to bus address 0x38 (with
all three address pins 0.)

TI describes device addresses that include a low-order bit which is
the R/-W bit of the i2c protocol.  Thus a 7-bit device address of 0x20
is described as 0x40 (for write) or 0x41 (for read).
This does not explain the 0x38 (7-bit) address of the test devices.

This is almost the simplest device possible.

If any of the eight port bits will be used for input, they
should first be written as 1's (this is the power-on state
of the port.)  This allows the internal pull-up resistors to
supply a 1 value, unless an external device pulls the pin low.

If a port pin has been written as 0, the output latch drives the pin
to 0 and this will be the value returned by read, regardless of
the external connection.
'''
        i2c_device.__init__(self, address = address, bus = bus)

        
    def write(self, value):
        self.wbyte_only(value)

    def read(self):
        return self.rbyte_only()


######################################################################
        
