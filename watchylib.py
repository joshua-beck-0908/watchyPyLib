import digitalio
import gc
import analogio
import board
import pwmio
import math
import time
import displayio
import vectorio
import terminalio
import wifi
import socketpool
import os
import json
import rtc
import random
import atexit
import alarm
import microcontroller
import supervisor
import adafruit_requests
import adafruit_display_text.label
import adafruit_display_text.scrolling_label
import adafruit_display_shapes.circle
import adafruit_display_shapes.rect
import adafruit_display_shapes.roundrect
import adafruit_display_shapes.triangle
import adafruit_display_shapes.line
import adafruit_display_shapes.polygon
import adafruit_imageload
import adafruit_pcf8563.pcf8563
import adafruit_ntp
import adafruit_bus_device.i2c_device
from adafruit_register.i2c_bit import RWBit, ROBit
from adafruit_register.i2c_bits import RWBits, ROBits
from adafruit_register.i2c_struct import UnaryStruct, ROUnaryStruct, Struct

supervisor.set_next_code_file(filename='crash.py',
    reload_on_error=True, reload_on_success=False)

##
# @brief Main Watchy class.
# Defines all the hardware components and provides some convenience functions.
class Watchy:
    def __init__(self):
        config = Settings('settings.json')
        config['i2c']  = self.i2c  = board.I2C()
        config['pool'] = self.pool = socketpool.SocketPool(wifi.radio)

        self.buzzer = Buzzer(board.VIB)
        self.display = Display(board.DISPLAY)
        self.accel = Accelerometer(config)
        self.button = ButtonPad([board.BTN1, board.BTN2, board.BTN3, board.BTN4])
        self.battery = Battery(board.VBAT)
        self.time = RTC(config)
        self.geo = GeoLocation(config)
        self.ntp = NTP(config)
        atexit.register(self.finish)
        
    def finish(self):
        self.display.fastRefresh = False
        self.display.greyMode = False
        self.display.darkMode = False
        self.display.display.root_group = displayio.CIRCUITPYTHON_TERMINAL
        self.display._rebuildUpdateSequence()
        
        
    @property
    def stepCount(self):
        return self.accel.stepCount
    
    @property
    def id(self):
        return ''.join([f'{i:02x}' for i in microcontroller.cpu.uid])

    def sleep(self):
        #TODO
        time.sleep(60)
        
    def reset(self):
        microcontroller.reset()
        
        
##
# @brief Defines the four buttons on the side of the watch.
# Provides convenience functions for accessing the buttons.
# See the Button class for more information.
#
# Available buttons are: up, down, cancel, confirm, menu, back, any
# You can access them like this:
# watchy.button.up.pressed
# watchy.button.down.wait()
# any is a meta button that can be used to wait for any button to be pressed.
# i.e. watchy.button.any.wait()

class ButtonPad:
    def __init__(self, pins):
        self.buttons = [Button(pin) for pin in pins]
        self.up = self.buttons[2]
        self.down = self.buttons[3]
        self.cancel = self.buttons[1]
        self.confirm = self.buttons[0]
        self.menu = self.confirm
        self.back = self.cancel
        self.any = AnyButton(self.buttons)
    
    ##
    # @brief Returns a list of all buttons that are currently pressed.
    # @return A list of all buttons that are currently pressed. Empty if none.
    @property
    def which(self):
        buttons = []
        for b in self.buttons:
            if b.pressed:
                buttons.append(b)
        return buttons

##
# @brief Defines a single button.
# Provides convenience functions for accessing the button.
class Button:
    def __init__(self, pin):
        self.pin = pin
        self.button = digitalio.DigitalInOut(pin)
        self.button.direction = digitalio.Direction.INPUT
        
    ##
    # @brief Returns True if the button is currently pressed.
    # @return True if the button is currently pressed.
    @property
    def pressed(self):
        return self.button.value
    
    ##
    # @brief Waits until the button is pressed.
    # @return None
    def wait(self):
        while not self.pressed:
            time.sleep(0.01)
            
            
##
# @brief Meta button that can be used to wait for any button to be pressed.
# Provides the same convenience functions as Button.
class AnyButton:
    def __init__(self, buttons):
        self.buttons = buttons

    # @brief Returns True if the button is currently pressed.
    # @return True if the button is currently pressed.
    @property
    def pressed(self):
        for button in self.buttons:
            if button.pressed:
                return True
        return False
    
    ##
    # @brief Waits until any button is pressed.
    # @return None
    def wait(self):
        while not self.pressed:
            time.sleep(0.01)
    
##
# @brief Buzzer class for controlling the vibration motor.
# Provides control over strength and allows user defined patterns.
class Buzzer:
    WEAKEST = 10000
    WEAK = 20000
    MEDIUM = 30000
    STRONG = 45000
    STRONGEST = 65535

    def __init__(self, pin):
        self.output = pwmio.PWMOut(pin, frequency=1000, duty_cycle=0)
        self.enabled = False
        self.power = Buzzer.MEDIUM
    
    # Set the strength of the vibration.
    # Accepts a value between 0 and 65535.
    # Five constants are provided for convenience:
    # Buzzer.WEAKEST, Buzzer.WEAK, Buzzer.MEDIUM, Buzzer.STRONG, Buzzer.STRONGEST
    def setPower(self, power):
        self.power = power
        if self.enabled:
            self.output.duty_cycle = power
        
    def on(self):
        self.output.duty_cycle = self.power
        
    def off(self):
        self.output.duty_cycle = 0
        
    # A pattern is a list of vibration elements, each element is a tuple of (on, off, repeats)
    # i.e. [(200, 400, 2), (300, 300, 1), (100, 100, 3)]
    # Would be: 200ms on, 400ms off, 2 times, then 300ms on, 300ms off, 1 time, then 100ms on, 100ms off, 3 times
    def runPattern(self, pattern):
        for field in pattern:
            for repeats in range(field[2]):
                self.on()
                time.sleep(field[0] / 1000)
                self.off()
                time.sleep(field[1] / 1000)
        self.off()
        
    def sleep(self):
        self.off()
        

class LookupTable:
    metricScalars = {'m': -3, 'u': -6, 'n': -9, 'p': -12, 'k': 3, 'M': 6, 'G': 9, 'T': 12}
    def __init__(self, content:dict) -> None:
        self.allData = content
        self.allData = {k.lower().replace(' ', ''): v for k, v in self.allData.items() if type(k) == str}
        for table in self.allData:
            if 'values' not in self.allData[table]:
                raise RuntimeError(f'Lookup table {table} missing values.')
            if 'type' not in self.allData[table]:
                self.allData[table]['type'] = 'numeric'
            if self.allData[table]['type'] == 'numeric':
                self.allData[table]['values'] = {float(k): v for k, v in self.allData[table]['values'].items()}
            elif self.allData[table]['type'] == 'word':
                self.allData[table]['values'] = {k.lower().replace(' ', ''): v for k, v in self.allData[table]['values'].items() if type(k) == str}
            self.allData[table]['reversed'] = {v: k for k, v in self.allData[table]['values'].items()}
            
        
    # TODO: multiword lookup (returns a list)
    def lookup(self, table, key):
        table = table.lower().replace(' ', '')
        if table not in self.allData:
            raise RuntimeError(f'Lookup table {table} not found.')
        if key == None or key == '':
            raise RuntimeError('Blank key.')
        table = self.allData[table]
        key = key.lower().replace(' ', '')

        if table['type'] == 'numeric':
            # Remove the unit if present. Allow for singular and plural.
            if 'unit' in table:
                unit = table['unit']
                key = key.replace(unit + unit[-1] + 'es', '')
                key = key.replace(unit + 'es', '')
                key = key.replace(unit + 's', '')
                key = key.replace(unit, '')

            if key[-1] in self.metricScalars:
                scalar = self.metricScalars[key[-1]]
                key = key[:-1]
            else:
                scalar = 0

            try:
                key = float(key) * 10 ** scalar
            except ValueError:
                print(table)
                raise RuntimeError(f'Key {key} is not a valid number.')
    
        if key in table['values']:
            return table['values'][key]
        else:
            raise RuntimeError(f'Key {key} not found in lookup table {table}.')
        
    def getkey(self, table, key):
        table = table.lower().strip()
        if table not in self.allData:
            raise RuntimeError(f'Lookup table {table} not found.')
        if key in table['reversed']:
            return table['reversed'][key]
        else:
            raise RuntimeError(f'Key {key} not found in lookup table {table}.')

##
# @brief ePaper display driver.
# Provides functions for drawing to the display.
# The display is a 200x200 black and white EPD display with a SSD1681 driver.
# A basic driver is provided in the firmware to draw to the display.
# That at least lets us see the terminal output.
# But it's slow and doesn't support many features.
# This replacement driver allows a lot more control.
class Display:
    # Hex constants for display colours, use BLACK and WHITE instead
    # for the Adafruit library constructs that require a Pallette object.
    
    ## @brief Black colour constant, or white in dark mode.
    BLACK_COLOUR    = 0xFFFFFF
    ## @brief White colour constant, or black in dark mode.
    WHITE_COLOUR    = 0x000000
    
    # The CircuitPython display library uses the top bit of the command
    # length to specify a delay is to be added after the command.
    # This is given with an extra parameter with the time in milliseconds.
    DELAY           = 0x80

    ## @brief The width of the display in pixels.
    WIDTH           = 200
    ## @brief The height of the display in pixels.
    HEIGHT          = 200
    X_OFFSET        = 0
    Y_OFFSET        = 0
    FIRST_COLUMN    = X_OFFSET
    LAST_COLUMN     = (X_OFFSET + WIDTH - 1)
    FIRST_ROW       = Y_OFFSET
    LAST_ROW        = (Y_OFFSET + HEIGHT - 1)
    X_WINDOW_START  = (FIRST_COLUMN >> 3)
    X_WINDOW_END    = (LAST_COLUMN >> 3)
    Y_WINDOW_START  = FIRST_ROW
    Y_WINDOW_END    = LAST_ROW
    
    # Constants for gate settings.
    EVEN_FIRST      = 0x0
    ODD_FIRST       = 0x4
    NO_INTERLACE    = 0x0
    INTERLACE       = 0x2
    SCAN_FORWARD    = 0x0
    SCAN_BACKWARD   = 0x1
    
    # Constants for border settings.
    WHITE_BORDER    = 0x01
    BLACK_BORDER    = 0x00
    
    # Temperature sensor constants, only two possible values.
    EXTERNAL_SENSOR = 0x48
    INTERNAL_SENSOR = 0x80
    
    # Colours for the display.
    # Only back and white is possible.
    # There is no red colour but enabling the red channel lets us
    # reused some LUTs for a nice grey scale effect.
    # It's also used for fast updates with no ghosting.

    # Unused
    RED_ENABLE      = 0x00
    # Use this for pure black and white.
    RED_DISABLE     = 0x40
    # Use this for grey mode.
    RED_INVERT      = 0x80
    # Used for dark mode.
    BLACK_ENABLE    = 0x00
    # Unused
    BLACK_DISABLE   = 0x04
    # Used for normal mode.
    BLACK_INVERT    = 0x08
    
    # Constants for RAM entry settings.
    Y_FORWARDS      = 0x02
    Y_BACKWARDS     = 0x00
    X_FORWARDS      = 0x01
    X_BACKWARDS     = 0x00
    RIGHT_THEN_DOWN = 0x00
    DOWN_THEN_RIGHT = 0x04
    
    # Constants for power modes.
    # The built in driver should already set these.
    # It's not clear what the difference between the two sleep modes is?
    MODE_NORMAL     = 0x00
    MODE_SLEEP_1    = 0x01
    MODE_SLEEP_2    = 0x03
    
    # Constants for display update sequences.
    # Generally, mode 1 is used for full updates and mode 2 for partial updates.
    # Combine the below commands to make an update sequence.
    # Note that commands like CLOCK_ON and CLOCK_OFF can be used together.
    # In this case, the signal will be enabled before the update and disabled after.
    U_CLOCK_ON      = 0x80
    U_ANALOG_ON     = 0x40
    U_LOAD_TEMP     = 0x20
    U_LOAD_LUT      = 0x10
    U_DISP_MODE_1   = 0x00
    U_DISP_MODE_2   = 0x08
    U_OUTPUT        = 0x04
    U_ANALOG_OFF    = 0x02
    U_CLOCK_OFF     = 0x01
    U_DISPLAY       = (U_OUTPUT | U_LOAD_LUT)
    
    # Settings to pass to CircuitPython for the display.time_to_sleep variable.
    SLOW_UPDATE_TIME     = 5.0
    FAST_UPDATE_TIME     = 0.2


    # Watchy display settings.
    GATE_SETTINGS        = EVEN_FIRST | NO_INTERLACE | SCAN_FORWARD
    ENTRY_MODE           = X_FORWARDS | Y_FORWARDS | RIGHT_THEN_DOWN 
    COLOUR_MODE          = RED_DISABLE | BLACK_INVERT
    BORDER_WAVEFORM      = BLACK_BORDER
    SLOW_UPDATE = (U_CLOCK_ON | U_ANALOG_ON | U_LOAD_TEMP | U_DISPLAY |\
                   U_DISP_MODE_1 | U_CLOCK_OFF | U_ANALOG_OFF)
    FAST_UPDATE = (U_CLOCK_ON | U_ANALOG_ON | U_LOAD_TEMP | U_DISPLAY |\
                   U_DISP_MODE_2)# | U_CLOCK_OFF | U_ANALOG_OFF)
    UPDATE_MODE = SLOW_UPDATE
    
    
    # SSD1608 command set.
    # Just to make it clear what each command is.
    # I'm not going to make my refresh routine full of magic numbers.
    CMD_GATE_SETTING       = 0x01
    CMD_DEEP_SLEEP         = 0x10
    CMD_ENTRY_MODE         = 0x11
    CMD_SOFT_RESET         = 0x12
    CMD_TEMP_SENSOR        = 0x18
    CMD_PANEL_COLOURS      = 0x21
    CMD_MASTER_ACTIVATE    = 0x20
    CMD_UPDATE_SEQUENCE    = 0x22
    CMD_BORDER_WAVEFORM    = 0x3C
    CMD_WRITE_RAM          = 0x24
    CMD_X_WINDOW           = 0x44
    CMD_Y_WINDOW           = 0x45
    CMD_X_OFFSET           = 0x4E
    CMD_Y_OFFSET           = 0x4F
    
    # Switch from the terminal display to our own buffer.
    # There's a lot of settings here that'll get interpreted later
    # when we actually do the update.
    def __init__(self, display):
        self.display = display
        self.objects = displayio.Group()
        self.nextUpdateTime = time.monotonic() + self.display.time_to_refresh
        self.display.root_group = self.objects
        self._darkMode = False
        self._greyMode = False
        self._invertBorder = False
        self._updateColours()
        self.fastRefresh = True
        self._fullRefresh = False
        self.initialRefresh = True
        #self._rebuildUpdateSequence()
        #self.refresh()
        #self.initialRefresh = False
        #self._rebuildUpdateSequence()
        self.BLACK = displayio.Palette(1)
        self.BLACK[0] = self.BLACK_COLOUR
        self.WHITE = displayio.Palette(1)
        self.WHITE[0] = self.WHITE_COLOUR
        
    ##
    # @brief Print the current update sequence to the serial terminal.
    # Prints out the current update sequence as a series of hex values.
    # This is mostly for debugging the driver settings.
    def _dumpSeq(self):
        for i in range(len(self.updateSequence)):
            print("0x{:02X}".format(self.updateSequence[i]), end=" ")
            if i % 16 == 15:
                print()

    ##
    # @brief Turn the colour settings into register values.
    # This routine is called automatically whenever the colour settings are 
    # changed.
    # It converts the colour settings into the register values that are
    # actually sent to the display.
    # It also handles the border waveform settings.
    def _updateColours(self):
        if self.darkMode:
            if self.greyMode:
                self.COLOUR_MODE = self.RED_INVERT | self.BLACK_ENABLE
            else:
                self.COLOUR_MODE = self.RED_DISABLE | self.BLACK_ENABLE
        else:
            if self.greyMode:
                self.COLOUR_MODE = self.RED_INVERT | self.BLACK_INVERT
            else:
                self.COLOUR_MODE = self.RED_DISABLE | self.BLACK_INVERT
        if self.darkMode ^ self.invertBorder:
            self.BORDER_WAVEFORM = 0x00
        else:
            self.BORDER_WAVEFORM = 0x01
        self.modeChanged = True
    
    ##
    # @brief Grey mode setting.
    # If true, the display will produce light grey pixels instead of black.
    # You can layer multiple grey pixels to get a darker shade.
    # It works the opposite way in dark mode i.e. dark grey pixels that lighten.
    # This setting will not be applied until a refresh is performed.
    @property
    def greyMode(self) -> bool:
        return self._greyMode
    
    @greyMode.setter
    def greyMode(self, enabled):
        self._greyMode = enabled
        self._updateColours()
    
    ##
    # @brief Dark mode setting.
    # If true, the display colours will be inverted.
    # This setting will not be applied until a refresh is performed.
    # This settings will also affect the border colour.
    @property
    def darkMode(self) -> bool:
        return self._darkMode
    
    @darkMode.setter
    def darkMode(self, enabled) -> None:
        self._darkMode = enabled
        self._updateColours()
        
    ##
    # @brief Invert border setting.
    # If true, the border colour will be inverted.
    # e.g. if dark mode is enabled, the border will be white.
    # Otherwise the border colour will match the display mode.
    # This setting will not be applied until a refresh is performed.
    @property
    def invertBorder(self) -> bool:
        return self._invertBorder
        
    @invertBorder.setter
    def invertBorder(self, enabled) -> None:
        self._invertBorder = enabled
        self._updateColours()
        
    ##
    # @brief If this setting it true, all refreshs will be slow refreshes.
    # These produce nice sharp images but are fairly slow with lots of flashing.
    @property
    def fullRefresh(self) -> bool:
        return self._fullRefresh or self.initialRefresh
    
    @fullRefresh.setter
    def fullRefresh(self, enabled):
        self._fullRefresh = enabled
        self.modeChanged = True
        
    ##
    # @brief If this setting is true, the next refresh will be a full refresh.
    # This setting is automatically cleared after the next refresh.
    # It's useful if you need to do a full refresh before drawing a new scene.
    @property
    def initialRefresh(self) -> bool:
        return self._initialRefresh
    
    @initialRefresh.setter
    def initialRefresh(self, enabled):
        self._initialRefresh = enabled
        self.modeChanged = True

    ##
    # @brief If this setting is true, the driver will do a fast refresh.
    # This will prioritise speed over image quality.
    # It may be useful for animation, if you need redrawing every second.
    @property
    def fastRefresh(self) -> bool:
        return self._fastRefresh
    
    @fastRefresh.setter
    def fastRefresh(self, enabled):
        self._fastRefresh = enabled
        if enabled:
            self.UPDATE_MODE = self.FAST_UPDATE
            self.frameDelay = self.FAST_UPDATE_TIME
        else:
            self.UPDATE_MODE = self.SLOW_UPDATE
            self.frameDelay = self.SLOW_UPDATE_TIME
        self.modeChanged = True

        
    ##
    # @brief Sets the area of the display to update.
    # This sets the area of the display that will be updated on the next
    # refresh. It will not be applied until the next refresh.
    # @param x1 The left offset of the start of the update area.
    # @param y1 The top offset of the start of the update area.
    # @param x2 The left offset of the end of the update area.
    # @param y2 The top offset of the end of the update area.
    def setWindow(self, x1, y1, x2, y2):
        self.FIRST_COLUMN = x1
        self.LAST_COLUMN = x2
        self.FIRST_ROW = y1
        self.LAST_ROW = y2
        self.X_WINDOW_START = (self.FIRST_COLUMN >> 8)
        self.X_WINDOW_END = (self.LAST_COLUMN >> 8)
        self.Y_WINDOW_START = self.FIRST_ROW
        self.Y_WINDOW_END = self.LAST_ROW
        self.modeChanged = True
        
    ##
    # @brief Clear the update window return to updating the whole display.
    # This will set the update window to the whole display.
    def clearWindow(self):
        self.setWindow(0, 0, self.WIDTH, self.HEIGHT)

    ##
    # @brief Run a display update with whatever settings are currently set.
    # This will run a display update with whatever settings are currently set.
    # If the refresh delay has not been met, this function will block until it
    # the display is ready.
    def refresh(self):
        if self.modeChanged:
            self._rebuildUpdateSequence()
        time.sleep(self.display.time_to_refresh)
        while True:
            try:
                self.display.refresh()
            except RuntimeError:
                time.sleep(0.1)
            else:
                break
        if self.initialRefresh:
            self.initialRefresh = False
        
    ##
    # @brief Convert the driver settings into a command sequence.
    # This will convert the current driver settings into a command sequence
    # that can be sent to the display in the format CircuitPython expects.
    # This is called automatically before refresh() if any settings have changed.
    def _rebuildUpdateSequence(self):
        if self.initialRefresh:
            updateMode = self.SLOW_UPDATE
        else:
            updateMode = self.UPDATE_MODE
            
            
        self.updateSequence = bytes([
            self.CMD_SOFT_RESET, self.DELAY, 20,
            self.CMD_GATE_SETTING, 3,
                (self.WIDTH - 1) & 0xFF, 
                ((self.WIDTH - 1) >> 8) & 0xFF,
                self.GATE_SETTINGS,
            self.CMD_BORDER_WAVEFORM, 1, self.BORDER_WAVEFORM,
            self.CMD_TEMP_SENSOR, 1, self.INTERNAL_SENSOR,
            self.CMD_PANEL_COLOURS, 2, self.COLOUR_MODE, 0x00,
            self.CMD_ENTRY_MODE, 1, self.ENTRY_MODE,
        ])

        if self.fastRefresh and not self.fullRefresh:
            self.updateSequence += bytes([
                0x37, 10, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00,
            ])
        
        if not self.fullRefresh:
            if not self.darkMode:
                self.updateSequence += bytes([
                    0x0C, 4, 0xFB, 0x89, 0xF6, 0x0F,
                    0x04, 3, 0x4B, 0xF0, 0x0A,
                    0x03, 1, 0x12,
                ])
            else:
                self.updateSequence += bytes([
                    0x03, 1, 0x10,
                    0x04, 3, 0x4B, 0xF0, 0x0A,
                    0x0C, 4, 0xFB, 0x89, 0xF6, 0x35,
                ])
            
        self.updateSequence += bytes([
            self.CMD_X_WINDOW, 2, 
                self.X_WINDOW_START, 
                self.X_WINDOW_END,
            self.CMD_Y_WINDOW, 4, 
                self.Y_WINDOW_START & 0xFF, 
                (self.Y_WINDOW_START >> 8) & 0xFF, 
                self.Y_WINDOW_END & 0xFF, 
                (self.Y_WINDOW_END >> 8) & 0xFF,
            self.CMD_X_OFFSET, 1, self.X_OFFSET,
            self.CMD_Y_OFFSET, 2, 
                self.Y_OFFSET & 0xFF, 
                (self.Y_OFFSET >> 8) & 0xFF,
            self.CMD_UPDATE_SEQUENCE, 1, updateMode,
        ])

        if self.fastRefresh and not self.fullRefresh:
            #if not self.darkMode and not self.greyMode:
            #    self.updateSequence += bytes([
            #    ])

            if self.greyMode:
                altColour = self.COLOUR_MODE
            else:
                if self.darkMode:
                    altColour = self.RED_INVERT | self.BLACK_INVERT
                else:
                    altColour = self.RED_INVERT | self.BLACK_ENABLE
                
            self.updateSequence += bytes([
                #self.CMD_PANEL_COLOURS, 2, self.COLOUR_MODE, 0x00,
                #self.CMD_MASTER_ACTIVATE, 0,
                self.CMD_PANEL_COLOURS, 2, altColour, 0x00,
                self.CMD_UPDATE_SEQUENCE, 1, updateMode,
                self.CMD_MASTER_ACTIVATE, 0,
                self.CMD_PANEL_COLOURS, 2, self.COLOUR_MODE, 0x00,
                self.CMD_UPDATE_SEQUENCE, 1, updateMode,
            ])

        self.display.update_refresh_mode(self.updateSequence, self.frameDelay)
        self.modeChanged = False
        
    ##
    # @brief Returns to opposite of the given colour.
    # @param colour The colour to invert (either BLACK or WHITE from this class).
    # The display is black and white so there are only two colours.
    # This still useful for a lot of internal drawing functions.
    def opposite(self, colour):
        if colour == self.BLACK:
            return self.WHITE
        else:
            return self.BLACK
    ##
    # @brief Creates a rectangle object.
    # @param x The x position of the top left corner of the rectangle.
    # @param y The y position of the top left corner of the rectangle.
    # @param width The width of the rectangle.
    # @param height The height of the rectangle.
    # @param colour The colour of the rectangle (either BLACK or WHITE from this class).
    # @param outlineThickness The thickness of the outline of the rectangle, zero for no outline (default).
    # @param roundingRadius The radius of the corners of the rectangle, zero for no rounding (default).
    # @return The rectangle object.
    def rectangle(self, x, y, width, height, colour=None, noFill=False, outlineThickness=None, roundingRadius=0):
        # https://docs.circuitpython.org/en/latest/shared-bindings/vectorio/index.html
        # https://docs.circuitpython.org/projects/display-shapes/en/latest/
        if not colour:
            colour = self.BLACK
        if noFill and outlineThickness == None:
            outlineThickness = 1
        if roundingRadius > 0:
            if noFill:
                rect = adafruit_display_shapes.roundrect.RoundRect(x=x, y=y, width=width, height=height, r=roundingRadius, fill=None, outline=self.opposite(colour)[0], stroke=outlineThickness)
            else:
                rect = adafruit_display_shapes.roundrect.RoundRect(x=x, y=y, width=width, height=height, r=roundingRadius, fill=colour[0], outline=self.opposite(colour)[0], stroke=outlineThickness)
        elif outlineThickness:
            if noFill:
                rect = adafruit_display_shapes.rect.Rect(x=x, y=y, width=width, height=height, fill=None, outline=colour[0], stroke=outlineThickness)
            else:
                rect = adafruit_display_shapes.rect.Rect(x=x, y=y, width=width, height=height, fill=colour[0], outline=self.opposite(colour)[0], stroke=outlineThickness)
        elif not noFill:
            rect = vectorio.Rectangle(pixel_shader=colour, x=x, y=y, width=width, height=height)
        else:
            return None
        self.objects.append(rect)
        return rect
    
    ##
    # @brief Creates a circle object.
    # @param x The x position of the centre of the circle.
    # @param y The y position of the centre of the circle.
    # @param radius The radius of the circle.
    # @param noFill If true, the circle will be an outline only.
    # @param colour The colour of the circle (either BLACK or WHITE from this class).
    # @return The circle object.
    def circle(self, x, y, radius, colour=None, noFill=False, outlineThickness=0):
        if not colour:
            colour = self.BLACK
        if noFill:
            circ = adafruit_display_shapes.circle.Circle(x0=x, y0=y, r=radius, fill=None, outline=colour[0], stroke=outlineThickness)
        elif outlineThickness > 0:
            circ = adafruit_display_shapes.circle.Circle(x0=x, y0=y, r=radius, fill=colour[0], outline=self.opposite(colour)[0], stroke=outlineThickness)
        else:
            circ = vectorio.Circle(pixel_shader=colour, x=x, y=y, radius=radius)
        self.objects.append(circ)
        return circ
    
    ##
    # @brief Creates a triangle object.
    # @param p1 The first point of the triangle as a (x, y) tuple.
    # @param p2 The second point of the triangle as a (x, y) tuple.
    # @param p3 The third point of the triangle as a (x, y) tuple.
    # @param colour The colour of the triangle (either BLACK or WHITE from this class). colour The colour of the triangle (either BLACK or WHITE from this class).
    # @param noFill If true, the triangle will be an outline only.
    # @param outlineThickness The thickness of the outline of the triangle, zero for no outline (default).
    # @return The triangle object.
    def triangle(self, p1, p2, p3, colour=None, noFill=False, outlineThickness=0):
        if not colour:
            colour = self.BLACK
        if noFill:
            tri = adafruit_display_shapes.triangle.Triangle(x0=p1[0], y0=p1[1], x1=p2[0], y1=p2[1], x2=p3[0], y2=p3[1], fill=None, outline=colour[0], stroke=outlineThickness)
        else:
            tri = adafruit_display_shapes.triangle.Triangle(x0=p1[0], y0=p1[1], x1=p2[0], y1=p2[1], x2=p3[0], y2=p3[1], fill=colour[0], outline=self.opposite(colour)[0], stroke=outlineThickness)
        self.objects.append(tri)
        return tri

    ##
    # @brief Creates a polygon object.
    # @param x The x offset of the polygon.
    # @param y The y offset of the polygon.
    # @param points A list of points that make up the polygon. A list of (x, y) tuples.
    # @param colour The colour of the polygon (either BLACK or WHITE from this class).
    # @return The polygon object.
    def polygon(self, x, y, points, colour=None) -> vectorio.Polygon:
        if not colour:
            colour = self.BLACK
        poly = vectorio.Polygon(pixel_shader=colour, points=points, x=x, y=y)
        self.objects.append(poly)
        return poly

    ##
    # @brief Creates a line object.
    # @param p1 The first point of the line as a (x, y) tuple.
    # @param p2 The second point of the line as a (x, y) tuple.
    # @param colour The colour of the line (either BLACK or WHITE from this class).
    # @return The line object.
    def line(self, p1, p2, colour=None) -> adafruit_display_shapes.line.Line:
        if not colour:
            colour = self.BLACK
        line = adafruit_display_shapes.line.Line(x1=p1[0], y1=p1[1], x2=p2[0], y2=p2[1], color=colour[0])
        self.objects.append(line)
        return line
    
    def image(self, filename, x, y) -> displayio.TileGrid:
        # https://docs.circuitpython.org/projects/imageload/en/latest/examples.html
        image, palette = adafruit_imageload.load(filename)
        tg = displayio.TileGrid(image, pixel_shader=palette, x=x, y=y)
        self.objects.append(tg)
        return tg
    
    def sprites(self, filename, x, y, tileWidth, tileHeight, defaultSprite=0) -> displayio.TileGrid:
        image, palette = adafruit_imageload.load(filename)
        tg = displayio.TileGrid(image, pixel_shader=palette, width=1, height=1, tile_width=tileWidth, tile_height=tileHeight, default_tile=defaultSprite, x=x, y=y)
        self.objects.append(tg)
        return tg
    
    TOP_LEFT =      (0.0, 0.0)
    TOP_CENTRE =    (0.5, 0.0)
    TOP_RIGHT =     (1.0, 0.0)
    CENTRE_LEFT =   (0.0, 0.5)
    CENTRE =        (0.5, 0.5)
    CENTRE_RIGHT =  (1.0, 0.5)
    BOTTOM_LEFT =   (0.0, 1.0)
    BOTTOM_CENTRE = (0.5, 1.0)
    BOTTOM_RIGHT =  (1.0, 1.0)

    ##
    # @brief Creates a text object on the display.
    # Create a text object on the display with the Adafruit Display Text library.
    # The object is returned and added to the display.
    # You can change the returned object's variables such as position or text
    # and it will be automatically updated when the display is refreshed.
    # @param text The text to display.
    # @param x The x offset of the text, default depends on the alignment.
    # @param y The y offset of the text, default depends on the alignment.
    # @param colour The colour of the text (either BLACK or WHITE from this class).
    # @param align The alignment of the text (either TOP_LEFT, TOP_CENTRE, TOP_RIGHT, CENTRE_LEFT, CENTRE, CENTRE_RIGHT, BOTTOM_LEFT, BOTTOM_CENTRE, BOTTOM_RIGHT from this class).
    # @param bg If true, the text will have a background fill of the opposite colour to the text.
    # @param font The font to use for the text, default is the built in font.
    # @return A configurable text object.
    def text(self, text, x=None, y=None, colour=None, align=None, bg=False, font=None, scale=1):
        # https://docs.circuitpython.org/projects/display_text/en/latest/api.html
        if not colour:
            colour = self.BLACK
        if not font:
            font = terminalio.FONT
        if align == None:
            align = self.TOP_LEFT
        if x == None:
            x = int(align[0] * self.WIDTH)
        if y == None:
            y = int(align[1] * self.HEIGHT)
            
        if bg:
            bgColour = self.opposite(colour)[0]
        else:
            bgColour = None
            
        text = adafruit_display_text.label.Label(font, text=text, anchor_point=(0.5, 0.5), color=colour[0], scale=scale, anchored_position=(100, 100), background_color=bgColour)
        self.objects.append(text)
        return text
        
##
# @brief A class for the BMA423 accelerometer.
# The Watchy uses a BMA423 accelerometer, which is a 3-axis accelerometer with
# a built in step counter, activity recognition, and tilt detection interrupts.
# This class provides a simple interface to the accelerometer.
# It's a pretty complicated device and I couldn't find a library for it, so
# I've written my own.
class Accelerometer:
    # Maximum i2c burst read/write length.
    _MAX_TRANSFER_LENGTH = 8
    
    # Should return this value when the device ID is read.
    _BMA423_ID = 0x13
    
    # Definitions for *all* the bits and registers with Adafruit's Register
    # library. I've tried to keep the names the same as the datasheet.
    # But I am using camelCase instead of snake_case. Sorry.
    # These definitions simplify the code a *lot* as they make setting a flag
    # or reading a register a one-liner, just like setting a variable.

    _deviceId = ROUnaryStruct(0x00, 'B')
    
    # Error codes
    _errReg = ROUnaryStruct(0x02, 'B')
    _auxErr = ROBit(0x02, 7)
    _fifoErr = ROBit(0x02, 6)
    _errorCode = ROBits(3, 0x02, 2)
    _cmdErr = ROBit(0x02, 1)
    _fatalErr = ROBit(0x02, 0)

    _auxManOp = RWBit(0x03, 2)
    _cmdRdy = ROBit(0x03, 4)
    _drdyAux = ROBit(0x03, 5)
    _drdyAcc = ROBit(0x03, 7)

    # Sensor registers. They have an annoying 12-bit format.
    _auxX = ROBits(num_bits=12, register_address=0x0A, lowest_bit=4, register_width=2, signed=True)
    _auxY = ROBits(num_bits=12, register_address=0x0C, lowest_bit=4, register_width=2, signed=True)
    _auxZ = ROBits(num_bits=12, register_address=0x0E, lowest_bit=4, register_width=2, signed=True)
    _auxR = ROBits(num_bits=12, register_address=0x10, lowest_bit=4, register_width=2, signed=True)
    _accX = ROBits(num_bits=12, register_address=0x12, lowest_bit=4, register_width=2, signed=True)
    _accY = ROBits(num_bits=12, register_address=0x14, lowest_bit=4, register_width=2, signed=True)
    _accZ = ROBits(num_bits=12, register_address=0x16, lowest_bit=4, register_width=2, signed=True)
    
    _sensorTime = ROBits(num_bits=24, register_address=0x18, lowest_bit=0, register_width=3)
    _porDetected = ROBit(register_address=0x1B, bit=0)
    _stepCounterOut = ROBits(num_bits=32, register_address=0x1E, lowest_bit=0, register_width=4)
    _temperature = ROBits(num_bits=8, register_address=0x22, lowest_bit=0, register_width=1, signed=True)
    _fifoLength = ROBits(num_bits=14, register_address=0x24, lowest_bit=0, register_width=2)
    _fifoData = ROBits(num_bits=8, register_address=0x26, lowest_bit=0, register_width=1)
    _activityTypeOut = ROBits(num_bits=2, register_address=0x27, lowest_bit=0, register_width=1)
    _statusReg = ROUnaryStruct(0x2A, 'B')
    _internalStatus = ROBits(num_bits=5, register_address=0x2A, lowest_bit=0, register_width=1)
    _axesRemapError = ROBit(register_address=0x2A, bit=5)
    _odr50hzError = ROBit(register_address=0x2A, bit=6)
    _odrHighError = ROBit(register_address=0x2A, bit=7)

    _accOdr = RWBits(num_bits=4, register_address=0x40, lowest_bit=0, register_width=1)
    _accBwp = RWBits(num_bits=3, register_address=0x40, lowest_bit=4, register_width=1)
    _accPerfMode = RWBit(register_address=0x40, bit=7)
    _accRange = RWBits(num_bits=2, register_address=0x41, lowest_bit=0, register_width=1)
    _auxOdr = RWBits(num_bits=4, register_address=0x46, lowest_bit=0, register_width=1)
    _auxOffset = RWBits(num_bits=4, register_address=0x46, lowest_bit=4, register_width=1)
    _accFifoDowns = RWBits(num_bits=3, register_address=0x45, lowest_bit=4, register_width=1)
    _accFifoFiltData = RWBit(register_address=0x45, bit=7)
    _fifoWaterMark = RWBits(num_bits=13, register_address=0x46, lowest_bit=0, register_width=2)
    _fifoStopOnFull = RWBit(register_address=0x48, bit=0)
    _fifoTimeEn = RWBit(register_address=0x48, bit=1)
    _fifoTagInt2En = RWBit(register_address=0x49, bit=2)
    _fifoTagInt1En = RWBit(register_address=0x49, bit=3)
    _fifoHeader = RWBit(register_address=0x49, bit=4)
    _fifoAuxEn = RWBit(register_address=0x49, bit=5)
    _fifoAccEn = RWBit(register_address=0x49, bit=6)
    _auxDeviceAddr = RWBits(num_bits=7, register_address=0x4B, lowest_bit=1, register_width=1)
    _auxRdBurst = RWBits(num_bits=2, register_address=0x4C, lowest_bit=0, register_width=1)
    _auxManualEn = RWBit(register_address=0x4C, bit=7)
    _auxReadAddr = RWBits(num_bits=8, register_address=0x4D, lowest_bit=0, register_width=1)
    _auxWriteAddr = RWBits(num_bits=8, register_address=0x4E, lowest_bit=0, register_width=1)
    _auxWriteData = RWBits(num_bits=8, register_address=0x4F, lowest_bit=0, register_width=1)
    _intLatch = RWBit(register_address=0x55, bit=0)
    _initCtrl = RWBits(num_bits=8, register_address=0x59, lowest_bit=0, register_width=1)
    _asicLsb = RWBits(num_bits=8, register_address=0x5B, lowest_bit=0, register_width=1)
    _asicMsb = RWBits(num_bits=8, register_address=0x5C, lowest_bit=0, register_width=1)
    _featuresIn = Struct(0x5E, 'B' * _MAX_TRANSFER_LENGTH)
    _intErr1 = RWBit(register_address=0x5F, bit=1)
    _intErr2 = RWBit(register_address=0x5F, bit=2)
    _nvmProgEn = RWBit(register_address=0x6A, bit=1)
    _spi3 = RWBit(register_address=0x6B, bit=0)
    _ifMode = RWBit(register_address=0x6B, bit=5)
    _accSelfTestEn = RWBit(register_address=0x6D, bit=0)
    _accSelfTextSign = RWBit(register_address=0x6D, bit=2)
    _accSelfTestAmp = RWBit(register_address=0x6D, bit=3)
    _nvSpiEn = RWBit(register_address=0x70, bit=0)
    _nvi2cWdtSel = RWBit(register_address=0x70, bit=1)
    _nvi2cWdtEn = RWBit(register_address=0x70, bit=2)
    _nvAccOffEn = RWBit(register_address=0x70, bit=3)
    _offAccX = RWBits(num_bits=8, register_address=0x71, lowest_bit=0, register_width=1)
    _offAccY = RWBits(num_bits=8, register_address=0x72, lowest_bit=0, register_width=1)
    _offAccZ = RWBits(num_bits=8, register_address=0x73, lowest_bit=0, register_width=1)
    _advPowerSave = RWBit(register_address=0x7C, bit=0)
    _fifoSelfWakeup = RWBit(register_address=0x7C, bit=1)
    _auxEn = RWBit(register_address=0x7D, bit=0)
    _accEn = RWBit(register_address=0x7D, bit=2)
    _cmd = RWBits(num_bits=8, register_address=0x7E, lowest_bit=0, register_width=1)

    _SMALL_CONFIG_LENGTH = 70
    _featureBuffer = bytearray(_SMALL_CONFIG_LENGTH)

    # Possible values for the various settings registers.
    # Keeping a list and automatically checking them with property setters
    # makes sure an incorrect value will never be written to the device.

    opts = LookupTable({
        'data rate': {
        'unit': 'hz', 
        'values': {
          0.78 : 0x01,    1.5 : 0x02,    3.1 : 0x03,     6.25 : 0x04,
         12.5  : 0x05,   25   : 0x06,   50   : 0x07,   100    : 0x08,
        200    : 0x09,  400   : 0x0a,  800   : 0x0b,  1600    : 0x0c,
        }},

        'bandwidth': {
        'unit': 'sample',
        'values': {
         1: 0x00,  2: 0x01,  4: 0x02,   8: 0x03, 
        16: 0x04, 32: 0x05, 64: 0x06, 128: 0x07,
        }},
        
        'acceleration range': {
        'unit': 'g',
        'values': {
        2: 0x00, 4: 0x01, 8: 0x02, 16: 0x03,
        }},
        
        'cmd': {
        'type': 'word',
        'values': {
        'nvmProg'   : 0xA0, 'fifoFlush':  0xB0, 'softReset' : 0xB6,
        }},
        
        'features': {
        'type': 'multiword',
        'values': {
            'step counter'          : (0x3B, 0x04), 
            'activity detection'    : (0x3B, 0x08),
            'tilt detection'        : (0x40, 0x01), 
            'single tap detection'  : (0x3C, 0x01),
            'double tap detection'  : (0x3E, 0x01),
        }},
    })

    def __init__(self, cfg) -> None:
        # The register library requires and I2CDevice object.
        # And it *must* be available through the class property 'i2c_device'.
        self.cfg = cfg
        i2c = self.cfg['i2c']
        self.i2cBus = i2c
        self.i2c_device = adafruit_bus_device.i2c_device.I2CDevice(i2c, 0x18)               
        # Interrupt subclasses objects.
        # Hopefully this is a little simplier than creating twenty different
        # properties with the same settings layout.
        intStatus = self.InterruptMask(self.i2c_device, rw=False, address=0x1C, extAddress=0x1D, ffull=0, fwm=1, aux=5, acc=7)
        int1 = self.InterruptMask(self.i2c_device, rw=True, address=0x56, extAddress=0x58, ctrlAddress=0x53, ffull=0, fwm=1, drdy=2)
        int2 = self.InterruptMask(self.i2c_device, rw=True, address=0x57, extAddress=0x58, ctrlAddress=0x54, ffull=4, fwm=5, drdy=6)
        print('ACCEL: Init...', end='')
        self.reset()
        print('done')
            
    ##
    # @brief Resets the device to the default configuration.
    # Resets all settings to their default values, and clears the FIFO.
    # This is called automatically at startup or when the configuration
    # is modified.
    def reset(self):
        if self._deviceId != self._BMA423_ID:
            raise RuntimeError("ACCEL: Incorrect device ID")

        # Do a soft reset every time including initially since there's no
        # reset pin and the Watchy doesn't power cycle the device.
        self._runCmd('softReset')
        # Power saving mode *must* be disabled before sending the configuration.
        self.lowPowerMode = False
        self._initCtrl = 0x00
        # This is a pretty big block of data from extracted from the example 
        # driver. There's only a few settings that are actually changed.
        print(f'Free RAM: {gc.mem_free()}')
        gc.collect()
        print(f'Free RAM: {gc.mem_free()}')
        with open('accelcfg.dat', 'rb') as f:
            self._loadMainConfig(f.read())
        self._initCtrl = 0x01
        gc.collect()
        
        #waitCycles = 15
        #for n in range(waitCycles):
        #    print(f'[{n}] Status: {self._internalStatus}')
        #    if self._internalStatus == 0x01:
        #        break
        #    time.sleep(0.01)
        #    if n == waitCycles - 1:
        #        raise RuntimeError(f'ACCEL: Init timed out: {self.errorMessage}')
        
        # The driver will take some time to apply the configuration.
        # It'll set the internal status register when done.
        # If this doesn't happen within 150ms, something went wrong.
        time.sleep(0.15)
        if self._internalStatus != 0x01:
            print(f'Error: {self.errorMessage}')
            raise RuntimeError(f'ACCEL: Init timed out: {self.errorMessage}')
        self._accEn = True
        self.dataRate = '50 Hz'
        self.bandwidthParameter = '2 samples'
        self.accelerationRange = '2g'
        self.lowPowerMode = True

    ##
    # @brief Runs a given command.
    # This is a blocking function. It will wait until the command is complete.
    # If will not wait before running a soft reset as the status may not be sane.
    # @param cmd The command to run as a string. Possible values: 'nvmProg', 'fifoFlush', 'softReset'
    # @throws ValueError if the command is invalid.
    def _runCmd(self, cmd) -> None:
        if cmd != 'softReset':
            while not self._cmdRdy:
                time.sleep(0.001)
        cmd = self.opts.lookup('cmd', cmd)
        self._cmd = cmd
        time.sleep(0.01)
        while not self._cmdRdy:
            time.sleep(0.001)
            
            
        
    ##
    # @brief Sets the address in the _featuresIn register.
    # This property provides an a quick way to set the internal address that
    # the _featuresIn register will write to.
    # This is a twelve bit address that is split across two registers.
    # It will throw a value error if an invalid address is given.
    # This process is taken from the code example provided by Bosch,
    # and is not documented in the datasheet.
    @property
    def _featureAddress(self):
        return (self._asicLsb & 0x0F | self._asicMsb << 4) * 2
    
    @_featureAddress.setter
    def _featureAddress(self, value):
        if value < 0 or value > 0x2000 - self._MAX_TRANSFER_LENGTH:
            raise ValueError('ACCEL: Invalid feature address')
        self._asicLsb = (value >> 1) & 0x0F
        self._asicMsb = value >> 5

    ##
    # @brief Turns the low power mode on or off.
    # Power saving mode reduces the power consumption of the device by
    # reducing the data rate and bandwidth parameter.
    # The device will draw 150uA in normal mode, and 14uA in low power mode.
    # It makes register writes very slow (1000us vs 2us), but after the
    # configuration is set there's not much need to change them again.
    @property
    def lowPowerMode(self):
        return self._advPowerSave and not self._accPerfMode
    
    @lowPowerMode.setter
    def lowPowerMode(self, value):
        self._advPowerSave = value
        microcontroller.delay_us(450)
        self._accPerfMode = not value
        
    ##
    # @brief Sets the output data rate of the device.
    # This property provides sets the data rate and provides a check to make
    # sure that the data rate is valid. A ValueError will be thrown if an
    # invalid data rate is given.
    # Must be a value from the DATA_RATE dictionary.
    # Note that many of the data rates are not supported in low power mode.
    # Also, some features have a minimum data rate requirement in low power mode.
    # Many require a data rate of at least 50 Hz. 
    # The tap detection requires a data rate of at least 200 Hz.
    @property
    def dataRate(self):
        return self.opts.getkey('data rate', self._accOdr)
    
    @dataRate.setter
    def dataRate(self, value):
        value = self.opts.lookup('data rate', value)
        self._writeChecks()
        self._accOdr = value
        
    ##
    # @brief Sets the bandwidth parameter of the device.
    # This property provides sets the bandwidth parameter and provides a check
    # to make sure that the bandwidth parameter is valid. A ValueError will be
    # thrown if an invalid bandwidth parameter is given.
    # Must be a value from the BANDWIDTH dictionary.
    @property
    def bandwidthParameter(self):
        return self.opts.getkey('bandwidth', self._accBwp)
    
    @bandwidthParameter.setter
    def bandwidthParameter(self, value):
        value = self.opts.lookup('bandwidth', value)
        self._writeChecks()
        self._accBwp = value

    @property
    def accelerationRange(self):
        return self.opts.getkey('acceleration range', self._accRange)
    
    @accelerationRange.setter
    def accelerationRange(self, value):
        value = self.opts.lookup('acceleration range', value)
        self._writeChecks()
        self._accRange = value

    @property
    def enableStepCounter(self):
        return self._stepCounterEn
    ##
    # @brief Returns the current acceleration in g's.
    # This property returns a tuple with the current acceleration in g's scaled 
    # to the current sensor settings. The tuple is in an (x, y, z) format.
    # This property is read only.
    @property
    def acceleration(self):
        x, y, z, = self._accX, self._accY, self._accZ
        data = [(axis + 1) / 2 ** (10 - self._accRange) for axis in (x, y, z)]
        return {'x': data[0], 'y': data[1], 'z': data[2]}

    @property
    def stepCount(self):
        return self._stepCounterOut

    @property
    def temperature(self):
        temp = float(self._temperature + 23)
        if self.cfg['tempUnit'] == 'C':
            return temp
        elif self.cfg['tempUnit'] == 'F':
            return temp * 9 / 5 + 32
        elif self.cfg['tempUnit'] == 'K':
            return temp + 273.15
        else:
            raise ValueError('ACCEL: Invalid temperature unit')
            
    @property
    def temperatureUnits(self):
        return self.cfg['tempUnit']
    
    @temperatureUnits.setter
    def temperatureUnits(self, value):
        if value.upper() in ['C', 'CEL', 'CELSIUS']:
            self.cfg['tempUnit'] = 'C'
        elif value.upper() in ['F', 'FAH', 'FAHRENHEIT']:
            self.cfg['tempUnit'] = 'F'
        elif value.upper() in ['K', 'KEL', 'KELVIN']:
            self.cfg['tempUnit'] = 'K'
        else:
            raise ValueError('ACCEL: Invalid temperature unit')

    @property
    def fullTemperatureUnit(self):
        return {'C': 'Celsius', 'F': 'Fahrenheit', 'K': 'Kelvin'}[self.cfg['tempUnit']]
    
    ##
    # @brief Returns the current activity.
    # The activity is determined by the accelerometer's activity recognition engine.
    # It can be one of four values: 'no activity', 'walking', 'running', or 'unknown'.
    @property
    def activity(self) -> str:
        act = self._activityTypeOut
        if act == 0x00:
            return 'no activity'
        elif act == 0x01:
            return 'walking'
        elif act == 0x02:
            return 'running'
        else:
            return 'unknown'
        
    ##
    # @brief Returns an error message based on the device status registers.
    # The error message will be a string that describes the error, based on the
    # flags in the status registers. If there is no error, the string will be
    # 'No error.'. If there are multiple error flags set, the string will
    # describe all of the errors in the order that they are checked.
    # If the error cannot be determined, the string will be 'Unknown error.'.
    # This property is read only.
    @property
    def errorMessage(self) -> str:
        if not self.error:
            return 'No error.'
        errors = ''
        if self._auxErr:
            errors += 'Auxiliary sensor error. '
        if self._fifoErr:
            errors += 'FIFO error. '
        if self._cmdErr:
            errors += 'Command error. '
        if self._fatalErr:
            errors += 'Fatal error. '
        if self._errorCode != 0x00:
            errors += 'Error in ACC_CONF register. '
        if self._odrHighError:
            errors += 'ODR set too low for feature.'
        if self._odr50hzError:
            errors += 'ODR requires a minimum of 50 Hz in power saving mode. '
        if self._axesRemapError:
            errors += 'Axes remapped incorrectly. '
        if self._internalStatus == 0x00:
            errors += 'Not initialized. '
        elif self._internalStatus == 0x02:
            errors += 'Initialization error. '
        elif self._internalStatus == 0x03:
            errors += 'Invalid driver. '
        elif self._internalStatus == 0x04:
            errors += 'Sensor stopped. '
        elif self._intErr1: 
            errors += 'Internal timeout.'
        elif self._intErr2:
            errors += 'Internal error.'
        if errors == '':
            return 'Unknown error.'
        else:
            return errors
        
    ##
    # @brief Returns the error status of the device.
    # This property returns a boolean value that indicates if there is an error
    # with the device. It returns True if *any* status register indicates an
    # error, and False if all status registers indicate no error.
    # This property is read only.
    @property
    def error(self) -> bool:
        return (self._errReg & 0xDF) != 0 or self._statusReg != 0x01 or self._intErr1 == True or self._intErr2 == True
    
    ##
    # @brief Runs checks before a register write.
    # This method runs checks before a register write to make sure that the
    # there is no errors, and the correct delays are used.
    # 
    def _writeChecks(self) -> None:
        if self.error:
            raise RuntimeError(f'ACCEL: ERROR: {self.errorMessage}')
        if self.lowPowerMode == False:
            time.sleep(0.001)

    ##
    # @brief Load the configuration data into the device.
    # This method loads the configuration data into the accelerometer.
    # This method can only be called after the device has been initialized.
    # It cannot be called when power saving mode is enabled.
    def _loadMainConfig(self, config) -> None:
        if self.lowPowerMode:
            raise RuntimeError('ACCEL: Cannot load config in low power mode.')
        if len(config) >= 0x2000:
            raise RuntimeError('ACCEL: Config too large.')
        if len(config) % 8 != 0:
            raise RuntimeError('ACCEL: Config size must be a multiple of eight.')

        # A lot is going on behind the scenes here.
        # _featuresAddress is actually a pair of registers that are used to
        # specify the address of the window that _featuresIn writes to.
        # _featuresIn reads and writes a list of eight bytes at a time.
        dataLen = self._MAX_TRANSFER_LENGTH
        for offset in range(0, len(config), dataLen):
            self._featureAddress = offset
            data = list(config[offset:offset + dataLen])
            self._featuresIn = data
            if self._featuresIn != tuple(data):
                raise RuntimeError('ACCEL: Config verify failed.')

    ##
    # @brief Read the configuration data from the device.
    # @param buffer The buffer to read the configuration data into. This must be a bytearray of length _SMALL_CONFIG_LENGTH .
    # This method reads the configuration data from the accelerometer.
    # Only the simple configuration data is read. The advanced configuration
    # data is only written to via the _loadMainConfig() method.
    # This method is used to cache the configuration data in the driver.
    def _readConfig(self, buffer: bytearray) -> None:
        dataLen = self._MAX_TRANSFER_LENGTH
        for offset in range(0, self._SMALL_CONFIG_LENGTH, dataLen):
            self._featureAddress = offset
            buffer[offset:offset+dataLen] = bytearray(self._featuresIn)

    ##
    # @brief Write the configuration data to the device.
    # @param buffer The buffer to write the configuration data from.
    # This method writes the configuration data to the accelerometer.
    # It is used to write the simple confirguration data cached in the driver
    # to the accelerometer after it has been modified.
    def _writeConfig(self, buffer: bytearray) -> None:
        dataLen = self._MAX_TRANSFER_LENGTH
        for offset in range(0, self._SMALL_CONFIG_LENGTH, dataLen):
            self._featureAddress = offset
            data = tuple(buffer[offset:offset + dataLen])
            self._featuresIn = data
            # Read back the data to verify it was written correctly.
            if self._featuresIn != tuple(data):
                raise RuntimeError('ACCEL: Config update verify failed.')
            
    def setFeature(self, feature: str) -> None:
        pass
        #TODO

    ##
    # @brief A subclass for managing interrupt registers.
    class InterruptMask:
        def __init__(self, dev, rw, address, extAddress, ctrlAddress=None, ffull=0, fwm=1, aux=None, acc=None, drdy=None) -> None:
            self.i2c_device = dev
            if rw:
                bitType = RWBit
            else:
                bitType = ROBit
            # Interrupt on error.
            self.errorIntOut = bitType(register_address=address, bit=7)
            # Interrupt on motion or when no motion is detected.
            self.anyNoMotionOut = bitType(register_address=address, bit=6)
            # Interrupt when a single or double tap is detected.
            self.wakeupOut = bitType(register_address=address, bit=5)
            # Interrupt on specified orientation (e.g. face up).
            self.wristTiltOut = bitType(register_address=address, bit=3)
            # Interrupt on specified activity (e.g. walking).
            self.activityTypeOut = bitType(register_address=address, bit=2)
            # Interrupt after every x steps, see the interruptOnSteps property.
            self.stepCounterOut = bitType(register_address=address, bit=1)
            # Interrupt when the FIFO is full.
            self.ffullInt = bitType(register_address=extAddress, bit=ffull)
            # Interrupt when the FIFO is at the watermark level.
            self.fwmInt = bitType(register_address=extAddress, bit=fwm)
            if aux:
                # Interrupt when the auxiliary sensor has data.
                self.auxInt = bitType(register_address=extAddress, bit=aux)
            if acc:
                # Interrupt when the accelerometer has data.
                self.accInt = bitType(register_address=extAddress, bit=acc)
            if drdy:
                # General data ready interrupt.
                self.drdyInt = bitType(register_address=extAddress, bit=drdy)
                
            if ctrlAddress is not None:
                # Interrupt pin set to edge triggered if set.
                # Otherwise interrupt pin is level triggered.
                self.edgeCtrl = RWBit(register_address=ctrlAddress, bit=0)
                # Interrupt pin is active high if set, otherwise active low.
                self.lvl = RWBit(register_address=ctrlAddress, bit=1)
                # Interrupt pin is open drain if set, otherwise push-pull.
                self.od = RWBit(register_address=ctrlAddress, bit=2)
                # Interrupt output enable pin.
                self.outputEn = RWBit(register_address=ctrlAddress, bit=3)
                # Interrupt input enable pin.
                self.inputEn = RWBit(register_address=ctrlAddress, bit=4)

            

##
# @brief A class for monitoring the voltage of the single cell LiPo battery.
class Battery:
    # The Watchy has a 100k/100k voltage divider on the battery sense pin.
    VOLTAGE_DIVIDER = 0.5
    # Take many samples and average them to get a more stable reading.
    SAMPLES = 10

    def __init__(self, sensePin) -> None:
        self.sensePin = analogio.AnalogIn(sensePin)
        
    ##
    # @brief Gets the battery voltage.
    @property
    def voltage(self) -> float:
        total = 0.0
        for n in range(self.SAMPLES):
            total += self.sensePin.value
        adc =  total / self.SAMPLES
        return adc * self.sensePin.reference_voltage / 65535 / self.VOLTAGE_DIVIDER
               
    ##
    # @brief Gets the battery percentage.
    # Uses a formula from https://electronics.stackexchange.com/a/551667
    # @return The battery percentage as a float.
    @property
    def percentage(self) -> float:
        # Don't ask me how this works...
        # Someone modelled LiPo discharge curves and came up with this formula.
        v = self.voltage
        v = math.pow(v / 3.7, 80)
        perc = 123 - (123 / math.pow(1 + v, 0.165))
        return min(100, max(0, perc))
    
##
# @brief Interface to the real time clock IC.
# The RTC is either a PCF8563 or DS3231 depending on device version.
class RTC:
    # https://docs.circuitpython.org/projects/pcf8563/en/latest/index.html
    
    ##
    # @brief Creates a new RTC object on the given I2C bus.
    # @param i2c The I2C bus to use. Can be initialised from board.I2C()
    def __init__(self, cfg) -> None:
        self.cfg = cfg
        i2c = self.cfg['i2c']
        self.cfg['rtc'] = self
        # Just use the PCF8563 library for now, will update this later.
        self.dev = adafruit_pcf8563.pcf8563.PCF8563(i2c)
        # Tells the firmware to use this device to get the current time.
        # This replaces the default time source which is the RTC inside the ESP32.
        rtc.set_time_source(self.dev)
        
    # Long and short string constants so named days and months can be returned.
    WEEKDAYS = ['Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday', 'Sunday']
    WEEKDAYS_SHORT = ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun']
    MONTHS = ['January', 'February', 'March', 'April', 'May', 'June', 'July', 'August', 'September', 'October', 'November', 'December']
    MONTHS_SHORT = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']
    
    ##
    # @brief Returns the current hour between zero and 23.
    @property
    def hour(self) -> int:
        return self.dev.datetime.tm_hour
    
    ##
    # @brief Returns the current minute between zero and 59.
    @property
    def minute(self) -> int:
        return self.dev.datetime.tm_min
    
    ##
    # @brief Returns the current second between zero and 59.
    @property
    def second(self) -> int:
        return self.dev.datetime.tm_sec
    
    ##
    # @brief Returns the current day of the month between one and 31.
    @property
    def day(self) -> int:
        return self.dev.datetime.tm_mday
    
    ##
    # @brief Returns the full name of the current weekday.
    @property
    def fullWeekday(self) -> str:
        return self.WEEKDAYS[self.dev.datetime.tm_wday]
    
    ##
    # @brief Returns the abbreviated name of the current weekday.
    @property
    def weekday(self) -> str:
        return self.WEEKDAYS_SHORT[self.dev.datetime.tm_wday]
    
    ##
    # @brief Returns the number of the current weekday between one and seven.
    @property
    def weekdayNumber(self) -> int:
        return self.dev.datetime.tm_wday + 1
    
    ##
    # @brief Returns the number of the current month between one and 12.
    @property
    def monthNumber(self) -> int:
        return self.dev.datetime.tm_mon
    
    ##
    # @brief Returns the abbreviated name of the current month.
    @property
    def month(self) -> str:
        return self.MONTHS_SHORT[self.dev.datetime.tm_mon - 1]
    
    ##
    # @brief Returns the full name of the current month.
    @property
    def fullMonth(self) -> str:
        return self.MONTHS[self.dev.datetime.tm_mon - 1]
    
    ##
    # @brief Return the current year are a two digit number.
    @property
    def year(self) -> int:
        return self.dev.datetime.tm_year % 100
    
    ##
    # @brief Returns the current year as a four digit number.
    @property
    def fullYear(self) -> int:
        return self.dev.datetime.tm_year
    
    ##
    # @brief Returns the current century number.
    @property
    def century(self) -> int:
        return (self.dev.datetime.tm_year // 100) + 1
    
    ##
    # @brief Returns the current date and time as a time struct.
    @property
    def time(self) -> time.struct_time:
        return self.dev.datetime
    
    @property
    def use24hClock(self) -> bool:
        return self.cfg['timeFormat'] == '24h'
    
    @use24hClock.setter
    def use24hClock(self, value: bool) -> None:
        if value:
            self.cfg['timeFormat'] = '24h'
        else:
            self.cfg['timeFormat'] = '12h'

    ##
    # @brief Returns the current time as a string in the format specified by the
    # timeFormat property. See: the use24hClock property.
    @property
    def clockTime(self) -> str:
        if self.cfg['timeFormat'] == '12h':
            return self.clockTime12h
        else:
            return self.clockTime24h
        
    @property
    def dateFormat(self) -> bool:
        return self.cfg['dateFormat']
    
    @dateFormat.setter
    def dateFormat(self, value: str):
        value = value.lower()
        if value == 'iso':
            value = 'ymd'
        if value not in ['ymd', 'dmy', 'mdy']:
            raise ValueError('Invalid date format')
        self.cfg['dateFormat'] = value

    ##
    # @brief Returns the current minute and hours as a string in HH:MM format.
    # A 24-hour clock is used.
    # An hour or minute less than 10 is padded with a zero.
    @property
    def clockTime24h(self) -> str:
        return f'{self.hour:02}:{self.minute:02}'

    ##
    # @brief Returns the current minute and hours as a string in HH:MM format.
    # This is the same as clockTime but with a 12-hour clock.
    # Either ' AM' or ' PM' is appended to the end.
    # A minute letter than 10 is padded with a zero.
    # An hour less than 10 is padded with a space.
    @property
    def clockTime12h(self) -> str:
        hour = self.hour
        if hour > 12:
            hour -= 12
            suffix = 'PM'
        else:
            suffix = 'AM'
        return f'{hour:2}:{self.minute:02} {suffix}'
    
    ##
    # @brief Returns the current date as a string.
    # The format is determined by the dateFormat property.
    # Possible options are:
    # - GeoLocation.YMD_FORMAT (year-month-day)
    # - GeoLocation.DMY_FORMAT (day/month/year)
    # - GeoLocation.MDY_FORMAT (month/day/year)
    # The default is year-month-day.
    # The GeoLocation class may change this later depending on the country.
    @property
    def date(self) -> str:
        format = self.cfg['dateFormat']
        if format == 'mdy':
            return f'{self.monthNumber:02}/{self.day:02}/{self.year:02}'
        elif format == 'dmy':
            return f'{self.day:02}/{self.monthNumber:02}/{self.year:02}'
        else:
            return f'{self.year:02}-{self.monthNumber:02}-{self.day:02}'
        
    ##
    # @brief A setter for the time property.
    # The time can be set with a time struct.
    # This should be done automatically by the NTP class if internet is available.
    @time.setter
    def time(self, time) -> None:
        self.dev.datetime = time
    
    ##
    # @brief Returns a time struct rounds up to the next minute.
    # This is useful for display refresh.
    @property
    def timeNextMinute(self) -> time.struct_time:
        current = time.mktime(self.time)
        nextMin = current // 60 * 60 + 60
        return time.localtime(nextMin)
        
##
# @brief A class for syncing the RTC with an NTP server.
# This sync should be done every day if possible, depending on the RTC accuracy.
# A DS3231 may only need once a week.
class NTP:
    # https://docs.circuitpython.org/projects/ntp/en/latest/
    
    ##
    # @brief The constructor for the NTP class.
    def __init__(self, cfg) -> None:
        self.cfg = cfg
        self.pool = cfg['pool']
        self.geo = cfg['geo']
        self.rtc = cfg['rtc']
        self.ntp = adafruit_ntp.NTP(self.pool, tz_offset=cfg['utcOffset'])
        self.ntp._tz_offset = int(cfg['utcOffset'] * 60 * 60)
        self.sync()
        
    ##
    # @brief Returns the current time from the NTP server as a time struct.
    # This time is already formatted for the local time zone.
    # The NTP library may cache this value to prevent unnecessary calls.
    @property
    def time(self) -> time.struct_time:
        return self.ntp.datetime
    
    ##
    # @brief Sets the RTC to the current time from an NTP server.
    def sync(self) -> None:
        self.rtc.time = self.ntp.datetime
            
##
# @brief A class for getting the current location from an IP address.
# This is done by querying an IP geolocation API.
# The API used is ip-api.com.
# This is a great free service that can be used (non-commerically) without an API key.
# The resulting data is used for time, weather, and date formatting.
class GeoLocation:
    # https://ip-api.com/docs/api:json

    # Lists of countries by the most common date format.
    # Retrieved from Wikipedia, I'm not sure how accurate this is.
    DMY_COUNTRIES = ['Italy', 'Ukraine', 'Romania', 'Netherlands', 'Mexico', 'Guatemala', 'Honduras', 'Brazil', 'Colombia', 'Chile', 'Argentina', 'Peru', 'Venezuela', 'Egypt', 'Algeria', 'Morocco', 'Tunisia', 'Somalia', 'Nigeria', 'Ethiopia', 'DRC', 'Tanzania', 'Sudan', 'Uganda', 'Turkey', 'Iraq', 'Saudi Arabia', 'Yemen', 'Tajikistan', 'Kyrgyzstan', 'Turkmenistan', 'Malaysia', 'Indonesia', 'Thailand', 'Cambodia', 'Pakistan', 'Bangladesh', 'Papua New Guinea', 'New Zealand', 'Philippines', 'Togo', 'Panama', 'Puerto Rico', 'Cayman Islands', 'Greenland', 'India', 'Russia', 'Vietnam', 'Germany', 'Iran', 'United Kingdom', 'France', 'Myanmar', 'Spain', 'Poland', 'Uzbekistan', 'Afghanistan', 'Nepal', 'Australia', 'Cameroon', 'Sri Lanka']
    YMD_COUNTRIES = ['China', 'Japan', 'South Korea', 'North Korea', 'Taiwan', 'Hungary', 'Mongolia', 'Lithuania', 'Bhutan']
    MDY_COUNTRIES = ['United States', 'South Africa', 'Kenya', 'Canada', 'Ghana'] 
    IMPERIAL_COUNTRIES = ['United States', 'Liberia', 'Myanmar']

    # The fields to request from the API.
    # - offset: The UTC offset in seconds.
    # - country: The full name of the country detected.
    # - city: The full name of the city detected.
    # - lat: The Geo-IP latitude.
    # - lon: The Geo-IP longitude.
    # - countryCode: The two letter ISO 3166-1 country code.
    fields = ['offset', 'country', 'city', 'lat', 'lon', 'countryCode']
    data = {}
    def __init__(self, cfg) -> None:
        self.cfg = cfg
        self.pool = cfg['pool']
        self.cfg['geo'] = self
        self.session = adafruit_requests.Session(self.pool)
        try:
            self.geolocate()
        except ConnectionError:
            self.loadSaved()

    ##
    # @brief Attempts to refresh the GeoLocation data.
    # The geolocation refresh may fail if the internet is not available.
    # Refreshing may be needed if the user travels to a new location.
    # e.g. If the user travels to a different town or suburb,
    # the temperature and weather may be different.
    def refresh(self):
        try:
            self.geolocate()
        except ConnectionError:
            pass
        
    ##
    # @brief Loads the GeoLocation data.
    # The geolocation data is loaded from the internet or from a saved file if 
    # the internet is not available.
    def geolocate(self):
        urlFields = ','.join(self.fields)
        result = self.session.get(f'http://ip-api.com/json/?fields={urlFields}')
        if result.status_code != 200:
            raise ConnectionError('Failed to get geolocation data')
        self.data = result.json()
        with open('geolocation.json', 'w') as f:
            f.write(json.dumps(self.data))
        self.loadProps()

    ##
    # @brief Loads the GeoLocation properties from the data.
    # This is called after the data is loaded from the internet or from a file.
    # This may update the time or date format.
    def loadProps(self) -> None:
        self.cfg['lat']       = self.lat = self.data['lat']
        self.cfg['lon']       = self.lon = self.data['lon']
        self.cfg['utcOffset'] = self.utcOffset = self.data['offset'] / 60 / 60
        self.cfg['country']   = self.country = self.data['country']
        self.cfg['city']      = self.city = self.data['city']
        self.cfg['iso3166']   = self.countryCode = self.data['countryCode']
        self.cfg['metric']    = self.country not in self.IMPERIAL_COUNTRIES
        if self.cfg['metric']:
            self.cfg['tempUnit'] = 'C'
        else:
            self.cfg['tempUnit'] = 'F'
        if self.country in self.DMY_COUNTRIES:
            self.cfg['dateFormat'] = 'dmy'
        elif self.country in self.YMD_COUNTRIES:
            self.cfg['dateFormat'] = 'ymd'
        elif self.country in self.MDY_COUNTRIES:
            self.cfg['dateFormat'] = 'mdy'
        else:
            self.cfg['dateFormat'] = 'ymd'
        self.cfg['timeFormat'] = '24h'
    ##
    # @brief Loads the GeoLocation data from a previously saved file.
    def loadSaved(self):
        try:
            with open('geolocation.json', 'r') as f:
                self.data = json.load(f)
                self.loadProps()
        except FileNotFoundError:
            return None

##
# TODO: Added a shared class for user settings.
class Settings:
    cfg = {}
    def __init__(self, filename) -> None:
        self.filename = filename
        self.load()

    def load(self):
        try:
            with open(self.filename, 'r') as f:
                self.cfg = json.load(f)
        except FileNotFoundError:
            pass

    def __getitem__(self, key):
        if type(key) is not str:
            raise TypeError('Settings key must be a string')
        if key not in self.cfg:
            raise KeyError(f'Settings key {key} not found')
        return self.cfg[key]
    
    def __setitem__(self, key, value):
        if type(key) is not str:
            raise TypeError('Settings key must be a string')
        self.cfg[key] = value
    
    
##
# @brief Power management class.
# Not yet implemented.
class Power:
    def __init__(self, rtc, buttons) -> None:
        self.rtc = rtc
        self.buttons = buttons
        self.alarmPins = [button.pin for button in self.buttons.buttons]
        #self.alarmPins += 

    def sleep(self):
        self.wakeTime = self.rtc.timeNextMinute
        self.ioAlarms = []
        for button in self.buttons.buttons:
            button.button.deinit()
            self.ioAlarms.append(button.ioAlarm)
        
def firstBoot():
    try:
        if __file__ == 'code.py':
            with open('main.py', 'w') as f:
                f.write('import watchylib\nwatchylib.firstBoot()')
    except NameError:
        pass
    
    defaultMain = """
import watchylib
watchy = watchylib.Watchy()
t = watchy.time
disp = watchy.display
clock = disp.text(f'88:88', scale=4, align=disp.CENTRE, bg=True')
while True:
    clock.text = f'{t.tm_hour}:{t.tm_min:02}'
    disp.update()
    watchy.sleep()"""
    
    defaultSafe = """
import os
import sys
import board
import digitalio
import time
import alarm
import supervisor
import microcontroller
menu = digitalio.DigitalInOut(board.BTN1)
back = digitalio.DigitalInOut(board.BTN2)
up = digitalio.DigitalInOut(board.BTN3)
down = digitalio.DigitalInOut(board.BTN4)

def refresh():
    time.sleep(board.DISPLAY.time_to_refresh)
    while True:
        try:
            board.DISPLAY.refresh()
        except RuntimeError:
            time.sleep(0.1)
        else:
            break

while True:
    print(f\"\"\"\x1b[2J<RESTART          EXIT TO TERM>
===========SAFE=MODE===========
You are in safe mode due to a
code error or power loss.
Press BACK to restart.
Press MENU to reset to factory.
Press UP for a serial terminal.
Press DOWN to shut down.
The web interface may still
be available.
Reason: {supervisor.runtime.safe_mode_reason}
<FACTORY RESET       POWER DOWN>\"\"\")
    refresh()
    while True:
        if back.value == True:
            selection = back
            problem = 'Your code will restart.'
            break
        elif up.value == True:
            selection = up
            problem = 'You need to connect via serial.'
            break
        elif down.value == True:
            selection = down
            problem = 'The board will be unresponsive.'
            break
        elif menu.value == True:
            selection = menu
            problem = 'Your code will be deleted.'
            break
    
    print('\x1b[2J<CANCEL            CONFIRM>')
    print(problem)
    print('Are you sure?')
    refresh()
    while back.value == up.value:
        time.sleep(0.01)
    if up.value:
        break
print('Running selection...')
print('Run microcontroller.reset() to exit safe mode.')
refresh()
if selection == back:
    microcontroller.reset()
elif selection == up:
    sys.exit(0)
elif selection == down:
    while True:
        alarm.exit_and_deep_sleep_until_alarms()
elif selection == menu:
    delFiles = ['main.py', 'settings.json', 'geolocation.json', 'settings.toml', 
    'boot.py', 'code.py', 'safemode.py', 'main.py', 'main.txt', 'code.txt']
    for file in os.listdir('/'):
        if file in delFiles:
            try:
                os.remove(file)
            except OSError:
                pass
microcontroller.reset()
"""

    defaultCrash = """
import os
import sys
import board
import digitalio
import time
import alarm
import supervisor
import wifi
import microcontroller

def refresh():
    time.sleep(board.DISPLAY.time_to_refresh)
    while True:
        try:
            board.DISPLAY.refresh()
        except RuntimeError:
            time.sleep(0.1)
        else:
            break

supervisor.set_next_code_file(None)
if not wifi.radio.connected:
    print('WiFi not connected.\\nAttempting to reconnect...')
    refresh()
    ssid = os.getenv('CIRCUITPY_WIFI_SSID')
    wifiPass = os.getenv('CIRCUITPY_WIFI_PASSWORD')
    if ssid and wifiPass:
        if not wifi.radio.enabled:
            wifi.radio.enabled = True
        try:
            wifi.radio.connect(ssid, wifiPass)
        except ConnectionError:
            pass
        time.sleep(5)

menu = digitalio.DigitalInOut(board.BTN1)
back = digitalio.DigitalInOut(board.BTN2)
up = digitalio.DigitalInOut(board.BTN3)
down = digitalio.DigitalInOut(board.BTN4)

traceback = supervisor.get_previous_traceback()
if not traceback:
    traceback = 'Code Finished.'
while True:
    print(f'\x1b[2J\x1b[5;0H{traceback}')
    print('\x1b[3;0H<RESTART          EXIT TO TERM>')
    print('===========CRASH=REPORT==========')
    print('\x1b[18;0H<SAFE MODE            POWER DOWN>')
    refresh()
          
    while up.value == down.value == back.value == menu.value == False:
        time.sleep(0.1)
    print('\x1b[2JRunning selection...')
    refresh()
    if up.value:
        sys.exit(0)
        break
    elif down.value:
        while True:
            alarm.exit_and_deep_sleep_until_alarms()
    elif back.value:
        microcontroller.reset()
    elif menu.value:
        microcontroller.on_next_reset(microcontroller.RunMode.SAFE_MODE)
        microcontroller.reset()
"""

    defaultBoot = """
import supervisor
supervisor.set_next_code_file(filename='main.py',
    reload_on_error=True, reload_on_success=True)
supervisor.reload()
"""
    CLEAR_SCREEN = '\x1b[2J'
    disp = board.DISPLAY
    confirm = digitalio.DigitalInOut(board.BTN1)
    cancel = digitalio.DigitalInOut(board.BTN2)
    up = digitalio.DigitalInOut(board.BTN3)
    down = digitalio.DigitalInOut(board.BTN4)
    white = displayio.Palette(1)
    white[0] = 0x000000
    rootdir = os.listdir('/')

    print('Running first time setup...')
    time.sleep(disp.time_to_refresh)
    disp.refresh()
    while confirm.value == True:
        time.sleep(0.01)

    if 'settings.json' not in rootdir:
        with open('settings.json', 'w') as f:
            f.write(json.dumps({}))
        
    if 'main.py' not in rootdir:
        with open('main.py', 'w') as f:
            f.write(defaultMain)
                         
    if 'safemode.py' not in rootdir:
        with open('safemode.py', 'w') as f:
            f.write(defaultSafe)
                         
    if 'crash.py' not in rootdir:
        with open('crash.py', 'w') as f:
            f.write(defaultCrash)
    
    if 'boot.py' not in rootdir:
        with open('boot.py', 'w') as f:
            f.write(defaultBoot)
            
    setupReason = None
    if 'settings.toml' not in rootdir:
        setupReason = 'File settings.toml not found!'
    elif not wifi.radio.connected:
        time.sleep(5) # Maybe the network is just slow to connect?
        if not wifi.radio.connected:
            setupReason = 'No WiFi connection!'
                         
    if setupReason:
        print(CLEAR_SCREEN + '<CANCEL                       UP>')
        print(setupReason)
        print("""You will not be able to connect
via the web interface.
Press CANCEL to continue or MENU
to run the interactive setup.
These are the top left and bottom
left buttons respectively.
You will need to connect to a COM
port at 115200 baud for the 
interactive setup.


<MENU                       DOWN>""")
        time.sleep(disp.time_to_refresh)
        disp.refresh()
        
        while confirm.value == cancel.value:
            time.sleep(0.01)

        if confirm.value:
            print(CLEAR_SCREEN + 'Running interactive setup...')
            print('Open a serial terminal at 115200')
            print(' baud and press ENTER')
            time.sleep(disp.time_to_refresh)
            disp.refresh()
            
            input()
            ssid = ''
            wifiPass = ''
            while not wifi.radio.connected:
                while ssid.strip() == '':
                    ssid = input('WiFi SSD> ')
                while wifiPass.strip() == '':
                    wifiPass = input('WiFi password> ')
                print('Attempting connection.', end='')
                wifi.radio.connect(ssid, wifiPass)
                waitTime = 10
                while(not wifi.radio.connected and waitTime > 0):
                    print('.', end='')
                    time.sleep(1)
                    waitTime -= 1
                if not wifi.radio.connected:
                    print('Failed')
                    ssid = ''
                    wifiPass = ''
                
            print('\nWiFi Connected!')
            print('You will need a password for')
            print('CircuitPython web login.')
            print('No username is required.')
            loginPass = ''
            while loginPass.strip() == '':
                input('Password> ')
            
            print('Saving settings...', end='')
            try:
                with open('settings.toml', 'r') as f:
                    settings = f.readlines()
            except OSError:
                settings = []
                    
            with open('settings.toml', 'w') as f:
                if ssid:
                    f.write(f'CIRCUITPYTHON_WIFI_SSID={ssid}\n')
                if wifiPass:
                    f.write(f'CIRCUITPYTHON_WIFI_PASSWORD={wifiPass}\n')
                if loginPass:
                    f.write(f'CIRCUITPYTHON_WEB_API_PASSWORD={loginPass}\n')
                f.write('\n'.join(settings))
            print('Done')
    try:
        if __file__ == 'code.py':
            os.remove('code.py')
    except NameError:
        pass
    microcontroller.reset()
    
    

