import time
import machine, onewire, ds18x20
from rotary_irq_rp2 import RotaryIRQ
from ssd1306 import SSD1306_I2C
from machine import Pin, I2C, PWM

# Initialize the mode selection button
button = machine.Pin(12, machine.Pin.IN, machine.Pin.PULL_DOWN)

# OLED display setup
WIDTH = 128
HEIGHT = 64
i2c = I2C(1, scl=Pin(15), sda=Pin(14))
oled = SSD1306_I2C(WIDTH, HEIGHT, i2c)

# Rotary encoder setup (used to adjust temperature settings)
SW = Pin(13, Pin.IN, Pin.PULL_UP)
r = RotaryIRQ(pin_num_dt=16,
              pin_num_clk=17,
              min_val=10,
              max_val=25,
              reverse=False,
              range_mode=RotaryIRQ.RANGE_BOUNDED)
val_old = r.value()  # Initialize previous rotary value

# PID control parameters
kp = 0.8  # Proportional gain
ki = 0.2  # Integral gain
kd = 0.001  # Derivative gain
time_previous = 0

# DS18B20 temperature sensor setup
ds_pin = machine.Pin(22)
ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))
roms = ds_sensor.scan()  # Find sensors
a = roms[0]  # Assume first sensor found is the one to use

# PID control variables
integral = 0
previous = 0
output = 0
setpoint = 10

# PWM setup for control output (e.g., heating element)
out = machine.Pin(20)
pel = PWM(out)
frequency = 50  # Frequency for PWM signal
pel.freq(frequency)

# Initial state variables
x = 0
y = 0

# Display initial set temperature
print("Set Temp.:  10")
oled.fill(0)
oled.text("Set Temp.:  10", 0, 0)
oled.show()

# Main loop
while True:
    try:
        n = 0  # Initialize button press tracking variable
        x = 0  # Track manual/automatic mode

        # Check the state of mode selection button
        if button.value() == 0 and x == 0:  # it state (0) enter manual mode
            x = 1  # Enter manual mode
            while x == 1:
                # Rotary encoder adjustment
                val_new = r.value()
                
                if val_old != val_new:  # Update if new value from rotary encoder
                    val_old = val_new
                    print("Set Temp.: ", val_new)

                # Display manual mode and set temperature
                oled.fill(0)
                oled.text("Manual Mode", 30, 0)
                oled.text("Set Temp.: ", 0, 10)
                oled.text(str(val_new), 80, 10)
                oled.show()

                # Exit manual mode if state of mode selection button is changed
                if button.value() != 0 and x == 1:
                    oled.fill(0)
                    oled.text("Switching to", 0, 0)
                    oled.text("Presaved values", 0, 10)
                    oled.show()
                    print("exiting loop of cond 1")
                    time.sleep(0.2)
                    break

                # Save set temperature when rotary encoder button  is pressed
                if SW.value() == 0 and n == 0:
                    print("Button Pressed")
                    oled.fill(0)
                    oled.text("Saving Value . . .", 0, 0)
                    oled.show()
                    time.sleep(0.5)
                    set_temp = val_new
                    n = 1

                    while SW.value() == 0:  # Wait for release
                        continue

                    while n == 1:
                        # PID control loop
                        time.sleep(0.1)
                        error = 0
                        integral = 0

                        # Update temperature reading
                        ds_sensor.convert_temp()
                        time_now = time.ticks_ms()
                        dt = (time_now - time_previous) / 1000.00
                        time_previous = time_now
                        current_temp = ds_sensor.read_temp(a)

                        # Calculate PID terms
                        error = set_temp - current_temp
                        proportional = error
                        integral += error * dt
                        derivative = (error - previous) / dt
                        previous = error

                        # Compute PID output
                        output = (kp * proportional) + (ki * integral) + (kd * derivative)
                        pwm_output = (-output) * 68.2 * 64  # Scale output for PWM

                        # Apply PWM output
                        pel.duty_u16(int(pwm_output))
                        time.sleep(0.005)

                        # Display process
                        print(pwm_output, output, current_temp)
                        oled.fill(0)
                        oled.text("Work in Progress !", 0, 0)
                        oled.text("Temp. Set: ", 0, 10)
                        oled.text(str(set_temp), 80, 10)
                        oled.text("Current Temp.: ", 0, 20)
                        oled.text(str(current_temp), 40, 30)
                        oled.text("*C", 95, 30)
                        oled.show()
                        time.sleep(0.1)

                        # Exit control loop if encoder button is pressed again
                        if SW.value() == 0 and n == 1:
                            print("button pressed again")
                            oled.fill(0)
                            oled.text("Entering Edit Mode", 0, 0)
                            time.sleep(0.5)
                            break
                    n = 0
                    oled.fill(0)
                    oled.show()
                    x = 0  # Exit manual mode
                
                time.sleep(0.1)

        else:
            # pre-saved values mode
            x = 1
            while x == 1:
                val_new = r.value()
                
                # If rotary encoder changes, update selection
                if val_old != val_new:
                    val_old = val_new
                    print("Condn 2", val_new, button.value(), val_old)
                
                # Display preset selection menu
                oled.fill(0)
                oled.text("Select Value", 0, 0)
                oled.text("10  15  20  25", 0, 10)
                
                y = (val_new - 2) % 4
                if y == 0:
                    set_temp = 10
                if y == 1:
                    set_temp = 15
                if y == 2:
                    set_temp = 20
                if y == 3:
                    set_temp = 25
                
                for i in range(1, 15, 1):
                    oled.pixel(y * 32 + i, 21, 1)
                
                oled.show()

                # check if state of mode selection button is changed 
                if button.value() == 0 and x == 1:
                    oled.fill(0)
                    oled.text("Switching to", 0, 0)
                    oled.text("Manual mode", 0, 10)
                    time.sleep(0.2)
                    oled.show()
                    break

                # Save selected temperature on encoder button press
                if SW.value() == 0 and n == 0:
                    print("Button Pressed")
                    oled.fill(0)
                    oled.text("Saving Value . . .", 0, 0)
                    oled.show()
                    time.sleep(0.5)
                    n = 1
                    while SW.value() == 0:
                        continue

                    while n == 1:
                        # PID control
                        time.sleep(0.1)
                        error = 0
                        integral = 0
                        ds_sensor.convert_temp()
                        time_now = time.ticks_ms()
                        dt = (time_now - time_previous) / 1000.00
                        time_previous = time_now
                        current_temp = ds_sensor.read_temp(a)

                        # PID calculations
                        error = set_temp - current_temp
                        proportional = error
                        integral += error * dt
                        derivative = (error - previous) / dt
                        previous = error

                        output = (kp * proportional) + (ki * integral) + (kd * derivative)
                        pwm_output = (-output) * 68.2 * 64
                        pel.duty_u16(int(pwm_output))
                        time.sleep(0.005)

                        # Display process
                        print(pwm_output, output, current_temp)
                        oled.fill(0)
                        oled.text("Work in Progress !", 0, 0)
                        oled.text("Temp. Set: ", 0, 10)
                        oled.text(str(set_temp), 80, 10)
                        oled.text("Current Temp.: ", 0, 20)
                        oled.text(str(current_temp), 40, 30)
                        oled.text("*C", 95, 30)
                        oled.show()
                        time.sleep(0.1)

                        # Exit PID control loop if encoder button is pressed again
                        if SW.value() == 0 and n == 1:
                            print("button pressed again")
                            oled.fill(0)
                            oled.text("Entering Edit Mode", 0, 0)
                            time.sleep(0.5)
                            break
                    n = 0
                    oled.fill(0)
                    oled.show()
                    x = 0
                
                time.sleep(0.1)
            
        time.sleep(0.1)

    except KeyboardInterrupt:
        break  # Exit the loop on keyboard interrupt
