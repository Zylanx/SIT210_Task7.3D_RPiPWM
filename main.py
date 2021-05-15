import time
import atexit

import RPi.GPIO as gpio


# Define the pin constants
BUZZER_PIN = 18
DISTANCE_ECHO = 14
DISTANCE_TRIGGER = 15


# The duty cycle is set to 25% to make it a bit quieter
DUTY_CYCLE = 25.0


# Setup RPi.GPIO to use gpio numbering
gpio.setmode(gpio.BCM)


# Class to represent distance sensors (In this case the HC-SR04)
class DistanceSensor:
    # The speed of sound. Used to calculate the distance from the time taken
    SPEED_OF_SOUND = 343.26

    # Initialise the sensor and its pins
    def __init__(self, trigger_pin, echo_pin):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin

        gpio.setup(trigger_pin, gpio.OUT)
        gpio.setup(echo_pin, gpio.IN)

        gpio.output(trigger_pin, gpio.LOW)

    # Returns the distance of objects in metres from the sensor.
    def get_distance(self):
        # Create a list to store multiple readings so we can get an average
        buffer = []

        # get three readings
        for _ in range(3):
            # hold the trigger high for 10 microseconds
            gpio.output(self.trigger_pin, gpio.HIGH)
            time.sleep(0.00001)
            gpio.output(self.trigger_pin, gpio.LOW)

            # wait until the echo pin starts outputting its distance record the start time of the echo high
            while gpio.input(self.echo_pin) == gpio.LOW:
                pass
            start_time = time.time()

            # Wait until the echo pin finished outputting high and record the time
            while gpio.input(self.echo_pin) == gpio.HIGH:
                pass
            end_time = time.time()

            # add the distance to the buffer
            buffer.append((end_time - start_time) * self.SPEED_OF_SOUND / 2.0)

        # return the average distance
        return sum(buffer) / len(buffer)


# Map the input in the input range to the output range
# Code from https://stackoverflow.com/questions/929103/convert-a-number-range-to-another-range-maintaining-ratio
# with edits to make it not horrible by me
def map(x, input_min, input_max, output_min, output_max):
    # range check
    if input_min == input_max:
        return None
    if output_min == output_max:
        return None

    if x is None:
        return None

    if x < input_min or x > input_max:
        return None

    # check reversed input range
    reverse_input = False
    old_min = min(input_min, input_max)
    old_max = max(input_min, input_max)
    if not old_min == input_min:
        reverse_input = True

    # check reversed output range
    reverse_output = False
    new_min = min(output_min, output_max)
    new_max = max(output_min, output_max)
    if not new_min == output_min:
        reverse_output = True

    portion = (x - old_min) * (new_max - new_min) / (old_max - old_min)
    if reverse_input:
        portion = (old_max - x) * (new_max - new_min) / (old_max - old_min)

    result = portion + new_min
    if reverse_output:
        result = new_max - portion

    return result


# The main method to be executed
def main():
    # Setup the buzzer pin and the sensor
    gpio.setup(BUZZER_PIN, gpio.OUT)
    distance_sensor = DistanceSensor(trigger_pin=DISTANCE_TRIGGER, echo_pin=DISTANCE_ECHO)

    # Setup the buzzer to output PWM with the given duty cycle
    buzzer = gpio.PWM(BUZZER_PIN, DUTY_CYCLE)

    # The current status of the buzzer
    buzzer_started = False

    while True:
        # Get the distance and convert it to centimetres
        dist = int(distance_sensor.get_distance() * 100)
        print(dist)
        # Map the distance to frequencies
        dist = map(dist, 0, 500, 5000, 100)

        # If the distance is none, then stop outputting
        if dist is None:
            if buzzer_started:
                buzzer.stop()
                buzzer_started = False
        # The buzzer should play the new frequency
        else:
            if buzzer_started:
                # If the buzzer is already playing, stop it then change the frequency and start it again
                buzzer.stop()
                buzzer.ChangeFrequency(dist)
                buzzer.start(DUTY_CYCLE)
            else:
                # The buzzer isn't already playing so change the frequency and start playing again
                buzzer.ChangeFrequency(dist)
                buzzer.start(DUTY_CYCLE)
                buzzer_started = True

        time.sleep(0.25)


if __name__ == "__main__":
    # Start the main function, cleaning up if it exits for any reason
    try:
        main()
    except:
        pass
    finally:
        gpio.cleanup()