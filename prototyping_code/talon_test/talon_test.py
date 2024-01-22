#!/usr/bin/python3

import pigpio

left_percent = 0
right_percent = 0

pi = pigpio.pi()

def main():
    # Ensure pigpio daemon has started
    if not pi.connected:
        exit()
    else:
        print('pigpio daemon started')

    # Motor Controller attached to GPIO 18 (pin 12).  Send the stop motors
    # command.
    pi.set_servo_pulsewidth(18, 1500)
    pi.set_servo_pulsewidth(13, 1500)

    while True:
        try:
            # Get the next command
            next_cmd_str = input('Please enter next motor command: ')

            # Split the string into the left and right commands
            if ' ' in next_cmd_str:
                space_idx = next_cmd_str.index(' ')

                left_cmd_str = next_cmd_str[:space_idx]
                right_cmd_str = next_cmd_str[space_idx+1:]

                try:
                    left_cmd_int = int(left_cmd_str)
                    right_cmd_int = int(right_cmd_str)

                    # Ensure values are within range
                    left_cmd_int = max(-100, left_cmd_int)
                    left_cmd_int = min(100, left_cmd_int)

                    right_cmd_int = max(-100, right_cmd_int)
                    right_cmd_int = min(100, right_cmd_int)

                    # Motor values have to be between 1000 - 2000 microseconds

                    if left_cmd_int == 0:
                        print('Stopping left motor')
                        left_percent = 0
                    else:
                        left_percent = left_cmd_int
                        print('Setting left percent to: ' + str(left_percent) +
                                '%')
                    if right_cmd_int == 0:
                        print('Stopping right motor')
                        right_percent = 0
                    else:
                        right_percent = right_cmd_int
                        print('Setting right percent to: ' + str(right_percent)
                                + '%')

                    left_cmd = left_percent * 5 + 1500
                    pi.set_servo_pulsewidth(18, left_cmd)

                    right_cmd = right_percent * 5 + 1500
                    pi.set_servo_pulsewidth(13, right_cmd)
                except ValueError:
                    print('ERROR: Invalid motor command')
        except KeyboardInterrupt:
            break

    print('\nStopping program')
    pi.set_servo_pulsewidth(18, 1500)
    pi.set_servo_pulsewidth(13, 1500)
    pi.stop()

if __name__ == '__main__':
    main()
