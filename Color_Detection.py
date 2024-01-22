import serial

# Configure serial port
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Update the port and baud rate as needed

try:
    while True:
        # Get a number from the user
        user_input = input("Enter a number to send to Arduino: ")

        # Check if the input is a valid integer
        try:
            number_to_send = int(user_input)
        except ValueError:
            print("Invalid input. Please enter a valid integer.")
            continue

        # Send the number to Arduino
        ser.write(str(number_to_send).encode())

except KeyboardInterrupt:
    print("\nExiting the program.")

finally:
    ser.close()  # Close the serial port on program exit
