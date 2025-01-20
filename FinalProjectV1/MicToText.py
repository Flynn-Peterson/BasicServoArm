import serial
import speech_recognition as sr
import time

# Initialize speech recognizer
recognizer = sr.Recognizer()

# Serial port setup
try:
    ser = serial.Serial("COM3", 115200, timeout=1)
    if not ser.is_open:
        ser.open()
    print("Serial port initialized.")
except Exception as e:
    print(f"Failed to initialize serial port: {e}")
    exit(1)

time.sleep(2)  # Allow time for the serial connection to stabilize


def create_command_message(command):
    # List of valid directions
    valid_directions = ["RIGHT", "LEFT", "UP", "DOWN", "WAVE", "GRAB"]

    # Try to parse the command
    parts = command.split()
    if parts[0] == "WAVE":
        message = "WAVE"
        return message.ljust(14)
    elif parts[0] == "GRAB":
        message = "GRAB"
        return message.ljust(14)

    elif len(parts) == 3 and parts[0] == "MOVE" and parts[1] in valid_directions:
        direction = parts[1]
        try:
            number = int(parts[2])  # Convert the last part to a number
        except ValueError:
            return "ERROR INVALID"  # If the number part is invalid

        # Format the message to ensure the direction part is 8 characters
        message = f"MOVE {direction}"
        message = message.ljust(11)  # Ensure the message is 11 characters long (padded with spaces)

        # Ensure number is 3 digits, padded with leading zeros if needed
        number_str = f"{number:03}"

        # Append the number at the end of the 11-character message
        final_message = f"{message}{number_str}"
        print(f"Formatted message: {final_message}")
        return final_message
    else:
        return "ERROR INVALID".rjust(14)  # Invalid command format


def send_command(command):
    try:
        command = create_command_message(command)
        ser.write(f"{command}".encode())  # Send the command
        print(f"Command sent: {command}")

        ser.flush()  # Ensure data is sent
    except Exception as e:
        print(f"Error sending command: {e}")


# Use the microphone for audio input
with sr.Microphone() as source:
    print("Adjusting for ambient noise...")
    recognizer.adjust_for_ambient_noise(source)  # Adjust microphone for ambient noise
    print("Ready to receive commands. Speak now!")

    while True:
        try:
            # Debug print to show the loop is active

            if ser.in_waiting:  # Check if there is data in the serial buffer
                data = ser.readline().decode('utf-8').strip()  # Read a line, decode, and remove newlines
                print(f"Received: {data}")
            print("Listening...")
            # Uncomment the next line to actually listen to audio
            audio = recognizer.listen(source)

            # Simulate recognized text for testing
            text = recognizer.recognize_google(audio)
            text = text.upper()
            text = create_command_message(text)
            print(f"Recognized text: {text}")
            send_command(text)
            time.sleep(5)

        except sr.UnknownValueError:
            print("Speech recognition could not understand the audio.")
        except sr.RequestError as e:
            print(f"Could not request results from Google Speech Recognition service; {e}")
        except Exception as e:
            print(f"Error in recognition loop: {e}")

        # Delay to avoid flooding the system with commands
        time.sleep(5)
