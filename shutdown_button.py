import RPi.GPIO as GPIO
import time
import subprocess

# Define GPIO pin for the button
BUTTON_PIN = 3

# Setup GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Variable to track button state
button_state = GPIO.HIGH

# Function to handle button events
def button_callback(channel):
    global button_state
    button_state = GPIO.input(BUTTON_PIN)

    if button_state == GPIO.HIGH:
        print("Shutdown button pressed.")
    else:
        stop_rosbag()   # Stop ROS bag recording
        time.sleep(30)  # Wait for 30 seconds
        print("Shutting down...")
        subprocess.call(['sudo', 'shutdown', '-h', 'now'])

# Function to gracefully stop rosbag recording
def stop_rosbag():
    print("Stopping ROS bag recording...")
    subprocess.call(['rosnode', 'kill', '/bag_record'])

# Add event listener for both rising and falling edges (button press and release)
GPIO.add_event_detect(BUTTON_PIN, GPIO.BOTH, callback=button_callback, 
bouncetime=200)

try:
    # Keep the script running
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    # Cleanup GPIO on script exit
    GPIO.cleanup()

