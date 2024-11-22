import serial
import pandas as pd

# Set up the serial connection to Arduino 
arduino = serial.Serial('COM4', 9600, timeout=1)  # 

# Initialize empty lists to store x, y, and theta data
x_data = []
y_data = []
theta_data = []

# Wait for Arduino to send the header (optional)
arduino.readline()

# Collect data from Arduino and store it in lists
try:
    while True:
        line = arduino.readline().decode('utf-8').strip()  # Read data from serial port
        print(line)
        if line:
            values = line.split(',')  # Split CSV values by comma
            x_data.append(float(values[0] + 1.135))  # Add x position
            y_data.append(float(values[1]) - 0.905)  # Add y position


            # x_data.append(float(values[0]))  # Add x position
            # y_data.append(float(values[1]))  # Add y position
            # theta_data.append(float(values[2]))  # Add theta (orientation)
except KeyboardInterrupt:
    print("Data collection stopped.")

# Create a pandas DataFrame from the collected data
df = pd.DataFrame({
    'X': x_data,
    'Y': y_data,
    'Theta': theta_data
})

# Export the DataFrame to a CSV file
df.to_csv('lidar_position.csv', index=False)

print("Data exported to 'lidar_position.csv'")

