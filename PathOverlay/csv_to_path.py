import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file with odometry data.
# Adjust the file path if needed.
df = pd.read_csv('data20/odometry_data.csv')

# Check if the necessary columns exist
if 'pos_x' not in df.columns or 'pos_y' not in df.columns:
    raise ValueError("CSV file must contain 'pos_x' and 'pos_y' columns.")

# Extract the X and Y coordinates
x_coords = df['pos_x']
y_coords = df['pos_y']

# Create the plot
plt.figure(figsize=(8, 6))
plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='blue', label='Robot Path')
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.title("Robot Path Visualization")
plt.legend()
plt.grid(True)
plt.axis('equal')  # Ensures that the x and y axes are equally scaled

# Show the plot
plt.show()
