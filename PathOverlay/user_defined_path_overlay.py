import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from PIL import Image

# ----- USER-DEFINED PARAMETERS -----
# Adjust these values based on your map metadata:
# For example, if your map has a resolution of 0.05 m/pixel:
map_resolution = 0.05   # meters per pixel
# And if the origin of your map (bottom-left corner in world coordinates)
# is at (-10, -10) meters (this depends on how your map is generated)
origin_x = -8        # world x coordinate corresponding to the left side of the image
origin_y = -5.0        # world y coordinate corresponding to the bottom of the image

# ----- LOAD THE MAP IMAGE -----
map_filename = 'data20/map_data.pgm'
try:
    map_img = Image.open(map_filename)
    map_data = np.array(map_img)
except Exception as e:
    raise RuntimeError(f"Error loading map image '{map_filename}': {e}")

# Get map dimensions in pixels
map_height, map_width = map_data.shape

# Calculate the extent of the map in world coordinates.
# extent = [left, right, bottom, top]
extent = [
    origin_x,
    origin_x + map_width * map_resolution,
    origin_y,
    origin_y + map_height * map_resolution
]

# ----- LOAD THE ODOMETRY DATA -----
csv_filename = 'data20/odometry_data.csv'
try:
    df = pd.read_csv(csv_filename)
except Exception as e:
    raise RuntimeError(f"Error loading odometry CSV '{csv_filename}': {e}")

# Ensure that the CSV has the necessary columns:
if 'pos_x' not in df.columns or 'pos_y' not in df.columns:
    raise ValueError("CSV file must contain 'pos_x' and 'pos_y' columns.")

# Extract the coordinates from the CSV.
x_coords = df['pos_x'].values
y_coords = df['pos_y'].values

# ----- PLOTTING: OVERLAY PATH ON MAP -----
plt.figure(figsize=(10, 8))

# Show the map. We use origin='lower' so that the (0,0) pixel is at the bottom-left.
plt.imshow(map_data, cmap='gray', extent=extent, origin='lower')

# Overlay the odometry path.
plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='red',
         linewidth=1, markersize=3, label='Odometry Path')

plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.title("Robot Path Overlaid on Map")
plt.legend()
plt.grid(True)
plt.xlim(extent[0], extent[1])
plt.ylim(extent[2], extent[3])

print("X range:", x_coords.min(), x_coords.max())
print("Y range:", y_coords.min(), y_coords.max())
print("Map extent:", extent)


# Show the result.
plt.show()

# Optionally, save the figure to a file.
# plt.savefig('overlay_path_on_map.png', dpi=300)
