import cv2
import numpy as np

# Create a dummy jagged map
grid = np.zeros((100, 100), dtype=np.uint8)
grid[20:80, 20:80] = 255
# add zigzags
for i in range(20, 80, 5):
    grid[15:20, i:i+2] = 255

from scipy import ndimage
close_radius = int(1.0 / 0.05) # 20 cells
mask = grid > 0

mask_closed = ndimage.binary_closing(mask, iterations=close_radius)
mask_opened = ndimage.binary_opening(mask_closed, iterations=close_radius)

contours, _ = cv2.findContours((mask_opened*255).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
largest = max(contours, key=cv2.contourArea)
print(len(largest))

