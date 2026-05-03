import cv2
import numpy as np

# Create a small grid
grid = np.zeros((20, 20), dtype=np.uint8)
grid[5:15, 5:15] = 255

contours, _ = cv2.findContours(grid, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
largest_contour = max(contours, key=cv2.contourArea)

approx = cv2.approxPolyDP(largest_contour, 1.0, True)

contour_points = largest_contour[:, 0, :]
corner_indices = []
for pt in approx:
    idx = np.where((contour_points[:, 0] == pt[0][0]) & (contour_points[:, 1] == pt[0][1]))[0]
    print(f"pt: {pt[0]}, idx: {idx}")
    if len(idx) > 0:
        corner_indices.append(idx[0])
print(corner_indices)
