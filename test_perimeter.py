import cv2
import numpy as np
import math

# Create a dummy map: 100x100
# Free space = middle 80x80
grid = np.zeros((100, 100), dtype=np.uint8)
grid[10:90, 10:90] = 255
# add a cutout to make it non-convex
grid[40:60, 80:100] = 0

contours, _ = cv2.findContours(grid, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
largest_contour = max(contours, key=cv2.contourArea)

epsilon = 0.005 * cv2.arcLength(largest_contour, True)
approx = cv2.approxPolyDP(largest_contour, epsilon, True)

contour_points = largest_contour[:, 0, :]
corner_indices = []
for pt in approx:
    idx = np.where((contour_points[:, 0] == pt[0][0]) & (contour_points[:, 1] == pt[0][1]))[0]
    if len(idx) > 0:
        corner_indices.append(idx[0])

corner_indices.sort()

step = 10.0 # 10 pixels spacing
pts = []
num_corners = len(corner_indices)
for i in range(num_corners):
    start_idx = corner_indices[i]
    end_idx = corner_indices[(i + 1) % num_corners]
    pts.append(contour_points[start_idx].tolist())
    
    if end_idx < start_idx:
        contour_segment = np.concatenate((contour_points[start_idx:], contour_points[:end_idx+1]))
    else:
        contour_segment = contour_points[start_idx:end_idx+1]
        
    accumulated = 0.0
    for j in range(1, len(contour_segment) - 1):
        prev = contour_segment[j - 1]
        curr = contour_segment[j]
        seg_len = math.hypot(curr[0] - prev[0], curr[1] - prev[1])
        accumulated += seg_len
        if accumulated >= step:
            pts.append(curr.tolist())
            accumulated = 0.0

print(f"Num points: {len(pts)}")
for p in pts:
    print(p)

