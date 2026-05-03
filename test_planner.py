import cv2
import numpy as np
import math
from scipy import ndimage

# Load map from yaml if possible, or just create a mock one
# Since we don't have a map file directly, let's create a realistic mock
grid = np.zeros((200, 200), dtype=np.uint8)
grid[20:180, 20:180] = 255
# Add a wall in the middle
grid[100:120, 20:100] = 0

mask = grid > 0

labeled_array, num_features = ndimage.label(mask)
if num_features > 1:
    sizes = ndimage.sum(mask, labeled_array, range(1, num_features + 1))
    largest_label = np.argmax(sizes) + 1
    mask = (labeled_array == largest_label)

inset_cells = 7
mask_eroded = ndimage.binary_erosion(mask, iterations=inset_cells)
mask_uint8 = (mask_eroded * 255).astype(np.uint8)

contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
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

step = 10.0 # spacing
pts = []
num_corners = len(corner_indices)
if num_corners > 0:
    for i in range(num_corners):
        start_idx = corner_indices[i]
        end_idx = corner_indices[(i + 1) % num_corners]
        pts.append([[contour_points[start_idx][0], contour_points[start_idx][1]]])
        
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
                pts.append([[curr[0], curr[1]]])
                accumulated = 0.0

print(f"Num pts: {len(pts)}")
