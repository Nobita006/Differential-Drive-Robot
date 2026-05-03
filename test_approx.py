import cv2
import numpy as np

# Create a simple contour
contour = np.array([[[0, 0]], [[10, 0]], [[10, 10]], [[5, 5]], [[0, 10]]], dtype=np.int32)
epsilon = 1.0
approx = cv2.approxPolyDP(contour, epsilon, True)

print("Original:")
print(contour)
print("Approx:")
print(approx)

# Are approx points exactly in contour?
for pt in approx:
    found = False
    for i, cpt in enumerate(contour):
        if np.array_equal(pt[0], cpt[0]):
            print(f"Point {pt[0]} found at index {i}")
            found = True
            break
    if not found:
        print(f"Point {pt[0]} NOT found!")

