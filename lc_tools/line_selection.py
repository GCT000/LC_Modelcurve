import cv2
import sys
import os

# get args
if len(sys.argv) != 2:
    print("Usage: python line_selection.py <image_path> <points_file>")
    sys.exit(1)

points_file = sys.argv[2]

# init variables
points = []

def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # add selected point and show
        points.append((x, y))
        cv2.circle(image, (x, y), 3, (0, 0, 255), -1)
        cv2.imshow('image', image)

def save_points_to_file():
    with open(points_file, 'w') as f:
        for point in points:
            f.write(f"{point[0]}, {point[1]}\n")
    print(f"Points saved to {points_file}")

# load image and resize
image_path = sys.argv[1]
image = cv2.imread(image_path)
image = cv2.resize(image, (image.shape[1], image.shape[0]))

cv2.imshow('image', image)
cv2.setMouseCallback('image', on_mouse)

while True:
    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):  # press 's' to save points
        save_points_to_file()
        break
    elif key == 27:  # press 'ESC' to exit
        break

cv2.destroyAllWindows()
