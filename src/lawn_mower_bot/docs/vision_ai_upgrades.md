# Computer Vision AI Upgrades

The current `vision_processor.py` relies on **HSV Color Thresholding**. It essentially says:
> *"Anything bright green is grass, anything else is an obstacle."*

While extremely fast (CPU only), it is very fragile. It will fail if:
1. Grass is brown, dead, or covered in shadows.
2. An obstacle is painted green (like a green ball).
3. The lighting changes drastically (clouds vs direct sun).

To make the lawnmower truly autonomous and intelligent, we should upgrade the vision node using modern computer vision AI.

---

## 🚀 Upgrade Path 1: Semantic Segmentation (Recommended)
Semantic Segmentation models evaluate an image pixel-by-pixel. Instead of asking "what color is this?", they ask "what *object class* does this pixel belong to?"

**Suggested Models:**
- **FastSeg** or **BiSeNet** (Extremely fast, realtime on CPU/Edge TPU)
- **Mask2Former** (State of the art, very accurate)

**How to Implement in ROS 2:**
1. Train a model on a dataset of lawns, sidewalks, and common yard obstacles (dogs, toys, hoses).
2. Modify `vision_processor.py` to load the ONNX or TensorRT model.
3. Pass each frame through the model to get a "Semantic Mask".
4. The mask will perfectly outline the grass, allowing the robot to know *exactly* where the grass ends and the mulch/sidewalk begins, regardless of lighting!

---

## 🚀 Upgrade Path 2: Object Detection (YOLOv8)
If the primary goal is just *collision avoidance* rather than precise edge-tracking, an Object Detection model is best.

**Suggested Models:**
- **YOLOv8n** (Nano version, runs realtime on almost anything)

**How to Implement in ROS 2:**
1. Install `ultralytics` package.
2. In `vision_processor.py`, run inference: `results = model.predict(cv_image)`
3. For every bounding box (e.g., "Dog", "Human", "Rock"), project the bottom center of the bounding box onto the ground plane using the same math we already have.
4. **Behavior Trees:** You can now publish *what* the obstacle is. Your Nav2 Behavior Tree can be programmed to smoothly steer around a "Rock", but immediately cut power to the blades and pause if it detects a "Human" or "Pet" within 2 meters!

---

## 🚀 Upgrade Path 3: True 3D Vision (Stereo / Depth)
Currently, our script uses a "Pinhole Camera Projection" mathematical hack. It assumes the ground is an infinite, perfectly flat plane. If the robot goes up a hill, or looks at a tall patch of grass, the math breaks and it projects the obstacle much further away than it really is.

**The Fix:**
Upgrade the physical hardware to an **Intel RealSense** or an **OAK-D** stereo camera.

**How to Implement in ROS 2:**
1. These cameras native ROS 2 drivers that publish a true `sensor_msgs/PointCloud2` out-of-the-box.
2. You no longer need to calculate depth from 2D pixels. The camera hardware uses two lenses to calculate the exact distance (in millimeters) to every pixel using stereo disparity.
3. Feed this true 3D PointCloud directly into the Nav2 `obstacle_layer` costmap. It is 100% accurate and immune to flat-ground assumptions!
