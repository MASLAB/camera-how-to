# Camera how-to
Guide for using a camera with ROS2.

## Prerequisite
1. [ROS2 installation](https://github.com/MASLAB/ros2-setup)
2. Access to a webcam (either laptop's webcam, or preferably the provided USB webcam)

## Connection to VirtualBox
To connect webcam in Virtualbox, select `Devices > USB`. You will get to select either your laptop integrated webcam, the provided webcam, or any other webcam you have attached to your computer. **DO NOT** use `Devices > Webcams`.

<p align="center">
<img src="image/vb_usb.png" width="75%" />
</p>

## Test webcam
You can test your webcam by installing `Cheese`, a camera application similiar to Apple's `Photobooth` or Windows's `Camera`. It is available in the Ubuntu App Center.

# ROS2 Image Processing
## Image publishing
Webcam video feed can be published as ROS2 [`Image`](https://docs.ros.org/en/jazzy/p/sensor_msgs/interfaces/msg/Image.html) message. This message is available as part of the standard (preinstalled) `sensor_msgs` package. 

Multiple nodes can subscribe to the webcam images and perform their own's image processing and publish their result. Example use case: a node that recognizes the wall and publish a distance to the wall, a node that recognizes the cubes and publish the locations of the cubes with their associated color.

To use camera in ROS2, let's use the ROS2 package `v4l2_camera`. More information on how to customize `v4l2_camera` can be found on the documentation site at: https://docs.ros.org/en/jazzy/p/v4l2_camera/index.html.

For now, let's start with the basics.

### `v4l2_camera` Installation
To install `v4l2_camera`, install the `ros-jazzy-v4l2-camera` package with:

```shell
sudo apt install ros-jazzy-v4l2-camera
```

To get images more quickly, we will also need `image-transport` to enable compression. Install it with:

```shell
sudo apt install ros-jazzy-image-transport
```

### Start publishing image
To start publishing images with `v4l2_camera`, run its camera node:

```shell
ros2 run v4l2_camera v4l2_camera_node
```

This command spins up a camera publishing node and publish camera images from camera. We can provide other parameters to control the camera. For example, we can set the image size to 1280 x 720 with `image_size` parameter:

```shell
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:=[1280,720]
```

For more parameters, see: https://docs.ros.org/en/jazzy/p/v4l2_camera/index.html#parameters

> [!TIP]
> In general, you can pass in parameters to a ROS2 node like this:
> ```shell
> ros2 run package_name executable_name --ros-args -p param1_name:=param1_value -p param2_name:=param2_value
> ```

> [!IMPORTANT]
> If your camera seems to run very slowly, it is likely due to automatic light balance. Disable it with `white_balance_automatic:=False`

If `v4l2_camera_node` runs successfully, you will see something like this:

<p align="center">
<img src="image/v4l2_run.png" width="50%" />
</p>

> [!NOTE]
> There are some errors on here, some are typical because cameras may not supports all the controls and encodings that `v4l2_camera` uses. One is particularly interesting and can be fixed; it is the `[camera_calibration_parsers]` error. This happens because we have not calibrated the camera yet. We will do them when we work on video homography. For now, we can ignore these errors.

### Visualizing image
To see the images published, open `rqt` like in the [FizzBuzz](https://github.com/MASLAB/ROS2_FizzBuzz?tab=readme-ov-file#rqt) tutorial.

This time, instead of looking at the node graph, let's look at images with `Plugins > Visualization > Image View`. Select the `/image_raw/compressed` topic with the drop down menu on top left corner to view the compressed images.

<p align="center">
<img src="image/camera_view.png" width="75%" />
</p>

> [!NOTE]
> Image view with RQT on virtual machine may crash from time to time, especially when trying to view uncompressed image. Therefore, make sure to view the compressed images instead of the raw ones.

## Image processing
We will use OpenCV (Open Computer Vision) for processing the images. Let's install OpenCV and make a node to subscribe to and process the images.

Before using OpenCV with ROS2, it will be helpful to checkout some examples here: https://github.com/MASLAB/cv-samples. This is from MASLAB 2020 but the code is still functional for `color_segmentation`. We will use it for detecting cubes later. For more OpenCV tutorials, check out https://www.geeksforgeeks.org/opencv-python-tutorial/

### Installation
To get OpenCV, install `python3-opencv` using `sudo apt install ...` as we have been using for other packages.

To convert ROS2 images to OpenCV, also install `ros-jazzy-cv-bridge`.

### ROS2 image processing package
#### Create package
Let's create a new ROS2 package in a ROS2 workspace. You may also create a new workspace or use the same one as FizzBuzz.

Navigate to your workspace with `cd ~/ros2_ws` or wherever you have your workspace. Create a new `image_processing` package:

```shell
cd ./src
ros2 pkg create --build-type ament_python image_processing
```

Edit `package.xml` to make it depends on `rclpy`, `std_msgs` (for publishing cube coordinate), `sensor_msgs` (for `Image` message), `cv_bridge` and `cv2` (for OpenCV).

#### Cube detecting node
Let's create a node that subscribes to the images and find cubes.

Make a node named `cube_detect` with `cube_detect_node.py` in `src/image_processing/image_processing/` (similar to FizzBuzz). 

For this node, do the following:
1. Import `CompressedImage` from `sensor_msgs.msg`
1. Import `cv2`
1. Import `CvBridge` from `cv_bridge`
1. Import `numpy` as `np`
1. Create a callback for processing an image called `image_callback` with:
    ```Python
    def image_callback(self, msg: CompressedImage):
    ```
1. In `__init__`:
    1. Initialize a new `CvBridge` with `self.bridge = CvBridge()`
    1. Make a subscriber `image_sub` that subscribes to `image_raw/compressed` topic with `CompressedImage` type and `image_callback` callback function
    1. Make a publisher `cube_image_pub` that publishes to `cube_image/compressed` topic with `CompressedImage` type
1. In `image_callback`:
    1. Convert the message to a CV2 image frame with `frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")`. (the `bgr8` argument converts the image to BGR format to be used later)
    1. Apply the same image processing technique as in https://github.com/MASLAB/cv-samples/blob/master/color_segmentation.py to find the contours of a green cube. You may want to tune your HSV thresholds for best results
    1. Instead of showing the image with `cv2.imshow`, publish the debugging image with
        ```Python
        cube_image_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
        self.cube_image_pub.publish(cube_image_msg)
        ```

Add the node to `setup.py`

> [!TIP]
> Refer to [FizzBuzz subscriber node](https://github.com/MASLAB/ROS2_FizzBuzz?tab=readme-ov-file#fizzbuzz-node) for example of how to do these actions. Fully implemented `image_processing` package is also available for reference.

## Cube detection test
Now that we have the image published and processed, run both `v4l2_camera_node` and `cube_detect_node`. Check `rqt` image view for the debugging image.

## Launch files
We can also make a launch file like [FizzBuzz launch](https://github.com/MASLAB/ROS2_FizzBuzz?tab=readme-ov-file#making-a-launch-file).

1. `Setup `package.xml`, `setup.py`, and add folder for launch
1. Create the launch file
1. Add the `cube_detector` node to launch file
1. Add `v4l2_camera` node with:
    ```Python
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        parameters=[
            {"image_size": [640,480]},
        ]
    )
    ```
1. Add parameters as necessary

Stop all currently running nodes then build and run the launch file. Open `rqt` and check that we still have cube detection.

## Set up on Raspberry Pi 5 (NEED ETHERNET)
To test it out on the Pi, connect to it via Ethernet. Once connected:

### Simple test
#### On Pi (through SSH or VSCode Remote):
1. Connect a webcam to a Pi's USB port
1. Install the necessary packages (`v4l2_camera`, OpenCV, etc)
1. Run the `v4l2_camera_node` on the Pi 

#### On personal computer
1. See that you get images with `rqt` on your computer

### Add image processing
#### On personal computer
1. If you have not already, clone your team's workspace's github repository at `git@github.mit.edu:maslab-2025/team-XX.git` to your computer (replace `XX` with your team number). This repository is already set up on your team's Raspberry Pi as workspace `ros_ws`. 
1. Create a new branch to experiment with
1. Add your `image_processing` code to that branch
1. Publish the branch

#### On Pi
1. Checkout the experimental branch
1. Pull the changes
1. Build the `image_processing` package and `source` the overlay
1. Launch the package

#### Return to personal computer
1. Run `rqt` and check for the processed image, now from the Pi.
