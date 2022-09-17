import cv2, pyrealsense2
import realsense_depth

dc = realsense_depth.DepthCamera()


re, depth_frame, colour_frame = dc.get_frame()

cv2.imshow("Colour frame", colour_frame)
cv2.waitKey(0)