#!/usr/bin/env python3

#import argparse
import cv2
import rospy
from geometry_msgs.msg import PointStamped

from yolo import YOLO
'''
ap = argparse.ArgumentParser()
ap.add_argument('-n', '--network', default="normal", choices=["normal", "tiny", "prn", "v4-tiny"],
                help='Network Type')
ap.add_argument('-d', '--device', type=int, default=0, help='Device to use')
ap.add_argument('-s', '--size', default=416, help='Size for yolo')
ap.add_argument('-c', '--confidence', default=0.2, help='Confidence for yolo')
ap.add_argument('-nh', '--hands', default=-1, help='Total number of hands to be detected per frame (-1 for all)')
args = ap.parse_args()
'''
'''
if args.network == "normal":
    print("loading yolo...")
    yolo = YOLO("models/cross-hands.cfg", "models/cross-hands.weights", ["hand"])
elif args.network == "prn":
    print("loading yolo-tiny-prn...")
    yolo = YOLO("models/cross-hands-tiny-prn.cfg", "models/cross-hands-tiny-prn.weights", ["hand"])
elif args.network == "v4-tiny":
    print("loading yolov4-tiny-prn...")
    yolo = YOLO("models/cross-hands-yolov4-tiny.cfg", "models/cross-hands-yolov4-tiny.weights", ["hand"])
else:
    print("loading yolo-tiny...")
    '''

class Camera:
    def __init__(self):
        self.hand_coord_pub = rospy.Publisher('/hand_coord', PointStamped, queue_size=1)
        self.hand_coord = PointStamped()
        self.hand_coord.header.frame_id = "camera_frame"

        model_cfg = rospy.get_param("yolo_path")
        cfg_path = model_cfg + "/cross-hands-tiny.cfg"
        weights_path = model_cfg + "/cross-hands-tiny.weights"
        rospy.loginfo(cfg_path)
        rospy.loginfo(weights_path)
        

        yolo = YOLO(cfg_path, weights_path, ["hand"])

        yolo.size = 416
        yolo.confidence = 0.2
        hands = 1

        print("starting webcam...")
        cv2.namedWindow("preview")
        vc = cv2.VideoCapture(4) #realsense d435i rgb frame detected on /video4 for adrian's computer

        if vc.isOpened():  # try to get the first frame
            rval, frame = vc.read()
        else:
            rval = False

        while rval:
            width, height, inference_time, results = yolo.inference(frame)

            # display fps
            cv2.putText(frame, f'{round(1/inference_time,2)} FPS', (15,15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,255,255), 2)

            # sort by confidence
            results.sort(key=lambda x: x[2])

            # how many hands should be shown
            hand_count = 1
            #hand_count = len(results)
            #if hands != -1:
            #    hand_count = int(hands)

            # display hands
            for detection in results[:hand_count]:
                id, name, confidence, x, y, w, h = detection
                cx = x + (w / 2)
                cy = y + (h / 2)
                rospy.loginfo(str(cx) + ", " + str(cy))

                self.hand_coord.point.x = cx
                self.hand_coord.point.y = cy
                self.hand_coord_pub.publish(self.hand_coord)

                # draw a bounding box rectangle and label on the image
                color = (0, 255, 255)
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                text = "%s (%s)" % (name, round(confidence, 2))
                cv2.putText(frame, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, color, 2)

            cv2.imshow("preview", frame)

            rval, frame = vc.read()

            key = cv2.waitKey(20)
            if key == 27:  # exit on ESC
                rospy.loginfo("shutting down")
                rospy.is_shutdown()
                break

        cv2.destroyWindow("preview")
        vc.release()

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('camera_node')
    cam = Camera()
    rospy.spin()
