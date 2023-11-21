#https://pysource.com
import cv2
from test.realsense_depth import DepthCamera


# Load Realsense camera
rs =DepthCamera()


while True:
	# Get frame in real time from Realsense camera
	ret, bgr_frame, depth_frame = rs.get_frame()

	




	cv2.imshow("depth frame", depth_frame)
	cv2.imshow("Bgr frame", bgr_frame)

	key = cv2.waitKey(1)
	if key == 27:
		break

rs.release()
cv2.destroyAllWindows()