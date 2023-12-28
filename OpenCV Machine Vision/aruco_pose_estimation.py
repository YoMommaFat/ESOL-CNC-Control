"""
This demo calculates multiple things for different scenarios.

Here are the defined reference frames:

TAG:
                A y
                |
                |
                |tag center
                O---------> x

CAMERA:
                X--------> x
                | frame center
                |
                |
                V y

F1: Flipped (180 deg) tag frame around x axis
F2: Flipped (180 deg) camera frame around x axis

The attitude of a generic frame 2 respect to a frame 1 can obtained by calculating euler(R_21.T)

We are going to obtain the following quantities:
    > from aruco library we obtain tvec and Rct, position of the tag in camera frame and attitude of the tag
    > position of the Camera in Tag axis: -R_ct.T*tvec
    > Transformation of the camera, respect to f1 (the tag flipped frame): R_cf1 = R_ct*R_tf1 = R_cf*R_f
    > Transformation of the tag, respect to f2 (the camera flipped frame): R_tf2 = Rtc*R_cf2 = R_tc*R_f
    > R_tf1 = R_cf2 an symmetric = R_f
"""

import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math

#--- Define Tag
id_to_find  = 0
marker_size  = 14 #- [cm]

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])

def draw_line(p0, p1, color, img):
    points = np.float32([p0, p1]).reshape(-1,3)
    imgpts, jac = cv2.projectPoints(points, rvecZ, tvecZ, camera_matrix, camera_distortionZ)
    imgpts = np.int32(imgpts).reshape(-1,2)
    imgpts[0][1] = imgpts[0][1] + canvas_height_half
    imgpts[1][1] = imgpts[1][1] + canvas_height_half
    a = tuple(imgpts[0])
    b = tuple(imgpts[1])
    cv2.line(img, a,  b, color, 2)

def draw_angled_rec(p, width, height, angle, color, img):
    x0 = p[0]
    y0 = p[1]
    b = math.cos(angle) * 0.5
    a = math.sin(angle) * 0.5
    pt0 = (x0 - a * height - b * width, y0 + b * height - a * width, p[2])
    pt1 = (x0 + a * height - b * width, y0 - b * height - a * width, p[2])
    pt2 = (2 * x0 - pt0[0], 2 * y0 - pt0[1], p[2])
    pt3 = (2 * x0 - pt1[0], 2 * y0 - pt1[1], p[2])

    draw_line(pt0, pt1, color, img)
    draw_line(pt1, pt2, color, img)
    draw_line(pt2, pt3, color, img)
    draw_line(pt3, pt0, color, img)

#--- Get the camera calibration path
calib_path  = ""
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')
camera_distortionZ  = np.loadtxt(calib_path+'cameraDistortionZ.txt', delimiter=',')

#--- 180 deg rotation matrix around the x axis
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0

#--- Define the aruco dictionary
#aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
#aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

parameters  = aruco.DetectorParameters_create()

#print(cv2.getBuildInformation())

#--- Capture the videocamera (this may also be a video or a picture)
cap = cv2.VideoCapture(0)

#-- Set the camera size as the one it was calibrated with
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 5)

#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

#-- Video streaming
fps = cap.get(cv2.CAP_PROP_FPS)
frame_size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
sys.stderr.write("OpenCV camera data: {}x{} @{}FPS\n".format(frame_size[0], frame_size[1], fps))

#-- Position calculation
canvas_height = 960
canvas_width = 1280
canvas_height_half = 480
canvas_width_half = 640
dist_marker_coil = -60
dist_camera_coil = -60
rvecZ, tvecZ = np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])
offset = np.array((canvas_height_half, 0))
pos_text = 720
dist_cnc_end = 5
ang_marker_err = 10
color_success = (0,255,0)

while True:
    #-- Read the camera frame
    ret, frame = cap.read()
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    
    # Undistort an image
    h,  w = frame.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(camera_matrix,camera_distortion,(w,h),1,(w,h))
    # undistort
    mapx,mapy = cv2.initUndistortRectifyMap(camera_matrix,camera_distortion,None,newcameramtx,(w,h),5)
    frame = cv2.remap(frame,mapx,mapy,cv2.INTER_LINEAR)

    #-- Create canvas
    canvas = np.zeros((canvas_height,canvas_width,3), np.uint8)
#    canvas[:,0:canvas_width//2] = (255,0,0) # (B, G, R)
#    canvas[:,canvas_width//2:canvas_width] = (0,255,0)

    #-- Convert in gray scale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

    #-- Find all the aruco markers in the image
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters, cameraMatrix=camera_matrix, distCoeff=camera_distortion)

    if ids is not None and ids[0] == id_to_find:
        
        #-- ret = [rvec, tvec, ?]
        #-- array of rotation and position of each marker in camera frame
        #-- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
        #-- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

        #-- Unpack the output, get only the first
        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

        #-- Draw the detected marker and put a reference frame over it
        aruco.drawDetectedMarkers(frame, corners)
#        aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
        cv2.drawFrameAxes(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
#        cv2.drawFrameAxes(frame, camera_matrix, camera_distortionZ, rvecZ, tvecZ, 1)
        cv2.line(frame, (320, 240), (420, 240), (0, 0, 255), 3)
        cv2.line(frame, (320, 240), (320, 340), (0, 255, 0), 3)

        #-- Print the tag position in camera frame
        str_position = "MARKER Position x=%4.2f  y=%4.2f  z=%4.2f"%(tvec[0], tvec[1], tvec[2])
        cv2.putText(canvas, str_position, (pos_text, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        #-- Print the tag orientation in camera frame
        str_position = "MARKER Orientation x=%4.2f  y=%4.2f  z=%4.2f"%(math.degrees(rvec[0]), math.degrees(rvec[1]), math.degrees(rvec[2]))
        cv2.putText(canvas, str_position, (pos_text, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        #-- Obtain the rotation matrix tag->camera
        R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
        R_tc    = R_ct.T

        #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
        roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

        #-- Print the marker's attitude respect to camera frame
        str_attitude = "MARKER Attitude r=%4.2f  p=%4.2f  y=%4.2f"%(math.degrees(roll_marker),math.degrees(pitch_marker), math.degrees(yaw_marker))
        cv2.putText(canvas, str_attitude, (pos_text, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        #-- Now get Position and attitude f the camera respect to the marker
        pos_camera = -R_tc*np.matrix(tvec).T

        str_pos_cam = "CAMERA Position x=%4.2f  y=%4.2f  z=%4.2f"%(pos_camera[0].item(), pos_camera[1].item(), pos_camera[2].item())
        cv2.putText(canvas, str_pos_cam, (pos_text, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        #-- Get the attitude of the camera respect to the frame
        roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
        str_att_cam = "CAMERA Attitude r=%4.2f  p=%4.2f  y=%4.2f"%(math.degrees(roll_camera),math.degrees(pitch_camera), math.degrees(yaw_camera))
        cv2.putText(canvas, str_att_cam, (pos_text, 300), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        #-- Check Marker Attitude
        if math.degrees(roll_camera) > ang_marker_err or math.degrees(roll_camera) < -ang_marker_err or math.degrees(pitch_camera) > ang_marker_err or math.degrees(pitch_camera) < -ang_marker_err:
            color_success = (0,0,255); str_att_err = "MARKER Attitude ERROR"
        else:
            color_success = (0,255,0); str_att_err = "MARKER Attitude OK"   
        cv2.putText(canvas, str_att_err, (pos_text, 350), font, 1, color_success, 2, cv2.LINE_AA)

        #-- Position and rotation in 2D
        str_pose = "Marker Pose in 2D x=%4.2f  y=%4.2f  alpha=%4.2f"%(tvec[0], tvec[1], math.degrees(yaw_marker))
        cv2.putText(canvas, str_pose, (pos_text, 500), font, 1, (255, 255, 255), 2, cv2.LINE_AA)

        #-- Calculate coil position
        canvas[offset[0]:offset[0] + canvas_height_half, offset[1]:offset[1] + canvas_width_half] = frame

        dist_x1 = -dist_marker_coil * math.cos(math.pi/2+yaw_marker)
        dist_y1 = dist_marker_coil * math.sin(math.pi/2+yaw_marker)
        str_pos1 = "Coil Position (irt. Marker) x=%4.2f  y=%4.2f"%(dist_x1, dist_y1)
        cv2.putText(canvas, str_pos1, (pos_text, 550), font, 1, (255, 255, 255), 2, cv2.LINE_AA)

        dist_x2 = tvec[0] + dist_x1
        dist_y2 = tvec[1] + dist_y1
        str_pos2 = "Coil Position (irt. Camera) x=%4.2f  y=%4.2f"%(dist_x2, dist_y2)
        cv2.putText(canvas, str_pos2, (pos_text, 600), font, 1, (255, 255, 255), 2, cv2.LINE_AA)

        dist_x3 = dist_x2 - 0
        dist_y3 = dist_y2 - dist_camera_coil
        str_pos3 = "Coil Position (irt. Positioner) x=%4.2f  y=%4.2f"%(dist_x3, dist_y3)
        cv2.putText(canvas, str_pos3, (pos_text, 650), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
        
        color_success = (0,255,0); 
        if dist_x3 > dist_cnc_end:
            color_success = (0,0,255); dist_x3 = dist_cnc_end
        if dist_x3 < -dist_cnc_end:
            color_success = (0,0,255); dist_x3 = -dist_cnc_end
        if dist_y3 > dist_cnc_end:
            color_success = (0,0,255); dist_y3 = dist_cnc_end
        if dist_y3 < -dist_cnc_end:
            color_success = (0,0,255); dist_y3 = -dist_cnc_end
        str_pos4 = "Commanded Position (irt. Positioner) x=%4.2f  y=%4.2f"%(dist_x3, dist_y3)
        cv2.putText(canvas, str_pos4, (pos_text, 700), font, 1, color_success, 2, cv2.LINE_AA)
        
        pos_coil_car = tvec.copy()
        pos_coil_cnc = tvec.copy()
        pos_coil_com = tvec.copy()
        pos_coil_car[0], pos_coil_car[1] = dist_x2, dist_y2
        pos_coil_cnc[0], pos_coil_cnc[1] = 0, dist_camera_coil
        pos_coil_com[0], pos_coil_com[1] = dist_x3, dist_y3+dist_camera_coil

        draw_line(tvec, pos_coil_car, (255,255,255), canvas)
        draw_angled_rec(pos_coil_car, 10, 10, -yaw_marker, (255,255,255), canvas)
        draw_line(pos_coil_car, pos_coil_cnc, (255,255,255), canvas)
        draw_angled_rec(pos_coil_cnc, 20, 20, 0, (0,255,255), canvas)
        draw_angled_rec(pos_coil_com, 10, 10, 0, color_success, canvas)

#        sys.stderr.write("Data4: \n{}\n".format(b))

        #--- Display the frame
        cv2.imshow('frame', canvas)
#        sys.stdout.buffer.write(canvas.tobytes())
#        sys.stdout.buffer.write(cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR).tobytes())

    #--- use 'q' to quit
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break
