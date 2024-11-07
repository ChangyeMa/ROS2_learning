import cv2
import numpy as np
from dt_apriltags import Detector

def ImageToWorld(H,K,center):
    # calculate rotation and translation vectors
    retval, rotations, translations, normals = cv2.decomposeHomographyMat(H, K)

    R = rotations[0]  # Example: taking the first solution
    t = translations[0]  # Example: taking the corresponding translation

    # print the rotation and translation vectors
    print(f"Rotation Matrix: {R}")
    print(f"Translation Vector: {t}")

    # calculate 3D coordinates of the point in the camera frame
    x_center = center[0]
    y_center = center[1]
    P_center_2D = np.array([[x_center], [y_center],[1]])
    print(f"2D coordinates of the center of the tag: {P_center_2D}")
    P_center_3D = R@P_center_2D + t
    print(f"3D coordinates of the center of the tag: {P_center_3D}")

    # calculate transformation matrix from camera frame to world frame

    # Example: Camera pose in world frame
    R_world_to_camera = np.eye(3)  # Identity matrix if the camera is aligned with the world
    t_world_to_camera = np.array([0, 0, 0])  # Assume the camera is at the origin in the world frame

    # Compute the transformation from camera to world
    R_camera_to_world = R_world_to_camera.T # Transpose of rotation to invert it
    t_camera_to_world = -R_camera_to_world@t_world_to_camera # Adjusted translation

    # Construct the camera-to-world extrinsic matrix
    extrinsic_camera_to_world = np.eye(4)
    extrinsic_camera_to_world[:3, :3] = R_camera_to_world 
    extrinsic_camera_to_world[:3, 3] = t_camera_to_world
    print(f"Extrinsic matrix from camera to world: {extrinsic_camera_to_world}")

    # Calculate the coordinates of the center of the tag in the world frame
    P_center_3D_homogeneous = np.append(P_center_3D, 1)
    P_center_3D_world = extrinsic_camera_to_world@P_center_3D_homogeneous

    print(f"3D coordinates of the center of the tag in the world frame: {P_center_3D_world}")
    return P_center_3D_world


# ============== detect tag in a video stream ==============
cap = cv2.VideoCapture(0)

while True:
    #capture frame by frame
    ret, img = cap.read()

    # convert the image to grayscale otherwise the detection will not work
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to eliminate noise
    kernel_size = 7
    blur = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

    # apply threshold to avoid effect from shadows
    thresh = cv2.threshold(blur, 50, 230, cv2.THRESH_BINARY)[1]

    # detector = apriltag.Detector()
    # detections = detector.detect(thresh)

    # using the dukietown version python binding:
    at_detector = Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)
    detections = at_detector.detect(thresh,
                                    estimate_tag_pose=True,
                                    camera_params=[600,600,320,240],
                                    tag_size=0.049)

    if len(detections) == 0:
        # if no tags are detected display this message
        print("No AprilTags detected.")

    else:
        # Print detection results
        for i, detection in enumerate(detections):

            # get the corner coordinates and draw tag boundary
            corners = detection.corners.astype(int)

            for i in range(4):
                pt1 = tuple(corners[i])
                pt2 = tuple(corners[(i + 1) % 4])
                cv2.line(img, pt1, pt2, (0, 255, 0), 2)

            # get the family and detected id of the tag
            tf = detections[0].tag_family
            id = detections[0].tag_id
            center = detections[0].center
            H =detections[0].homography
            # === using dt_apriltags to get the rotation matrix and translation vector ===
            R_matrix = detections[0].pose_R
            t = detections[0].pose_t
            '''
            R_matrix not validated yet
            t is ok for depth measurement
            '''
            
            # print(f"Tag Family of first detection: {tf}")
            # print(f"Tag ID of first detection: {id}")
            # print(f"Center of first detection: {center}")
            print(f"Rotation matrix method 1: {R_matrix}")
            print(f"Translation method 1: {t}")

            # # ==== method 2: using cv2.decomposeHomographyMat to get the rotation matrix and translation vector
            # # define a intrinsic camera matrix
            # K = np.array([[1200, 0, 1080],
            #               [0, 1200, 540],
            #               [0, 0, 1]])
            # # calculate rotation and translation vectors
            # retval, rotations, translations, normals = cv2.decomposeHomographyMat(H, K)

            # R_2 = rotations[0]  # Example: taking the first solution
            # t_2 = translations[0]  # Example: taking the corresponding translation
            # print(f"Rotation matrix method 2: {R_2}")
            # print(f"Translation method 2: {t_2}")
            # # === end of method 2 ===

            # assume that the DT version is giving R and t in from world to camera
            # Compute the transformation from camera to world
            R_camera_to_world = R_matrix .T # Transpose of rotation to invert it
            t_camera_to_world = -R_camera_to_world@t# Adjusted translation
            # print(f"t_camera_to_world:{t_camera_to_world}")

            # calculate 3D coordinates of the point in the camera frame
            x_center = detections[0].center[0]
            y_center = detections[0].center[1]
            P_center_2D = np.array([[x_center], [y_center],[1]])
            # print(f"2D coordinates of the center of the tag: {P_center_2D}")
            P_center_3D = R_matrix@P_center_2D + t
            # P_center_3D=np.array([[x_center], [y_center],[1]])

            print(f"3D coordinates of the center of the tag: {P_center_3D}")

            # Example: Camera pose in world frame

            # define rotation matrix of camera around x axis
            theta_x=0
            R_x = np.array([[1, 0, 0],
                            [0, np.cos(theta_x), -np.sin(theta_x)],
                            [0, np.sin(theta_x), np.cos(theta_x)]])
            
            # define rotation matrix of camera around y axis
            theta_y=0#np.pi/4
            R_y = np.array([[np.cos(theta_y), 0, -np.sin(theta_y)],
                            [0, 1, 0],
                            [np.sin(theta_y), 0, np.cos(theta_y)]])

            # define rotation matrix of camera around z axis
            theta_z=0
            R_z = np.array([[np.cos(theta_z), -np.sin(theta_z), 0],
                            [np.sin(theta_z), np.cos(theta_z), 0],
                            [0, 0, 1]])

            R_world_to_camera = R_x@R_y@R_z  # Identity matrix if the camera is aligned with the world
            t_world_to_camera = np.array([0, 0, 0])  # Assume the camera is at the origin in the world frame

            # Construct the camera-to-world extrinsic matrix
            extrinsic_camera_to_world = np.eye(4)
            extrinsic_camera_to_world[:3, :3] = R_camera_to_world 
            extrinsic_camera_to_world[:3, 3:4] = t_camera_to_world
            # print(f"Extrinsic matrix from camera to world: {extrinsic_camera_to_world}")

            # Calculate the coordinates of the center of the tag in the world frame
            P_center_3D_homogeneous = np.append(P_center_3D, 1)
            P_center_3D_world = extrinsic_camera_to_world@P_center_3D_homogeneous
            print(f"3D coordinate in world {P_center_3D_world}")

            # draw the center and the id of the tag
            img=cv2.circle(img, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)
            img=cv2.putText(img, str(id), 
                (int(center[0]), int(center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # show the img with the detections on the screen
    cv2.imshow('test', img)
    # cv2.imshow('test', thresh)

    # break the loop if 'q' is pressed
    key = cv2.waitKey(100)
    if key == ord('q'):
        break

cv2.destroyAllWindows()
cap.release()



# # ============== detect tag in a single image ==============
# # Load the image in grayscale

# imagepath = 'tag36h11_0.png'
# image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)

# # Apply Gaussian blur to eliminate noise
# kernel_size = 7
# blur = cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)

# # apply threshold to avoid effect from shadows
# thresh = cv2.threshold(blur, 50, 230, cv2.THRESH_BINARY)[1]
# image = thresh



# # Check if the image is loaded properly
# if image is None:
#     print(f"Error: Image at path '{imagepath}' could not be loaded.")
# else:
#     # Display the image
#     cv2.imshow("Image", image)
#     cv2.waitKey(1)  # "0" to wait for a key press to close the window
#     cv2.destroyAllWindows()

#     # Initialize the AprilTag detector
#     options = apriltag.DetectorOptions(families="tag36h11",
#                                        border=1,
#                                        nthreads=4,
#                                        quad_decimate=1.0,
#                                        quad_blur=0.0,
#                                        refine_edges=True,
#                                        refine_decode=False,
#                                        refine_pose=False,
#                                        debug=False,
#                                        quad_contours=True)
#     detector = apriltag.Detector()

#     # Detect AprilTags in the image
#     detections = detector.detect(image)

#     # Check if any tags are detected
#     if len(detections) == 0:
#         print("No AprilTags detected.")
#     else:
#         # Print detection results
#         # for i, detection in enumerate(detections):
#         #     print(f"Detection {i+1}: {detection}")

#         # Example of accessing the tag family of the first detection
#         tf = detections[0].tag_family
#         id = detections[0].tag_id
#         H = detections[0].homography
        
#         # print other attributes of the first detection
#         print(f"Center of first detection: {detections[0].center}")
#         print(f"Homography Matrix: {detections[0].homography}")
#         # print(f"Detected corners: {detections[0].corners}")
#         print(f"Tag Family of first detection: {tf}")
#         print(f"Tag ID of first detection: {id}")

#         # define a intrinsic camera matrix
#         K = np.array([[585.75, 0, 320.5], [0, 585.75, 240.5], [0, 0, 1]])

#         P_3D = ImageToWorld(H,K,detections[0].center)

#         # # calculate rotation and translation vectors
#         # retval, rotations, translations, normals = cv2.decomposeHomographyMat(H, K)

#         # R = rotations[0]  # Example: taking the first solution
#         # t = translations[0]  # Example: taking the corresponding translation

#         # # print the rotation and translation vectors
#         # print(f"Rotation Matrix: {R}")
#         # print(f"Translation Vector: {t}")

#         # # calculate 3D coordinates of the point in the camera frame
#         # x_center = detections[0].center[0]
#         # y_center = detections[0].center[1]
#         # P_center_2D = np.array([[x_center], [y_center],[1]])
#         # print(f"2D coordinates of the center of the tag: {P_center_2D}")
#         # P_center_3D = R@P_center_2D + t
#         # print(f"3D coordinates of the center of the tag: {P_center_3D}")

#         # # calculate transformation matrix from camera frame to world frame

#         # # Example: Camera pose in world frame
#         # R_world_to_camera = np.eye(3)  # Identity matrix if the camera is aligned with the world
#         # t_world_to_camera = np.array([0, 0, 0])  # Assume the camera is at the origin in the world frame

#         # # Compute the transformation from camera to world
#         # R_camera_to_world = R_world_to_camera.T # Transpose of rotation to invert it
#         # t_camera_to_world = -R_camera_to_world@t_world_to_camera # Adjusted translation

#         # # Construct the camera-to-world extrinsic matrix
#         # extrinsic_camera_to_world = np.eye(4)
#         # extrinsic_camera_to_world[:3, :3] = R_camera_to_world 
#         # extrinsic_camera_to_world[:3, 3] = t_camera_to_world
#         # print(f"Extrinsic matrix from camera to world: {extrinsic_camera_to_world}")

#         # # Calculate the coordinates of the center of the tag in the world frame
#         # P_center_3D_homogeneous = np.append(P_center_3D, 1)
#         # P_center_3D_world = extrinsic_camera_to_world@P_center_3D_homogeneous

#         # print(f"3D coordinates of the center of the tag in the world frame: {P_center_3D_world}")



