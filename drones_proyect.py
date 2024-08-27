import cv2 as cv
import cv2.aruco as aruco
from djitellopy import TelloSwarm, Tello
import time

def fly_in_square(tello):
    tello.move_up(30)        
    time.sleep(1)            
    tello.move_left(30)       
    time.sleep(1)             
    tello.move_down(30)     
    time.sleep(1)            
    tello.move_right(30)     
    time.sleep(1)            

    tello.send_rc_control(0, 20, 0, 33)
    time.sleep(30)

def fly1(tello, i):
    tello.move_down(40)
    time.sleep(1)

def fly2(tello, i):
    tello.move_up(40)
    time.sleep(1)

def flip_forward(tello):
    """Flip forward.
    """
    tello.flip("f")
    
def findAruco(img, marker_size=6, total_markers=250, draw=True):
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    key = getattr(aruco, f'DICT_{marker_size}X{marker_size}_{total_markers}')
    arucoDict = aruco.getPredefinedDictionary(key)
    arucoParam = aruco.DetectorParameters()
    aruco_detector = aruco.ArucoDetector(arucoDict, arucoParam)
    bbox, ids, _ = aruco_detector.detectMarkers(gray)
    print(ids)

    if draw and ids is not None:
        aruco.drawDetectedMarkers(img, bbox, ids)
        return img, ids

if __name__ == "__main__":
    drone_ip = "192.168.50.249"   

    drone = Tello(host=drone_ip)
    drone.connect()

    # Start the video stream
    drone.streamon()

    # Get the frame read object
    frame_read = drone.get_frame_read()

    ids = None

    temp = True

    # Use 'q' to quit from the loop
    while temp:
        frame = frame_read.frame
        if frame is None:
            continue

        # Resize for better display
        frame = cv.resize(frame, (360, 240))

        # Display the resulting frame
        cv.imshow('Tello Video', frame)
    
        # Detect ArUco markers in the frame
        frame, ids = findAruco(frame)
        
        # Display the resulting frame with ArUco markers
        cv.imshow('Aruco Marker Detection', frame)

        if ids == 1 or ids == 2 :
            temp = False

        # Break the loop on 'q' key press
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    drone.streamoff()
    drone.end()
    cv.destroyAllWindows()

    swarm = TelloSwarm.fromIps([
        "192.168.50.109",  # ANGIE
        "192.168.50.249",  # EUGENIO
        "192.168.50.142"    # ARI
    ])

    swarm.connect()
    swarm.takeoff()

    if ids is not None and 1 in ids:
        print("id = 1 --> square_circle")
        swarm.parallel(lambda i, tello: fly_in_square(tello))

    if ids is not None and 2 in ids:
        print("id = 2 --> fuente")
        swarm.sequential(lambda i, tello: fly2(tello, i))
        swarm.sequential(lambda i, tello: fly1(tello, i))
        swarm.parallel(lambda i, tello: flip_forward(tello))
        swarm.land()
        swarm.end()




