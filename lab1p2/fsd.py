import cv2
import sys
import time
import utils
import signal
import numpy as np
import multiprocessing as mp

import picar_4wd as fc
from a_star import *
from mapping import scan_dist, map_obj

MODE = 'legacy'

if MODE=='legacy':
    #import picar_4wd as fc
    import utils
    from tflite_support.task import core, vision, processor
elif MODE=='picamera':
    from tflite_runtime.interpreter import Interpreter
    #import numpy as np
    from picamera2 import Picamera2
    from picarx import Picarx
    import picamera_utils
    LABEL_PATH='labelmap.txt'


# Motor contorl constants
DIRECITONS = ['W', 'N', 'E', 'S']
FORWARD_SPEED = 8
FORWARD_TIME = 0.4
TURN_SPEED = 30
TURN_TIME = 1

# Define model and camera parameters
OBJ_DETECT_MODEL = 'efficientdet_lite0.tflite'
CAMERA_ID = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
NUM_THREADS = 4
ENABLE_EDGETPU = False

# Define A Start grid size
GRID_SIZE = 11


def get_direction_distance(start, end):
    ew = end[0] - start[0]
    ns = end[1] - start[1]
    if ns and ew == 0:
        return ('S', ns) if ns > 0 else ('N', ns)
    elif ew and ns == 0:
        return ('E', ew) if ew > 0 else ('W', ew)
    else:
        raise Exception(f"Unexpected start {start} and end {end}")

def cleanup(signum, frame):
    """Handle Ctrl+C to stop the car and clean up."""
    print("\nCtrl+C detected, stopping the car.")
    fc.stop()
    sys.exit(0)

def scan_for_new_walls(car):
    dist = np.array(scan_dist(1))
    mapping = map_obj(dist, car.loc, car.direction)
    
    walls = list()
    for m in mapping:
        wall = (m[0]//10, m[1]//10)
        if wall in walls:
            continue
        walls.append(wall)
    
    return walls

def create_diagram(car):
    """ Scan and map sresult to A star grid """
    diagram = GridWithWeights(GRID_SIZE, GRID_SIZE)
    diagram.walls = scan_for_new_walls(car)
    
    return diagram

def update_diagram(diagram, new_walls):
    for nw in new_walls:
        if nw not in diagram.walls:
            diagram.walls.append(nw)

def create_route(diagram, now_loc, goal):
    came_from, cost_so_far = a_star_search(diagram, now_loc, goal)
    path = reconstruct_path(came_from, start=now_loc, goal=goal)
    draw_grid(diagram, point_to=came_from, start=now_loc, goal=goal)
    print()
    draw_grid(diagram, path=reconstruct_path(came_from, start=now_loc, goal=goal))
    
    return path

def auto_drive(start, goal, car, stop_sign_event, drive_done_event):
    """ Main driving logic """
    print(f"Car (A) starts at {start}, Destination (Z) at {goal}")
    diagram = create_diagram(car)
    path = create_route(diagram, start, goal)
    print(f"Route: {path}")
    cnt = 0
    while tuple(car.loc) != goal:
        cnt += 1
        print(f"cnt={cnt}")
        car.move_to(path[cnt])
        
        if tuple(car.loc) != path[cnt]:  # stop or turn
            print(f"Car did not move to {path[cnt]} from {car.loc}, scan again")
            # scan and add new walls
            new_walls = scan_for_new_walls(car)
            update_diagram(diagram, new_walls)
            print(f"Updated diagram")
            draw_grid(diagram, start=tuple(car.loc), goal=goal)
            cnt -= 1
            for w in new_walls:
                if w in path[cnt:]:
                    print(f"New wall {w} is in the route, rerouting")
                    path = create_route(diagram, tuple(car.loc), goal)
                    print(f"New route: {path}")
                    cnt = 0
                    break
            
    drive_done_event.set()
    print('Auto drive complete')

def object_detection_legacy(stop_sign_event, drive_done_event, flip_frame=False, show_camera=False):
    """Process responsible for object detection."""
    # Initialize camera
    cap = cv2.VideoCapture(CAMERA_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    # Visualization parameters
    row_size = 20  # pixels
    left_margin = 24  # pixels
    text_color = (0, 0, 255)  # red
    font_size = 1
    font_thickness = 1
    fps_avg_frame_count = 10

    # Initialize object detection model
    base_options = core.BaseOptions(
        file_name=OBJ_DETECT_MODEL, use_coral=ENABLE_EDGETPU, num_threads=NUM_THREADS)
    detection_options = processor.DetectionOptions(max_results=3, score_threshold=0.3)
    options = vision.ObjectDetectorOptions(
        base_options=base_options, detection_options=detection_options)
    detector = vision.ObjectDetector.create_from_options(options)

    counter, fps = 0, 0
    start_time = time.time()

    # Start object detection loop
    while True:
        if drive_done_event.is_set():
            print("drive_done_event")
            break
        
        success, frame = cap.read()
        if not success:
            sys.exit("ERROR: Unable to read from camera.")

        counter += 1
        
        # Flip the frame if needed
        if flip_frame:
            frame = cv2.rotate(frame, cv2.ROTATE_180)

        """I put this inside this if statement to have it process every other image to help with performance"""
        if counter % 2 == 0:
            # Convert BGR to RGB as required by the model
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Create a TensorImage object from the RGB image.
            input_tensor = vision.TensorImage.create_from_array(rgb_frame)

            # Run object detection estimation
            detection_result = detector.detect(input_tensor)

            # Draw keypoints and edges on input image
            frame = utils.visualize(frame, detection_result)

            # Check if a stop sign is detected
            # ============================================================= 
            # this is the big piece needed for detection, 
            # most of the rest of this script is not new
            # =============================================================
            if any([detection.categories[0].category_name == "stop sign" for detection in detection_result.detections]):
                print("Stop sign detected!")
                stop_sign_event.set()
                break

        # Calculate and display FPS
        if counter % fps_avg_frame_count == 0:
            end_time = time.time()
            fps = fps_avg_frame_count / (end_time - start_time)
            start_time = time.time()

        if show_camera:
            """I saw pretty decent improvement on fps while not displaying the frame livefeed ~4.5fps to 8.5fps"""
            fps_text = 'FPS = {:.1f}'.format(fps)
            #print(fps_text)
            text_location = (left_margin, row_size)
            cv2.putText(frame, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN, font_size, text_color, font_thickness)

            # Display the frame with detection
            cv2.imshow('Object Detection', frame)
            
        # Stop the program if the ESC key is pressed.
        if cv2.waitKey(1) == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


def object_detection_picamera(stop_sign_event, drive_done_event, flip_frame=False, show_camera=False):
    with open(LABEL_PATH, 'r') as f:
        labels = {i: line.strip() for i, line in enumerate(f.readlines())}

    interpreter = Interpreter(model_path=OBJ_DETECT_MODEL, num_threads=NUM_THREADS)
    interpreter.allocate_tensors()
    
    # Get input dimensions from the model
    input_details = interpreter.get_input_details()
    model_height = input_details[0]['shape'][1]
    model_width = input_details[0]['shape'][2]

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (FRAME_WIDTH, FRAME_HEIGHT)})
    picam2.configure(config)
    picam2.start()

    row_size = 20
    left_margin = 24  
    text_color = (0, 0, 255) 
    font_size = 1
    font_thickness = 1
    fps_avg_frame_count = 10

    counter, fps = 0, 0
    start_time = time.time()

    while True:
        if drive_done_event.is_set():
            print("drive_done_event")
            break
        
        frame = picam2.capture_array()
             
        counter += 1

        """I put this inside this if statement to have it process every other image to help with performance"""
        if counter % 2 == 0:
            # Convert BGRA to RGB as required by the model
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)  

            # Resize the image to the model's expected size
            resized_frame = cv2.resize(frame, (model_width, model_height))

            # Add batch dimension and convert data type
            input_tensor = np.expand_dims(resized_frame, axis=0).astype('uint8')

            # Set the Input Tensor
            interpreter.set_tensor(input_details[0]['index'], input_tensor)

            # Run object detection estimation
            interpreter.invoke()

            # Get Output Details 
            output_details = interpreter.get_output_details()
            
            boxes = interpreter.get_tensor(output_details[0]['index'])[0]  # shape: [1, None, 4]
            classes = interpreter.get_tensor(output_details[1]['index'])[0]  # shape: [1, None]
            scores = interpreter.get_tensor(output_details[2]['index'])[0]  # shape: [1, None]

            detection_result = picamera_utils.create_detections(boxes, classes, scores, labels)
            
            # Draw keypoints and edges on input image
            frame = picamera_utils.visualize(frame, detection_result)

            # Check if a stop sign is detected
            # ============================================================= 
            # this is the big piece needed for detection, 
            # most of the rest of this script is not new
            # =============================================================
            if any([detection.categories[0].category_name == "stop sign" for detection in detection_result]):
                print("Stop sign detected!")
                stop_sign_event.set()
                break

            # Calculate and display FPS
            if counter % fps_avg_frame_count == 0:
                end_time = time.time()
                fps = fps_avg_frame_count / (end_time - start_time)
                start_time = time.time()        

            fps_text = 'FPS = {:.1f}'.format(fps)
            #print(fps_text)
            
            if show_camera:
                text_location = (left_margin, row_size)
                cv2.putText(frame, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN, font_size, text_color, font_thickness)

                # Display the frame with detection
                cv2.imshow('Object Detection', frame)

        # Stop the program if the ESC key is pressed.
        if cv2.waitKey(1) == 27:
            break
    
    picam2.stop()
    cv2.destroyAllWindows()


class Picar:
    def __init__(self, loc, stop_sign_event):
        self.loc = loc
        self.direction = 'S'
        self.stop_sign_event = stop_sign_event
        
    def __enter__(self):
        return self
    
    def __exit__(self, *args):
        fc.stop()
        pass
    
    def __del__(self):
        self.__exit__()
        
    
    def move_to(self, dest):
        """ Move to dest, return early if stop sign or turning left/right """
        if self.stop_sign_event.is_set():
            print('Stop sign, wait for 3 seconds')
            fc.stop()
            time.sleep(3)
            self.stop_sign_event.clear()
            return
        
        print(f"at {self.loc}, moving to {dest}")
        direction, dist = get_direction_distance(self.loc, dest)
        
        if self.direction != direction:
            self.turn_to(direction)
            return

        print(f"move {dist}")
        fc.forward(FORWARD_SPEED)
        time.sleep(abs(dist)*FORWARD_TIME)
        fc.stop()
        
        if direction in ['W', 'E']:
            self.loc[0] += dist
        else:
            self.loc[1] += dist

        assert(self.loc[0] == dest[0] and self.loc[1] == dest[1])

    def turn_to(self, direction):
        if self.direction == direction: return
        
        cur_dir_idx = DIRECITONS.index(self.direction)

        if DIRECITONS[(cur_dir_idx+1)%4] == direction:
            print('turn right')
            fc.turn_right(TURN_SPEED)
            time.sleep(TURN_TIME)
            fc.stop()
        elif DIRECITONS[cur_dir_idx-1] == direction:
            print('turn left')
            fc.turn_left(TURN_SPEED)
            time.sleep(TURN_TIME)
            fc.stop()
            
        else:
            print(f"turn 180 degree, from {self.direction} to {direction}")
            fc.turn_left(TURN_SPEED)
            time.sleep(2*TURN_TIME)
            fc.stop()

        self.direction = direction
    

if __name__ == "__main__":

    goal = (GRID_SIZE-1, GRID_SIZE-1)
    
    try:
        if sys.argv[1]:
            goal = (0, GRID_SIZE-1)
    except:
        pass
    
    start = (GRID_SIZE//2, 0)

    signal.signal(signal.SIGINT, cleanup)

    # Create multiprocessing Events to
    stop_sign_event = mp.Event()  # object_detection_process signals stop sign to auto_drive_process
    drive_done_event = mp.Event()  # auto_drive_procees signals done driving to object_detection_process

    with Picar(list(start), stop_sign_event) as car:
        
        # Create the processes
        if MODE == 'legacy':
            object_detection_process = mp.Process(target=object_detection_legacy, args=(stop_sign_event, drive_done_event, False, False))
        elif MODE == 'picamera':
            object_detection_process = mp.Process(target=object_detection_picamera, args=(stop_sign_event, drive_done_event, False, False))
        else:
            raise Exception(f"Error: {MODE} mode is not supported")
        auto_drive_process = mp.Process(target=auto_drive, args=(start, goal, car, stop_sign_event, drive_done_event))
        
        # Start the processes
        object_detection_process.start()
        auto_drive_process.start()
        
        # Wait for both processes to complete
        object_detection_process.join()
        auto_drive_process.join()
