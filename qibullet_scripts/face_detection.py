import face_recognition
import cv2
import numpy as np
import os
from typing import List, Tuple
from collections import defaultdict
import time
import requests
import rospy
from std_msgs.msg import String, Bool

SHOW_VIDEO_FEED = True
MEMBERS_FOLDER_PATH = "ressources/images/members"
FACE_DISAPPEARANCE_BUFFER = 3.0
STATUS_UPDATE_INTERVAL = 5.0

class FaceDetection:
    def __init__(self):
        self.last_seen_timestamps = defaultdict(float)
        self.last_status_update = time.time()
        self.currently_present = set()
        self.unknown_count = 0
        self.last_reset_time = time.time()

        # Initialize ROS publishers
        self.face_pub = rospy.Publisher("/detected_faces", String, queue_size=10)
        self.reset_pub = rospy.Publisher("/reset_conversation", Bool, queue_size=10)
        self.tts_pub = rospy.Publisher("/tts_commands", String, queue_size=10)
    
    def speak(self, text: str) -> None:
        """Speak the given text using the TTS engine."""
        self.tts_engine.say(text)
        self.tts_engine.runAndWait()

    def update_face_presence(self, current_faces: List[str], current_time: float) -> None:
        current_face_set = set(current_faces)
        self.unknown_count = current_faces.count("Unknown")

        # Publish detected face names and send TTS text
        for face in current_faces:
            if face != "Unknown":
                self.face_pub.publish(face)
                rospy.loginfo(f"Published detected face: {face}")

            self.last_seen_timestamps[face] = current_time
            if face != "Unknown" and face not in self.currently_present:
                print(f"👋 {face} entered the frame")
                self.tts_pub.publish(f"Welcome {face}")  # Publish text for TTS
                self.currently_present.add(face)

        # Handle unknown faces
        if "Unknown" in current_faces:
            for _ in range(self.unknown_count):
                self.tts_pub.publish("Welcome Stranger")  # Publish text for unknown faces

        # Handle faces that have disappeared
        for face in list(self.currently_present):
            if face not in current_face_set:
                time_since_last_seen = current_time - self.last_seen_timestamps[face]
                if time_since_last_seen > FACE_DISAPPEARANCE_BUFFER:
                    print(f"👋 {face} left the frame")
                    self.currently_present.remove(face)

        # Trigger conversation reset if no faces detected for a period of time
        if not self.currently_present and current_time - self.last_reset_time > FACE_DISAPPEARANCE_BUFFER:
            rospy.loginfo("No faces detected. Publishing reset signal...")
            self.reset_pub.publish(True)
            self.last_reset_time = current_time

        if current_time - self.last_status_update >= STATUS_UPDATE_INTERVAL:
            self.print_status_update()
            self.last_status_update = current_time

    def print_status_update(self) -> None:
        known_faces = [face for face in self.currently_present if face != "Unknown"]
        print("==================\n")
        print(f"Known faces in frame: {', '.join(known_faces) if known_faces else 0}")
        print(f"Unknown faces in frame: {self.unknown_count}")
        print("==================\n")

def load_known_faces(folder_path: str) -> Tuple[List[np.ndarray], List[str]]:
    known_face_encodings = []
    known_face_names = []
    
    for filename in os.listdir(folder_path):
        if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
            name = os.path.splitext(filename)[0]
            image_path = os.path.join(folder_path, filename)
            
            face_image = face_recognition.load_image_file(image_path)
            face_encodings = face_recognition.face_encodings(face_image)
            
            if face_encodings:
                known_face_encodings.append(face_encodings[0])
                known_face_names.append(name)
                print(f"Loaded face encoding for: {name}")
            else:
                print(f"No face found in image: {filename}")
    
    return known_face_encodings, known_face_names

def process_video_frame(frame: np.ndarray, 
                        known_face_encodings: List[np.ndarray], 
                        known_face_names: List[str]) -> Tuple[np.ndarray, List[str]]:
    # Detect faces in the (RGB) frame
    face_locations = face_recognition.face_locations(frame)
    face_encodings = face_recognition.face_encodings(frame, face_locations)
    
    detected_faces = []
    
    for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
        matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
        name = "Unknown"

        if True in matches:
            first_match_index = matches.index(True)
            name = known_face_names[first_match_index]
        
        detected_faces.append(name)
        
        if SHOW_VIDEO_FEED:
            # Draw rectangle and text (remember frame is currently in RGB)
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, name, (left + 6, bottom - 6), font, 0.6, (255, 255, 255), 1)
    
    return frame, detected_faces


# def send_reset_intent():
#     url = "http://localhost:5005/webhooks/rest/webhook"  
#     payload = {"sender": "ros_system", "message": "/reset_conversation"}
#     response = requests.post(url, json=payload)
#     print(f"Reset intent sent to RASA: {response.status_code}")

# def send_slot_update():
#     url = "http://localhost:5005/webhooks/rest/webhook"
#     payload = {
#         "sender": "ros_system",
#         "events": [{"event": "slot", "name": "reset_required", "value": True}]
#     }
#     response = requests.post(url, json=payload)
#     print(f"Slot update sent to RASA: {response.status_code}")


def main():
    rospy.init_node("face_recognition_node", anonymous=True)

    video_capture = cv2.VideoCapture(4)
    if not video_capture.isOpened():
        print("Error: Could not open video capture device")
        return

    known_face_encodings, known_face_names = load_known_faces(MEMBERS_FOLDER_PATH)
    if not known_face_encodings:
        print("Error: No face encodings loaded")
        return

    face_tracker = FaceDetection()

    try:
        while not rospy.is_shutdown():
            ret, frame = video_capture.read()
            if not ret:
                print("Error: Could not read frame")
                break

            # Convert from BGR (OpenCV default) to RGB for face_recognition
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            processed_frame, current_faces = process_video_frame(
                frame, known_face_encodings, known_face_names
            )

            face_tracker.update_face_presence(current_faces, time.time())

            if SHOW_VIDEO_FEED:
                # Convert back to BGR for proper display in OpenCV
                display_frame = cv2.cvtColor(processed_frame, cv2.COLOR_RGB2BGR)
                cv2.imshow('Video', display_frame)

                # Allow OpenCV to process the GUI events
                key = cv2.waitKey(1)
                if key == ord('q') or key == 27:
                    break

    finally:
        video_capture.release()
        if SHOW_VIDEO_FEED:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
