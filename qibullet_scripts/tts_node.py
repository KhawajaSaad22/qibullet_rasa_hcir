#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import pyttsx3

class TextToSpeechNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("tts_node", anonymous=True)
        
        # Initialize pyttsx3
        self.engine = pyttsx3.init()
        voices = self.engine.getProperty('voices')
        for voice in voices:
            print(voice)
        self.engine.setProperty('voice', "English (Great Britain)")
        self.engine.setProperty("rate", 150)  # Set speaking rate
        self.engine.setProperty("volume", 0.9)  # Set volume

        # Subscribe to the TTS topic
        self.subscriber = rospy.Subscriber("/tts_commands", String, self.tts_callback)
        rospy.loginfo("TTS Node initialized and listening on /tts_commands")

    def tts_callback(self, msg):
        text = msg.data
        rospy.loginfo(f"Received text to speak: {text}")
        
        # Use pyttsx3 to convert text to speech
        self.engine.say(text)
        self.engine.runAndWait()

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        tts_node = TextToSpeechNode()
        tts_node.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TTS Node shutting down.")
