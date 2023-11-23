import rclpy
from vosk import Model, KaldiRecognizer
from std_msgs.msg import Int16MultiArray, String
import spacy
import logging

logging.getLogger().setLevel(logging.WARNING)

nlp = spacy.load("en_core_web_lg")  # Load the English language model
# Define your target keywords
keywords = ["circle", "front","back", "left", "right", "square","stop"]

def recognize_speech(audio_data, model):
    recognizer = KaldiRecognizer(model, 16000)
    recognizer.AcceptWaveform(audio_data.data.tobytes())
    result = recognizer.Result()
    return result

def audio_data_callback(msg, text_publisher, model):
    recognized_text = recognize_speech(msg, model)
    recognized_text = recognized_text.lower()  # Convert to lowercase for better matching
    detected_keywords = []
    print("actual Text:", recognized_text)  # Add this line to print the recognized text
    # Process the recognized text using spaCy
    doc = nlp(recognized_text)

    # Check if any of the recognized words are close to the target keywords
    for token in doc:
        for keyword in keywords:
            if token.similarity(nlp(keyword)) > 0.6:  # Adjust similarity threshold as needed
                detected_keywords.append(keyword)
                break

    if detected_keywords:
        text_publisher.publish(String(data=" ".join(detected_keywords)))

def recognized_text_callback(msg):
    recognized_text = msg.data
    logging.info("Recognized Text: %s", recognized_text)



def main():
    rclpy.init()
    node = rclpy.create_node('speech_recognition_node')
    
    text_publisher = node.create_publisher(String, 'recognized_text', 10)

    model_path = '/home/adithya/ros2_lab/src/vosk-model-en-us-0.22'
    model = Model(model_path)

    audio_subscriber = node.create_subscription(Int16MultiArray, 'audio_data', lambda msg: audio_data_callback(msg, text_publisher, model), 10)
    text_subscriber = node.create_subscription(String, 'recognized_text', recognized_text_callback, 10)

    logging.getLogger().setLevel(logging.INFO)  # Change logging level to INFO

    print("ROS 2 speech recognition node is running.")

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

