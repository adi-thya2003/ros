import rclpy
from std_msgs.msg import Int16MultiArray
import pyaudio
import numpy as np

def audio_playback_node():
    rclpy.init()
    node = rclpy.create_node('audio_playback_node')
    audio_subscriber = node.create_subscription(Int16MultiArray, 'audio_data', audio_data_callback, 10)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

def audio_data_callback(msg):
    audio_data = msg.data
    sample_rate = 16000  # Adjust this based on your audio data
    play_audio(audio_data, sample_rate)

def play_audio(audio_data, sample_rate):
    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16,
                    channels=1,
                    rate=sample_rate,
                    output=True)
    stream.write(audio_data.tobytes())
    stream.stop_stream()
    stream.close()
    p.terminate()

def main(args=None):
    audio_playback_node()

if __name__ == '__main__':
    main()
