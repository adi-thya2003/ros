import rclpy
from std_msgs.msg import Int16MultiArray
import pyaudio
import numpy as np
from pydub import AudioSegment

def audio_capture_node():
    rclpy.init()
    node = rclpy.create_node('audio_capture_node')
    audio_publisher = node.create_publisher(Int16MultiArray, 'audio_data', 10)

    # Audio parameters
    sample_rate = 16000
    num_channels = 1
    chunk_size = 32768

    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16,
                    channels=num_channels,
                    rate=sample_rate,
                    input=True,
                    frames_per_buffer=chunk_size)

    while rclpy.ok():
        audio_data = np.frombuffer(stream.read(chunk_size), dtype=np.int16)

        # Create an AudioSegment from the audio data
        audio_segment = AudioSegment(audio_data.tobytes(),
                                     frame_rate=sample_rate,
                                     sample_width=audio_data.itemsize,
                                     channels=num_channels)

        # Apply high-pass filter for noise reduction
        audio_segment = audio_segment.high_pass_filter(5000)  # Adjust the cutoff frequency as needed

        # Convert back to numpy array
        denoised_audio = np.array(audio_segment.get_array_of_samples(), dtype=np.int16)

        # Create a ROS message with the denoised audio
        audio_msg = Int16MultiArray(data=denoised_audio)
        audio_publisher.publish(audio_msg)

    stream.stop_stream()
    stream.close()
    p.terminate()

    node.destroy_node()
    rclpy.shutdown()

def main(args=None):
    audio_capture_node()

if __name__ == '__main__':
    main()
