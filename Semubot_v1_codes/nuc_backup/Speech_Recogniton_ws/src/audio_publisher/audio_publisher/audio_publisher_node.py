#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData
import pyaudio

class SimpleAudioPublisher(Node):
    def __init__(self):
        super().__init__('simple_audio_publisher')
        
        # Parameters
        self.declare_parameter('device_name', 'default')  # Changed to 'pulse'
        self.declare_parameter('device_index', -1)
        self.declare_parameter('rate', 16000)
        self.declare_parameter('chunk', 1600)
        
        device_name = self.get_parameter('device_name').value
        device_index = self.get_parameter('device_index').value
        self.rate = self.get_parameter('rate').value
        self.chunk = self.get_parameter('chunk').value
        
        # Setup PyAudio
        self.audio = pyaudio.PyAudio()
        
        if device_index < 0:
            device_index = self.find_device_index(device_name)
        
        if device_index is None:
            self.get_logger().error(f'Device "{device_name}" not found')
            self.list_devices()
            raise RuntimeError('Audio device not found')
        
        info = self.audio.get_device_info_by_index(device_index)
        self.get_logger().info(f'Using device [{device_index}]: {info["name"]}')
        
        # Open stream
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.rate,
            input=True,
            input_device_index=device_index,
            frames_per_buffer=self.chunk
        )
        
        # Publisher
        self.publisher = self.create_publisher(AudioData, 'audio', 10)
        
        # Timer
        self.timer = self.create_timer(self.chunk / self.rate, self.publish_audio)
        
        self.get_logger().info('Audio publisher ready')
    
    def find_device_index(self, device_name):
        """Find device by exact or partial name match"""
        for i in range(self.audio.get_device_count()):
            try:
                info = self.audio.get_device_info_by_index(i)
                if info['maxInputChannels'] > 0:
                    # Exact match or contains match
                    if (device_name.lower() == info['name'].lower() or 
                        device_name.lower() in info['name'].lower()):
                        return i
            except:
                continue
        return None
    
    def list_devices(self):
        self.get_logger().info('Available input devices:')
        for i in range(self.audio.get_device_count()):
            try:
                info = self.audio.get_device_info_by_index(i)
                if info['maxInputChannels'] > 0:
                    self.get_logger().info(f"  [{i}] {info['name']} (channels: {info['maxInputChannels']})")
            except:
                continue
    
    def publish_audio(self):
        try:
            data = self.stream.read(self.chunk, exception_on_overflow=False)
            msg = AudioData()
            msg.data = data
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
    
    def __del__(self):
        if hasattr(self, 'stream'):
            self.stream.stop_stream()
            self.stream.close()
        if hasattr(self, 'audio'):
            self.audio.terminate()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SimpleAudioPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
