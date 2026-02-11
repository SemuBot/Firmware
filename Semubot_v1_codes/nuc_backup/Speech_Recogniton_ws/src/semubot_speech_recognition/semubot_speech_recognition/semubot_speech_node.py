import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_interfaces.srv import Llama
from audio_common_msgs.msg import AudioData
import subprocess
from threading import Thread
import time
import select
import numpy as np
import wave
import os

# Change these variables if neccessary
conda_env = "transformer-tts"
#kiirkirj_command = "rec -t raw -r 16k -e signed -b 16 -c 1 - | play -t raw -e signed-integer -b 16 -c 1 -r 16k -"
#kiirkirj_command = "play -t raw -e signed-integer -b 16 -c 1 -r 16k -"
kiirkirj_command = "docker exec -i kiirkirjutaja python main.py -"
#kiirkirj_command = "rec -t raw -r 16k -e signed -b 16 -c 1 - | docker exec -i kiirkirjutaja python main.py -"



# For synthesis (assuming conda env is set up)
python_path = "/home/semubot/anaconda3/envs/transformer-tts/bin/python"
tts_path = "/home/semubot/text-to-speech"
conda_path = "/home/semubot/anaconda3/bin/activate" # Path to anaconda activate script
tts_file_path = f"{tts_path}/to_synthesize"

class SemubotSpeechNode(Node):
    def __init__(self):
        super().__init__('SemubotSpeechNode')
        self.is_busy = False
        self.output_detected = True
        
        self.wait_time = 7
        self.audio_buffer = []
        self.recording_counter = 0

        # create client to send service request to Llama node
        self.LLama_client = self.create_client(Llama, 'Llama_service')
        
        # Publisher for debug topic
        self.debug_publisher = self.create_publisher(String, 'Speech_topic_debug', 10)
        
        # Start Speech synthesis
        self.start_speech_recogintion(kiirkirj_command)
        
        
    def save_audio_to_wav(self, audio_data_chunks):
            """Saves a list of audio byte chunks to a WAV file."""
            self.recording_counter += 1
            path_with_tilde = f"~/recorded_utterance_{self.recording_counter}.wav"
            #filename = os.path.expanduser(path_with_tilde)
            # Combine the list of chunks into a single byte string
            audio_data = b''.join(audio_data_chunks)
            '''
            try:
                with wave.open(filename, 'wb') as wf:
                    wf.setnchannels(1)  # Mono
                    wf.setsampwidth(2)  # 16-bit audio -> 2 bytes
                    wf.setframerate(16000)  # 16kHz sample rate
                    wf.writeframes(audio_data)
                self.get_logger().info(f"Successfully saved audio to {filename}")
            except Exception as e:
                self.get_logger().error(f"Failed to save WAV file: {e}")
            '''
    def start_speech_recogintion(self, program_cmd):
        # Start kiirkirjutaja in container
        self.kiirkirjutaja_process = subprocess.Popen(program_cmd, shell=True, stdout=subprocess.PIPE, stdin = subprocess.PIPE)

        # Subscribe to audio stream
        self.microphone_subscriber = self.create_subscription(msg_type=AudioData, topic="audio", callback=self.audio_stream, qos_profile=10)
        self.mic_subscriber_start_time = None
        self.get_logger().info("started subscribing")
        
        self.kk_monitor_thread = Thread(target=self.monitor_program_output)
        self.kk_monitor_thread.start()
        self.get_logger().info("Started a non-blocking thread")
        
        #self.monitor_program_output()

    def audio_stream(self, msg):
        if self.is_busy:
            return
        input_bytes=bytes(msg.data)
        if self.started_rec:
            self.audio_buffer.append(input_bytes)
     
        self.kiirkirjutaja_process.stdin.write(input_bytes)
        
    # Get constant output of speech synthesis
    def monitor_program_output(self):
        self.get_logger().info("started waiting for kiirkirjutaja output")
        self.recognised_speech_buffer = bytes()
        last_input_time = time.time()
        self.started_rec  = False
         
        while True:
            readable,_,_ = select.select([self.kiirkirjutaja_process.stdout], [], [], 0.1)
            
            if readable:
                output = self.kiirkirjutaja_process.stdout.read(1)
                if output:
                    self.recognised_speech_buffer += output
                    last_input_time = time.time()
                    if not self.started_rec:
                        self.started_rec = True
                        self.get_logger().info('Started recognition')

            if self.started_rec and (time.time() - last_input_time > self.wait_time):
                self.get_logger().info('Detected pause, delegating LLM/TTS to a new thread.')
                self.is_busy = True
                if self.audio_buffer:
                    self.save_audio_to_wav(self.audio_buffer)

                # Reset the audio buffer for the next recording
                self.audio_buffer = []
                # Copy the buffer content to pass to the new thread
                text_to_process = self.recognised_speech_buffer.decode("utf-8")

                # Create and start the new thread for the blocking tasks
                processing_thread = Thread(target=self.run_llm_and_synthesis, args=(text_to_process,))
                processing_thread.start()
                
                # IMMEDIATELY reset and get ready for the next utterance
                self.get_logger().info('Ready for next speech recognition.')
                self.started_rec = False
                self.recognised_speech_buffer = bytes()
                last_input_time = time.time()
    def run_llm_and_synthesis(self, text_input):
        """
        This function runs in a separate thread and contains all the blocking calls.
        """
        self.get_logger().info('New thread started for processing.')
        # Now call the original blocking function with the provided text
        self.call_LLM_service(text_input)
        self.get_logger().info('LLM and synthesis thread finished.')


    # Modify call_LLM_service to accept the text as an argument
    def call_LLM_service(self, recognised_text):
        request = Llama.Request()
        request.input_string = recognised_text
        self.get_logger().info('Whole recognized text was: ' + request.input_string)
        
        while not self.LLama_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        future = self.LLama_client.call_async(request)
        # Note: You can't use rclpy.spin_until_future_complete in a non-node thread.
        # A better way is to use a synchronous call if available or handle futures differently.
        # For simplicity in this fix, we'll assume the async call works and we just need to get the result.
        # This part might need more advanced handling with ROS2 executors if issues arise.
        
        # Let's wait for the future to complete in a simple way
        while not future.done():
            time.sleep(0.1)

        if future.result() is not None:
            string_to_synthesize = future.result().output_string
            self.get_logger().info('Llama service call successful, string to synthesize:' + string_to_synthesize)
            self.Synthesize(string_to_synthesize, voice="mari")
        else:
            self.get_logger().warning('Llama service call failed')



    '''
    def Synthesize(self, string_to_synthesize, voice):
        # Write string to file
        with open(f"{tts_file_path}.txt", "w") as file:
            file.write(string_to_synthesize)
        
        # Activate Anaconda environmenent and construct the synthesis command
        activate_cmd = f"{conda_path} {conda_env} && cd {tts_path} && "
        program_cmd = f"{python_path} synthesizer.py --speaker {voice} --config config.yaml {tts_file_path}.txt {tts_file_path}.wav"
        full_cmd = f"{activate_cmd}{program_cmd}"
        
        # Run the synthesis command
        self.tts = subprocess.run(full_cmd, shell=True, universal_newlines=True)
        
        self.get_logger().info("Finished synthesizing")
        self.get_logger().info("Speaking True")

        try:
            # TEMPORARY, play audio using play command
            play_audio_cmd = f"/usr/bin/play {tts_file_path}.wav"
            self.play_audio = subprocess.run(play_audio_cmd, shell=True, universal_newlines=True)
        finally:
            self.get_logger().info("Speaking false")
            self.is_busy = False
    '''
    def flush_stt_buffer(self):
            """
            Reads and discards all pending data from the kiirkirjutaja process stdout pipe.
            This prevents stale, partially recognized text from carrying over.
            """
            self.get_logger().info("Flushing stale output from STT process...")
            while True:
                # Check if there is data to read with a zero-timeout
                readable, _, _ = select.select([self.kiirkirjutaja_process.stdout], [], [], 0.0)
                if readable:
                    # Read a chunk of data and discard it
                    self.kiirkirjutaja_process.stdout.read(2048)
                else:
                    # No more data in the pipe, buffer is flushed
                    break
            self.get_logger().info("STT buffer flushed.")
    # 
    def Synthesize(self, string_to_synthesize, voice="fi_FI-harri-medium"):
        voice="/home/semubot/Software/text-to-speech/piper/fi_FI-harri-medium"
        """
        Synthesizes and plays audio directly using the Piper command-line tool.

        Args:
            string_to_synthesize (str): The text to be spoken.
            voice (str): The name of the Piper model to use (e.g., 'fi_FI-harri-medium').
        """
        self.get_logger().info(f"Synthesizing and playing: '{string_to_synthesize}'")

        # Construct the command as a list of arguments.
        # This is safer than using a single string with shell=True, as it
        # handles special characters in the input string correctly.
        command = [
            "python3",
            "-m", "piper",
            "-m", voice,
            "--",  # A good practice to separate flags from the main argument
            string_to_synthesize
        ]
        
        try:
            # Run the Piper command. This is a blocking call that will wait
            # until the audio has finished playing.
            # We capture stderr to log any potential errors from Piper.
            subprocess.run(
                command, 
                check=True,          # Raises an exception if the command fails
                capture_output=True, # Captures stdout and stderr
                text=True            # Decodes stdout/stderr as text
            )
            self.get_logger().info("Finished playback.")

        except subprocess.CalledProcessError as e:
            # Log any errors from the command for easier debugging.
            self.get_logger().error(f"Piper command failed with exit code {e.returncode}:")
            self.get_logger().error(f"Piper stderr: {e.stderr}")
            
        finally:
            self.flush_stt_buffer()
            # CRITICAL: Always reset the busy state, whether playback succeeded or failed.
            # This allows the robot to listen again.
            self.get_logger().info("Clearing BUSY state, ready to listen.")
            self.is_busy = False
def main(args=None):
    rclpy.init(args=args)
    node = SemubotSpeechNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()