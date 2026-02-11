import numpy as np

# Specify the audio file path
audio_file = 'rec.raw'

# Read raw audio data
with open(audio_file, 'rb') as f:
    # Read all the data from the file
    raw_data = f.read()

# Convert raw binary data to numpy array of 16-bit signed integers
audio_array = np.frombuffer(raw_data, dtype=np.int8)

# Print the length of the audio array (number of samples)
print(audio_array[:100])