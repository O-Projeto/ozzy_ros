import requests
from gtts import gTTS
from pydub import AudioSegment
from pydub.playback import play
import io

# Your Hugging Face API key
api_key = "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"
api_key2 = "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"

# The model you want to use
model = 'gpt2'

# The text you want to send
text = 'Quem foi Pedro Alvares Cabral'

# The API endpoint
api_url = f'https://api-inference.huggingface.co/models/{model}'

# The headers for the request
headers = {
    'Authorization': f'Bearer {api_key2}',
    'Content-Type': 'application/json'
}

# The payload for the request
payload = {
    'inputs': text
}

# Send the request
response = requests.post(api_url, headers=headers, json=payload)

# Get the response
response_text = response.json()

# Print the response
print(response_text[0]["generated_text"])

# Define the text to convert to audio
text = response_text[0]["generated_text"]

# Generate the speech using gTTS
tts = gTTS(text, lang='en')
audio_buffer = io.BytesIO()
tts.write_to_fp(audio_buffer)

# Seek to the start of the BytesIO object
audio_buffer.seek(0)

# Load the audio into pydub
audio_segment = AudioSegment.from_file(audio_buffer, format="mp3")

# Play the audio
play(audio_segment)
