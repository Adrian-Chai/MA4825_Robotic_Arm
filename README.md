# MA4825_Robotic_Arm

## Speech-To-Text
Convert speech to text using Google Cloud API. Script takes in microphone input and output converted text.<br>

Google Cloud Authentication:<br>

Append to end of ~/.bashrc (Linux)<br>
```export GOOGLE_APPLICATION_CREDENTIALS="/path/to/tranquil-bazaar-362708-892a3c5098e8.json"```<br>
```source .bashrc```

Dependencies:<br>
(i) python3 <br>
```
cd /speech_to_text/
pip3 install -r speech_requirements.txt
```

Run Script:<br>
```
cd /speech_to_text/
python3 transcribe_streaming_mic.py`
```

Source: https://github.com/googleapis/python-speech<br>