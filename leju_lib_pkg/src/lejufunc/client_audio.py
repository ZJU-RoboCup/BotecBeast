#!/usr/bin/env python
# coding=utf-8

import pyaudio
import wave
import os
import subprocess

CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000

audio_path = os.environ['HOME'] + '/Music/'


def record_audio(duration, filename):
	audio = pyaudio.PyAudio()

	# start recording
	stream = audio.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)

	frames = []
	for i in range(0, int(RATE / CHUNK * duration)):
		data = stream.read(CHUNK)
		frames.append(data)

	# stop recording
	stream.stop_stream()
	stream.close()
	audio.terminate()

	# save wav file
	file_path = '%s%s%s' % (audio_path, filename, '.wav')
	waveFile = wave.open(file_path, 'wb')
	waveFile.setnchannels(CHANNELS)
	waveFile.setsampwidth(audio.get_sample_size(FORMAT))
	waveFile.setframerate(RATE)
	waveFile.writeframes(b''.join(frames))
	waveFile.close()


def play_audio(filename):
	"""
	Play wav file

	Args:
	  filename: name of audio file
	"""

	file_path = '%s%s%s' % (audio_path, filename, '.wav')

	subprocess.check_output(['play', file_path])


def play_sound(sound_name):
	"""
	Play preset audio file

	Args:
	  sound_name: name of sound file, car / train / dog / cat
	"""

	file_dir = audio_path + "preset/"
	file_map = {
	    "car": "car_horn.mp3",
	    "train": "train_horn.mp3",
	    "dog": "dog.mp3",
	    "cat": "cat.mp3"
	}

	file_path = '%s%s' % (file_dir, file_map.get(sound_name))
	# print(file_path)

	subprocess.check_output(['play', file_path])


if __name__ == '__main__':
	# print("start record audio")
	# try:
	# 	record_audio(5, "test")
	# 	print("record finish")
	# except Exception as err:
	# 	print(err)
	try:
		# play_audio("test")
		play_sound("dog")
		play_sound("cat")
		play_sound("car")
		play_sound("train")
	except Exception as err:
		print(err)
