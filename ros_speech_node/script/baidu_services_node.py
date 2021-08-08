#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import sys
import os
import collections
import requests
import json
import base64
import hashlib
import pyaudio
import time
import wave
import webrtcvad
from array import array
from struct import pack
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ros_speech_node.srv import *

FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK_DURATION_MS = 30
PADDING_DURATION_MS = 1500
CHUNK_SIZE = int(RATE * CHUNK_DURATION_MS / 1000)
CHUNK_BYTES = CHUNK_SIZE * 2
NUM_PADDING_CHUNKS = int(PADDING_DURATION_MS / CHUNK_DURATION_MS)
NUM_WINDOW_CHUNKS = int(400 / CHUNK_DURATION_MS)
NUM_WINDOW_CHUNKS_END = NUM_WINDOW_CHUNKS * 2

CONFIG_PATH = os.path.join(os.environ['HOME'], '.lejuconfig/config.json')
FILE_PATH = "/tmp/baidu_voice_%s.wav"
TTS_FILE_PATH = "/tmp/baidu_tts_%s.wav"
FACE_PATH = "/tmp/baidu_face.jpg"


class VoiceHandle:
    def __init__(self):
        self.__stoprecord = False
        self.__got_a_sentence = False
        self.__timeout = False
        self.file_path = None
        self.device_index = None
        self.chunk = CHUNK_SIZE
        self.channels = CHANNELS
        self.rate = RATE
        self.format = FORMAT
        self.num_padding_chunks = NUM_PADDING_CHUNKS
        self.num_window_chunks = NUM_WINDOW_CHUNKS
        self.num_window_chunks_end = NUM_WINDOW_CHUNKS_END
        self.audio = pyaudio.PyAudio()
        self.vad = webrtcvad.Vad(1)

        count = self.audio.get_device_count()
        for i in range(count):
            device = self.audio.get_device_info_by_index(i)
            if "sysdefault" == device["name"]:
                self.device_index = i if device["maxInputChannels"] > 0 else None
                break

        self.stream = self.audio.open(format=self.format,
                                      channels=self.channels,
                                      rate=self.rate,
                                      start=False,
                                      input=True, output=False,
                                      input_device_index=self.device_index,
                                      frames_per_buffer=self.chunk)

    def normalize(self, snd_data):
        MAXIMUM = 32767
        times = float(MAXIMUM) / max(abs(i) for i in snd_data)
        r = array('h')
        for i in snd_data:
            r.append(int(i * times))
        return r

    def record_to_file(self, path, data, sample_width):
        data = pack('<' + ('h' * len(data)), *data)
        wf = wave.open(path, 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(sample_width)
        wf.setframerate(self.rate)
        wf.writeframes(data)
        wf.close()
        return path

    def voicehandle(self):
        while not rospy.is_shutdown() and not self.__stoprecord:
            ring_buffer = collections.deque(maxlen=self.num_padding_chunks)
            triggered = False
            ring_buffer_flags = [0] * self.num_window_chunks
            ring_buffer_index = 0

            ring_buffer_flags_end = [0] * self.num_window_chunks_end
            ring_buffer_index_end = 0
            raw_data = array('h')
            index = 0
            start_point = 0
            start_time = time.time()
            print("* recording: ")
            self.stream.start_stream()
            while not self.__got_a_sentence and not rospy.is_shutdown():
                chunk = self.stream.read(self.chunk)
                raw_data.extend(array('h', chunk))
                index += self.chunk
                active = self.vad.is_speech(chunk, self.rate)
                sys.stdout.write('1' if active else '_')
                ring_buffer_flags[ring_buffer_index] = 1 if active else 0
                ring_buffer_index += 1
                ring_buffer_index %= self.num_window_chunks

                ring_buffer_flags_end[ring_buffer_index_end] = 1 if active else 0
                ring_buffer_index_end += 1
                ring_buffer_index_end %= self.num_window_chunks_end

                if not triggered:
                    if int(time.time() - start_time) > 15:
                        self.__timeout = True
                        self.stream.stop_stream()
                        break
                    ring_buffer.append(chunk)
                    num_voiced = sum(ring_buffer_flags)
                    if num_voiced > 0.8 * self.num_window_chunks:
                        sys.stdout.write(' Open ')
                        triggered = True
                        start_point = index - self.chunk * 20
                        ring_buffer.clear()
                else:
                    ring_buffer.append(chunk)
                    num_unvoiced = self.num_window_chunks_end - sum(ring_buffer_flags_end)
                    if num_unvoiced > 0.90 * self.num_window_chunks_end:
                        sys.stdout.write(' Close ')
                        triggered = False
                        self.__got_a_sentence = True
                        self.__stoprecord = True
                        self.stream.stop_stream()
                        print("* done recording")
                sys.stdout.flush()
            sys.stdout.write('\n')
            self.__got_a_sentence = False
            if self.__stoprecord:
                raw_data.reverse()
                for index in range(start_point):
                    raw_data.pop()
                raw_data.reverse()
                raw_data = self.normalize(raw_data)
                self.file_path = FILE_PATH % str(int(time.time()))
                self.record_to_file(self.file_path, raw_data, 2)
            if self.__timeout:
                self.file_path = None
                break
        self.stream.close()
        self.audio.terminate()


class BaiduService:
    def __init__(self):
        rospy.init_node('ros_speech_node')
        with open(CONFIG_PATH, "r") as f:
            json_data = json.load(f)
            self.__voice_key = json_data['baidu_voice']
            self.__face_key = json_data['baidu_face']
            self.__sign = json_data['leju_sign']
        self.__voice_token = None
        self.__face_token = None
        self.bridge = CvBridge()
        self.speech2text_server = rospy.Service('/ros_speech_node/speech_recognize_service', SpeechService,
                                                self.speech_to_text)
        self.text2speech_server = rospy.Service('/ros_speech_node/text_to_speech_service', TextService,
                                                self.text_to_speech)

        self.face_detect_srv = rospy.Service('ros_vision_node/face_detect', FaceDetectService, self.face_detect)
        self.face_verify_srv = rospy.Service('ros_vision_node/face_verify', FaceVerifyService, self.face_verify)
        rospy.spin()

    def get_token(self, api_key, sign):
        timestamp = str(int(round(time.time() * 1000)))
        checksum = hashlib.sha1((timestamp + sign).encode('utf-8')).hexdigest()
        url = "http://robanai.lejurobot.com:8090/PullToken?timestamp={}&checksum={}&api_key={}".format(timestamp,
                                                                                                       checksum,
                                                                                                       api_key)
        payload = {}
        headers = {}
        response = requests.get(url, headers=headers, data=payload)
        result = json.loads(response.text)
        return result["access_token"]

    def get_audio(self, file):
        with open(file, 'rb') as f:
            data = f.read()
        return data

    @property
    def voice_token(self):
        if not self.__voice_token:
            self.__voice_token = self.get_token(self.__voice_key, self.__sign)
        return self.__voice_token

    @property
    def face_token(self):
        if not self.__face_token:
            self.__face_token = self.get_token(self.__face_key, self.__sign)
        return self.__face_token

    def text_request(self, file_path):
        speech_data = self.get_audio(file_path)
        speech = base64.b64encode(speech_data).decode('utf-8')
        token = self.voice_token
        data = {
            'format': 'wav',
            'rate': '16000',
            'channel': 1,
            'cuid': 'cuid_robot',
            'len': len(speech_data),
            'speech': speech,
            'token': token,
            'dev_pid': 1537
        }
        url = 'http://robanai.lejurobot.com:8080/server_api'
        headers = {'Content-Type': 'application/json'}
        response = requests.post(url, json=data, headers=headers).json()
        if 'result' in response:
            return response['result'][0]
        else:
            rospy.logerr(str(response))
            return None

    def speech_to_text(self, req):
        voice_handle = VoiceHandle()
        voice_handle.voicehandle()
        if voice_handle.file_path:
            result = self.text_request(voice_handle.file_path)
            print(result)
            return result
        else:
            return None

    def text_to_speech(self, req):
        text = req.text
        language = 'zh'
        gender = 0 if req.gender == 'female' else 1
        token = self.voice_token
        data = {
            'tok': token,
            'tex': text,
            'cuid': 'cuid_robot',
            'lan': language,
            'ctp': 1,
            'spd': 5,
            'pit': 5,
            'vol': 15,
            'per': gender,
            'aue': 6
        }
        url = 'http://robanai.lejurobot.com:8089/text2audio'
        response = requests.post(url, data=data)
        file_path = TTS_FILE_PATH % str(int(time.time()))
        if response.status_code == 200:
            with open(file_path, 'wb') as f:
                f.write(response.content)
            err = ''
        else:
            file_path = ''
            err = 'Something went wrong'
        return err, file_path

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as err:
            print(err)
        else:
            return cv_image

    def file_open(self, image_path):
        with open(image_path, 'rb') as fp:
            image_base64 = base64.b64encode(fp.read())
        return image_base64

    def face_detect(self, req):
        try:
            if req.path == "":
                msg = rospy.wait_for_message("/camera/color/image_raw", Image, 1)
                current_frame = self.image_callback(msg)
                cv2.imwrite(FACE_PATH, current_frame)
                image_path = FACE_PATH
            else:
                image_path = req.path
            result = self.face_request(image_path)
        except Exception as err:
            rospy.logerr(err)
            return "Error"
        else:
            return result

    def face_request(self, image_path):
        token = self.face_token
        image_base64 = self.file_open(image_path)
        params = {
            "image": image_base64,
            "image_type": "BASE64",
            "face_field": "age,gender,emotion",
            "max_face_num": 10
        }
        url = "http://robanai.lejurobot.com:8010/rest/2.0/face/v3/detect"
        request_url = url + "?access_token=" + token
        headers = {'content-type': 'application/json'}
        response = requests.post(request_url, headers=headers, data=params).json()
        if 'result' in response:
            res = response.get('result')
            if res:
                result = json.dumps(res['face_list'])
                return result
            else:
                rospy.logerr(response.get('error_msg'))
                return "Error"
        else:
            rospy.logerr(str(response))
            return "Error"

    def face_verify(self, req):
        token = self.face_token
        faceid1 = req.faceId1
        faceid2 = req.faceId2
        params = json.dumps(
            [{"image": faceid1, "image_type": "FACE_TOKEN", "face_type": "LIVE", "quality_control": "LOW"},
             {"image": faceid2, "image_type": "FACE_TOKEN", "face_type": "LIVE", "quality_control": "LOW"}]
        )
        url = "http://robanai.lejurobot.com:8010/rest/2.0/face/v3/match"
        request_url = url + "?access_token=" + token
        headers = {'content-type': 'application/json'}
        response = requests.post(request_url, headers=headers, data=params).json()
        if 'result' in response:
            res = response.get('result')
            if res:
                result = res['score']
                return result
            else:
                rospy.logerr(response.get('error_msg'))
                return None
        else:
            rospy.logerr(str(response))
            return None


if __name__ == '__main__':
    BaiduService()
