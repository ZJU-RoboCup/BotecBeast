# -*- encoding:utf-8 -*-
import websocket
from iFLYOS import iFLYOS
from Recorder import Recorder
import json
from enum import Enum
import pyaudio
from threading import Thread
import time
from std_msgs.msg import UInt8MultiArray
import rospy

class RequestType(Enum):
    AUDIO_IN = "recognizer.audio_in"
    TEXT_IN = "audio_player.tts.text_in"

    def request_json(self, request_id, access_token, device_id, **kwargs):
        common_json = self.__common_json(access_token, device_id)
        common_json["iflyos_request"] = {
            "header": {
                "name": self.value,
                "request_id": request_id
            },
        }
        if self == self.AUDIO_IN:
            common_json["iflyos_request"]["payload"] = {
                "enable_vad": True,
                "profile": "CLOSE_TALK",
                "format": "AUDIO_L16_RATE_16000_CHANNELS_1",
            }
        elif self == self.TEXT_IN:
            common_json["iflyos_request"]["payload"] = {
                "text": kwargs["text"]
            }
        return json.dumps(common_json)

    def __common_json(self, access_token, device_id):
        return {
            "iflyos_header": {
                "authorization": "Bearer " + access_token,
                "device": {
                    "device_id": device_id,
                    "platform": {
                        "name": "linux",
                        "version": "1604"
                    }
                },
            },
            "iflyos_context": {
                "system": {
                    "version": "1.3"
                },
                "recognizer": {
                    "version": "1.1"
                },
                "speaker": {
                    "version": "1.0",
                    "volume": 10,
                },
                "audio_player": {
                    "version": "1.2",
                    "playback": {
                        "state": "IDLE",
                    }
                },
            },
        }


class ResponseType(Enum):
    AUDIO_OUT = "audio_player.audio_out"
    STOP_CAPTURE = "recognizer.stop_capture"
    INTERMEDIATE_TEXT = "recognizer.intermediate_text"

    @classmethod
    def get_result(cls, data):
        """ Parsing response data

        Returns:
        if not None,
        return (is_last, result_type, value)
            - result_type:          ResponseType
            - value:
                AUDIO_OUT:          mp3 url
                STOP_CAPTURE:       None
                INTERMEDIATE_TEXT:  (text, is_last)
        """
        iflyos_meta = data["iflyos_meta"]
        is_last = iflyos_meta["is_last"]
        if len(data["iflyos_responses"]) == 0:
            return is_last, None, None
        response = data["iflyos_responses"][-1]
        try:
            result_type = ResponseType(response["header"]["name"])
        except:
            return None
        if result_type == ResponseType.AUDIO_OUT:
            value = response["payload"]["secure_url"], response["payload"]["metadata"]["text"] 
        elif result_type == ResponseType.STOP_CAPTURE:
            value = None
        elif result_type == ResponseType.INTERMEDIATE_TEXT:
            value = (response["payload"]["text"],
                     response["payload"]["is_last"])
        return is_last, result_type, value


class EVS:
    def __init__(self,
                 client_id,
                 device_id,
                 thirdparty_id,
                 text_callback=None,
                 voice_callback=None,
                 second_callback=None,
                 neterr_callback=None,
                 token_path=None):
        """
        Args:
            - client_id:
            - device_id:
            - thirdparty_id:
            - text_callback:  Real-time recorded text
                eg. `def text_callback(text, is_last):`
            - voice_callback:  Reply voice secure url
                eg. `def voice_callback(url)`
            - second_callback:  Called once a second while recording
                eg. `def second_callback()`
            - token_path: save token path, if None, don't save.
        """

        self.__iflyos = iFLYOS(client_id, device_id, device_id, token_path)
        self.__device_id = device_id
        self.__ws = None
        self.__ws_is_err = True
        self.__ws_is_opening = False
        self.__recorder_ = None
        self.__is_recording = False
        self.__current_request_id = ""
        self.text_callback = text_callback
        self.voice_callback = voice_callback
        self.second_callback = second_callback
        self.neterr_callback = neterr_callback

    @property
    def __recorder(self):
        if self.__recorder_ == None:
            self.__recorder_ = Recorder(640)
        return self.__recorder_
    
    def close(self):
        self.__ws.close()

    @property
    def ws(self):
        if self.__ws_is_err:
            access_token = self.__iflyos.access_token
            if access_token:
                url = "wss://ivs.iflyos.cn/embedded/v1?token={}&device_id={}".format(
                    access_token, self.__device_id)
                self.__ws = websocket.WebSocketApp(url,
                                                   on_close=lambda ws: self.__ws_on_close(ws),
                                                   on_message=lambda ws, msg: self.__ws_on_message(ws, msg),
                                                   on_error=lambda ws, err: self.__ws_on_error(ws, err),
                                                   on_open=lambda ws: self.__ws_on_open(ws))
                self.__ws_is_opening = True
                Thread(target=self.__ws.run_forever, 
                       kwargs={"ping_interval": 5, "ping_timeout": 2}).start()

        return self.__ws

    def __ws_on_message(self, ws, msg):
        json_data = json.loads(msg)
        print(json_data)
        if "request_id" in json_data["iflyos_meta"]:
            if json_data["iflyos_meta"]["request_id"] != self.__current_request_id:
                return
        result = ResponseType.get_result(json_data)
        if result is None:
            return
        _, result_type, value = result
        if result_type == ResponseType.STOP_CAPTURE:
            self.stop_record()
        elif result_type == ResponseType.INTERMEDIATE_TEXT:
            if self.text_callback:
                self.text_callback(value[0], value[1])
        elif result_type == ResponseType.AUDIO_OUT:
            if self.voice_callback:
                self.voice_callback(value[0], value[1])

    def __ws_on_error(self, ws, err):
        self.__ws.close()
        self.neterr_callback()

    def __ws_on_close(self, ws):
        self.__ws_is_opening = False
        self.__ws_is_err = True
        self.__ws = None

    def __ws_on_open(self, ws):
        self.__ws_is_opening = False
        self.__ws_is_err = False

    @property
    def is_recording(self):
        return self.__is_recording

    def __request(self, request_id, json_lambda):
        access_token = self.__iflyos.access_token
        self.__current_request_id = request_id
        if self.ws:
            while self.__ws_is_opening:
                time.sleep(0.01)
            if not self.__ws_is_err:
                json_data = json_lambda(access_token)
                self.ws.send(json_data)
                return True

        if self.neterr_callback:
            self.neterr_callback()
        return False

    def start_record(self, request_id):

        result = self.__request(request_id, lambda access_token: RequestType.AUDIO_IN.request_json(request_id,
                                                                                                   access_token,
                                                                                                   self.__device_id))
        if not result:
            return

        self.__is_recording = True
        if rospy.get_param("is_hlw_mic", False):
            Thread(target=self.__receiving_audio).start()
        else: 
            Thread(target=self.__start_record).start()

    def tts_convert(self, request_id, text):
        self.__request(request_id, lambda access_token: RequestType.TEXT_IN.request_json(request_id,
                                                                                         access_token,
                                                                                         self.__device_id,
                                                                                         text=text))

    def __receiving_audio(self):
        self.left_time = time.time()
        def subCallback(data):
            if self.second_callback:
                if time.time() - self.left_time > 1:
                    self.left_time = time.time()
                    self.second_callback()
            if self.__is_recording:
                if not self.__ws_is_err:
                    self.ws.send(data.data, 0x2)
                else:
                    if self.neterr_callback:
                        self.neterr_callback()

        sub = rospy.Subscriber("/audio/stream", UInt8MultiArray, subCallback)
        while self.__is_recording and not self.__ws_is_err and not rospy.is_shutdown():
            time.sleep(0.01)
        sub.unregister()
        
    def __start_record(self):
        left_time = time.time()
        for data in self.__recorder.read():
            if self.second_callback:
                if time.time() - left_time > 1:
                    left_time = time.time()
                    self.second_callback()

            if self.__is_recording:
                if not self.__ws_is_err:
                    self.ws.send(data, 0x2)
                else:
                    if self.neterr_callback:
                        self.neterr_callback()
                        break
            else:
                break

    def cancel_record(self):
        if self.__ws:
            self.__ws.send("__CANCEL__")
        self.__is_recording = False

    def stop_record(self):
        if self.__ws:
            self.__ws.send("__END__")
        self.__is_recording = False


if __name__ == "__main__":
    client_id = "xxx"  # TODO：添加真实的 client_id
    device_id = "roban001"

    def text_callback(text, is_last):
        print(u"text callback: {}, {}".format(text, is_last))

    def voice_callback(url):
        print(u"voice_callback: " + url)

    def second_callback():
        print("1秒回调")

    evs = EVS(client_id,
              device_id,
              device_id,
              text_callback,
              voice_callback,
              second_callback,
              "./iflyos_token.json")
    request_id = "lemon_123456"
    evs.start_record(request_id)
