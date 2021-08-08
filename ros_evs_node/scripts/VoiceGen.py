#!/usr/bin/env python
# coding=utf-8

from EVS import EVS
from Player import Player
import time
from threading import Thread
import os
import random
import requests
import json


class VoiceGen:
    def __init__(self):
        with open("config.json", "r") as f:
            json_data = json.loads(f.read())
        self.client_id = json_data["client_id"]
        self.device_id = json_data["device_id"]
        self.evs = EVS(self.client_id,
                        self.device_id,
                        self.device_id,
                        voice_callback=self.voice_callback)
        with open("texts.txt", "r") as f:
            self.texts = f.readlines()
        self.request_id = 1
        self.tts_convert()

    def tts_convert(self):
        text = self.texts.pop(0)
        print("准备下载第 {} 个， 文字内容为： {}".format(self.request_id, text))
        self.evs.tts_convert("manual_" + str(self.request_id), text)
        self.request_id += 1

    def voice_callback(self, url):
        r = requests.get(url, stream=True, timeout=1)
        if r.status_code == 200:
            with open(str(self.request_id - 1) + ".mp3", 'wb') as f:
                for chunk in r.iter_content(chunk_size=1024):
                    f.write(chunk)
        if len(self.texts) != 0:
            self.tts_convert()
        else:
            print("所有文件下载完成...")
            self.evs.close()

if __name__ == "__main__":
    VoiceGen()