#!/usr/bin/env python
# coding=utf-8

import rospy
from EVS import EVS
from Player import Player
from std_msgs.msg import String
import time
from threading import Thread
import rospkg
import os
import random
import requests
import json


REPLY_PATH = "/tmp/voice.mp3"


class Dialog:
    def __init__(self):
        rospy.init_node("ros_evs_node", anonymous=True)
        with open(rospy.get_param("~config_path"), "r") as f:
            json_data = json.loads(f.read())
            client_id = json_data["client_id"]
            device_id = json_data["device_id"]
        with open(rospkg.RosPack().get_path("ros_evs_node") + "/voice/text.json", 'r') as f:
            self.texts = json.loads(f.read())
        self.is_neterr = False
        self.record_time = 0
        self.request_id = 1
        token_path = rospy.get_param("~token_path")
        self.evs = EVS(client_id,
                       device_id,
                       device_id,
                       self.text_callback,
                       self.voice_callback,
                       self.second_callback,
                       self.neterr_callback,
                       token_path)
        self.player = Player()
        self.is_talking = False
        rospy.Subscriber("/micarrays/wakeup", String, self.wakup_callback)
        self.pub = rospy.Publisher('/voice_control_text', String, queue_size=10)
        rospy.spin()

    def wakup_callback(self, data):
        self.record_time = 0
        self.is_talking = False

        if self.player.is_playing:
            self.player.stop()

        if self.evs.is_recording:
            self.evs.cancel_record()

        self.play_net_voice("wakeup")
        Thread(target=self.__wait_play_to_start).start()

    def start_record(self):
        self.is_neterr = False
        self.evs.start_record("manual_" + str(self.request_id))
        self.request_id += 1

    def tts_convert(self, text):
        self.is_neterr = False
        self.evs.tts_convert("manual_" + str(self.request_id), text)
        self.request_id += 1

    def second_callback(self):
        if self.is_talking:
            return
        self.record_time += 1
        if self.record_time == 5:       # 5秒未回应
            self.evs.stop_record()
            self.five_time_out_play()
        elif self.record_time == 10:    # 10秒未回应
            self.record_time = 0        # 10秒未回应后重置
            self.evs.stop_record()
            self.ten_time_out_play()

    def neterr_callback(self):
        self.is_neterr = True
        if self.player.is_playing:
            return
        self.play_local_voice("neterr_")


    def play_local_voice(self, local):
        """
        Args:
            - local:
                "wakeup_", "5slater_", "10slater_", "neterr_"
        """
        path = rospkg.RosPack().get_path("ros_evs_node") + "/voice/"
        files = [name for name in os.listdir(path) if name.startswith(local)]
        self.player.play(path + random.choice(files))

    def play_net_voice(self, item):
        """
        Args:
            - item:
                "wakeup", "5slater", "10slater"
        """
        replytexts= self.texts[item]
        random_text = random.choice(replytexts)
        self.tts_convert(random_text)


    def five_time_out_play(self):
        self.play_net_voice("5slater")
        Thread(target=self.__wait_play_to_start).start()

    def __wait_play_to_start(self):
        while not self.player.is_playing:
            if self.is_neterr:
                return
            time.sleep(0.01)
        while self.player.is_playing:
            time.sleep(0.01)
        if self.player.is_abort:
            return
        if not self.is_neterr:
            self.start_record()

    def ten_time_out_play(self):
        self.play_net_voice("10slater")

    def voice_callback(self, url, text):
        try:
            r = requests.get(url, stream=True, timeout=1)
            if r.status_code == 200:
                with open(REPLY_PATH, 'wb') as f:
                    for chunk in r.iter_content(chunk_size=1024):
                        f.write(chunk)
                self.evs.stop_record()
                self.player.play(REPLY_PATH)
                self.pub.publish(text)
            else:
                self.neterr_callback()
        except:
            self.neterr_callback()


    def text_callback(self, text, is_last):
        self.is_talking = True
        self.record_time = 0
        print(("\033[32mis_last: \033[0m" if is_last else "\033[31mno_last: \033[0m") + text)


if __name__ == "__main__":
    Dialog()
