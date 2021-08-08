# coding=utf-8
import os
from threading import Thread

class Player:
    def __init__(self):
        self.__is_playing = False
        self.__is_abort = False
        self.current_url = ""

    @property
    def is_playing(self):
        return self.__is_playing

    @property
    def is_abort(self):
        return self.__is_abort

    def __play(self):
        self.__is_abort = False
        self.__is_playing = True
        os.system("play -q " + self.current_url)
        self.__is_playing = False

    def play(self, url):
        if self.__is_playing == True:
            self.stop()
        self.current_url = url
        Thread(target=self.__play).start()

    def stop(self):
        if self.__is_playing == True:
            self.__is_playing = False
            self.__is_abort = True
            os.system("ps -aux | grep {} | awk '{{print($2}}' | xargs kill".format(self.current_url)))


if __name__ == "__main__":
    player = Player()
    player.play(
        "https://tts.iflyos.cn/live/e16389e1e92fbed17a71d31be7fef177bfe8c471.mp3")
    import time
    time.sleep(1)
    player.stop()
    print(player.is_playing)

