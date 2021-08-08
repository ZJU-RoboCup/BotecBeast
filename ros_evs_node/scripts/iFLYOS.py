# coding=utf-8
import json
import requests
import sys
if sys.version_info.major == 2:
    from urllib import quote
else:
    from urllib.parse import quote
import time
from threading import Thread
import os


class iFLYOS:
    def __init__(self, client_id, device_id, thirdparty_id, token_path, neterr_callback=None):
        self.__client_id = client_id
        self.__device_id = device_id
        self.__thirdparty_id = thirdparty_id
        self.__token_path = token_path
        self.__token = None
        self.__dead_time = 0
        self.__read_token()

    def __read_token(self):
        file_text = ""
        if self.__token_path is not None and os.path.exists(self.__token_path):
            with open(self.__token_path, "r+") as f:
                file_text = f.read()

        if file_text == "":
            # 1. apply auth
            result = self.__apply_auth()
            if not result:
                return
            user_code, device_code, expires_in = result

            # 2. get token
            get_token_thread = Thread(target=self.__get_token,
                                      args=[device_code, expires_in])
            get_token_thread.start()
            # 3. implicit auth
            self.__set_auth(user_code)
            get_token_thread.join()
            # 4. save
            self.__save_token()
        else:
            file_json = json.loads(file_text)
            self.__set_token(file_json)

    def __judge_token(self):
        if time.time() < self.__dead_time:
            return
        if not self.__token:
            self.__read_token()
        else:
            # refresh token
            url = "https://auth.iflyos.cn/oauth/ivs/token"
            data = {
                "grant_type": "refresh_token",
                "refresh_token": self.__refresh_token
            }
            # result = requests.post(url, json=data).json()
            result = self.__request(url, data)
            if not result:
                return
            elif "error" in result:
                pass
            else:
                self.__set_token(result)
                self.__save_token()

    @property
    def access_token(self):
        self.__judge_token()
        return self.__token

    def __apply_auth(self):
        scope_data = quote('{"user_ivs_all": {"device_id": "'
                           + self.__device_id + '"}}')

        url = "https://auth.iflyos.cn/oauth/ivs/device_code?" \
              + "client_id={}&scope=user_ivs_all user_device_text_in&scope_data={}".format(self.__client_id, scope_data)

        result = self.__request(url)
        if result:
            return result["user_code"], result["device_code"], result["expires_in"]

    def __get_token(self, device_code, expires_in):
        url = "https://auth.iflyos.cn/oauth/ivs/token"
        data = {
            "client_id": self.__client_id,
            "grant_type": "urn:ietf:params:oauth:grant-type:device_code",
            "device_code": device_code
        }
        stop_time = time.time() + expires_in
        while time.time() < stop_time:
            result = self.__request(url, data)
            if not result:
                return
            elif "error" in result:
                pass
            else:
                self.__set_token(result)
                return

    def __set_token(self, json_data):
        self.__token = json_data["access_token"]
        self.__refresh_token = json_data["refresh_token"]
        if "created_at" in json_data:
            self.__dead_time = json_data["created_at"] + \
                json_data["expires_in"]
        else:
            self.__dead_time = json_data["dead_time"]

    def __set_auth(self, user_code):
        url = "https://api.iflyos.cn/thirdparty/general/auth"
        data = {
            "client_id": self.__client_id,
            "thirdparty_id": self.__thirdparty_id,
            "user_code": user_code
        }
        self.__request(url, data)

    def __save_token(self):
        if not self.__token_path or not self.__token:
            return

        data = {
            "access_token": self.__token,
            "refresh_token": self.__refresh_token,
            "dead_time": self.__dead_time
        }
        with open(self.__token_path, "w+") as f:
            f.write(json.dumps(data, indent=4))

    def __request(self, url, data=None):
        try:
            result = requests.post(url, json=data).json()
            return result
        except Exception as err:
            print(err)
            return



if __name__ == "__main__":
    client_id = "xxxxxx" # TODO: add client_id
    device_id = "roban001"
    i = iFLYOS(client_id, device_id, device_id, "./iflyos_token.json")
    print(i.access_token)
