#!/usr/bin/env python3
# coding=utf-8

import os
import requests
import time
import json
from xml.etree import ElementTree

DEFAULT_FILEPATH = "/home/lemon/Music/text_to_speech.wav"


class TextToSpeech(object):
    def __init__(self, subscription_key):
        self.subscription_key = subscription_key
        self.access_token = None

    def get_token(self):
        fetch_token_url = "https://eastasia.api.cognitive.microsoft.com/sts/v1.0/issueToken"
        headers = {
		    'Ocp-Apim-Subscription-Key': self.subscription_key
        }
        response = requests.post(fetch_token_url, headers=headers)
        self.access_token = str(response.text)

    def choose_voice(self, language="CN", gender="male"):
        if language == "CN":
            target_language = "zh-CN"
            if gender == "male":
                target_voice = "Microsoft Server Speech Text to Speech Voice (zh-CN, Kangkang, Apollo)"
            else:
                target_voice = "Microsoft Server Speech Text to Speech Voice (zh-CN, Yaoyao, Apollo)"
        else:
            target_language = "en-US"
            if gender == "male":
                target_voice = "Microsoft Server Speech Text to Speech Voice (en-US, Guy24kRUS)"
            else:
                target_voice = "Microsoft Server Speech Text to Speech Voice (en-US, ZiraRUS)"

        return target_language, target_voice

    def save_audio(self, language, gender, text):
        self.get_token()

        target_text = str(text)
        target_language, target_voice = self.choose_voice(language, gender)

        base_url = "https://eastasia.tts.speech.microsoft.com/cognitiveservices/v1"
        headers = {
            'Authorization': 'Bearer ' + self.access_token,
            'Content-Type': 'application/ssml+xml',
            'X-Microsoft-OutputFormat': 'riff-24khz-16bit-mono-pcm',
            'User-Agent': 'AiSpeechASRTTS'
        }
        xml_body = ElementTree.Element('speak', version='1.0')
        xml_body.set('{http://www.w3.org/XML/1998/namespace}lang', target_language)
        voice = ElementTree.SubElement(xml_body, 'voice')
        voice.set('{http://www.w3.org/XML/1998/namespace}lang', target_language)
        voice.set('name', target_voice)
        voice.text = target_text
        body = ElementTree.tostring(xml_body)

        response = requests.post(base_url, headers=headers, data=body)
        if response.status_code == 200:
            with open(DEFAULT_FILEPATH, 'wb') as audio:
                audio.write(response.content)
                # print("\nStatus code: " + str(response.status_code) + "\nYour TTS is ready for playback.\n")
                filepath = DEFAULT_FILEPATH
                error_msg = ""
        else:
            # print("\nStatus code: " + str(response.status_code) +
            #     "\nSomething went wrong. Check your subscription key and headers.\n")
            filepath = ""
            error_msg = "Something went wrong. Check your subscription key and headers."

        return filepath, error_msg


if __name__ == '__main__':
    config_path = '/home/lemon/.lejuconfig/config.json'
    with open(config_path, 'r') as file:
        azure_config = json.load(file)
        speech_key = azure_config['speech_key']
    assert speech_key

    app = TextToSpeech(speech_key)
    try:
        # app.save_audio("CN", "male", "4是4，10是10，14是14，40是40")
        app.save_audio("EN", "female", "Are you from Room 250?")
    except Exception as err:
        print(err)
	
