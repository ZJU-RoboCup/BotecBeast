#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import rospy

import azure.cognitiveservices.speech as speechsdk

import tts

from ros_speech_node.srv import *

config_path = '/home/lemon/.lejuconfig/config.json'
with open(config_path, 'r') as file:
	azure_config = json.load(file)
	speech_key = azure_config['speech_key']
assert speech_key
service_region = "eastasia"
speech_language = "zh-CN"
speech_config = speechsdk.SpeechConfig(subscription=speech_key, region=service_region, speech_recognition_language=speech_language)

text_client = tts.TextToSpeech(speech_key)


def process_punctuation(text):
    punctuation_list = ["，", "。", "？", "！", "：", "“", "”", "《", "》", "；", "、"]
    after_process_text = text

    for p in punctuation_list:
        after_process_text = after_process_text.replace(p, "")

    return after_process_text


def speech_to_text(req):
    speech_recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config)
    # print("Say something...")

    result = speech_recognizer.recognize_once()
    text = ""

    if result.reason == speechsdk.ResultReason.RecognizedSpeech:
        rospy.loginfo("Recognized: {}".format(result.text))
        text = result.text
    elif result.reason == speechsdk.ResultReason.NoMatch:
    	rospy.logerr("No speech could be recognized: {}".format(result.no_match_details))
    elif result.reason == speechsdk.ResultReason.Canceled:
    	cancellation_details = result.cancellation_details
    	rospy.logerr("Speech Recognition canceled: {}".format(cancellation_details.reason))
    	if cancellation_details.reason == speechsdk.CancellationReason.Error:
    		rospy.logerr("Error details: {}".format(cancellation_details.error_details))

    if text != "":
        text = process_punctuation(text)

    return SpeechServiceResponse(text)


def text_to_speech(req):
    target_language = req.language.upper()
    target_gender = req.gender.lower()
    target_text = req.text

    try:
        filepath, error_msg = text_client.save_audio(target_language, target_gender, target_text)
    except Exception as err:
        filepath = ""
        error_msg = str(err)

    return TextServiceResponse(error_msg, filepath)


def main():
    rospy.init_node('ros_speech_node')

    speech_recognize_server = rospy.Service('/ros_speech_node/speech_recognize_service', SpeechService, speech_to_text)
    text_to_speech_server = rospy.Service('/ros_speech_node/text_to_speech_service', TextService, text_to_speech)
    print("ready to process speech request")

    rospy.spin()


if __name__ == '__main__':
	main()


