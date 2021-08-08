#!/usr/bin/env python
# coding=utf-8

import rospy
import subprocess

from ros_speech_node.srv import SpeechService, TextService


def punctuation_filter(text):
	punctuation_list = ["，", "。", "？", "！", "：", "“", "”", "《", "》", "；", "、"]
	after_process_text = text

	for p in punctuation_list:
		after_process_text = after_process_text.replace(p, "")

	return after_process_text


def speech_to_text(text):
	"""
	Transfer speech to text, match target words / sentence

	Return:
	  True / False
	"""

	rospy.wait_for_service("/ros_speech_node/speech_recognize_service", timeout=2)
	speech_client = rospy.ServiceProxy("/ros_speech_node/speech_recognize_service", SpeechService)
	result = speech_client().result

	result = punctuation_filter(result)
	target_text = punctuation_filter(text)

	return result == target_text


def text_to_speech(language, gender, text):
    """
    Transfer text to speech

    Args:
      language: target language, CN / EN
      gender: target gender, male / female
      text: target text

    No return, play the audio file instead
    """

    rospy.wait_for_service('/ros_speech_node/text_to_speech_service', timeout=2)
    text_client = rospy.ServiceProxy('/ros_speech_node/text_to_speech_service', TextService)
    result = text_client(language, gender, text)
    # print(result)

    if result.errorMsg != "":
        raise Exception(result.errorMsg)
    else:
        subprocess.check_output(['play', result.filepath])


if __name__ == '__main__':
	rospy.init_node('speech_test', anonymous=True)

	try:
		result = speech_to_text("你，叫。什么名字？？？？？")
		print(result)
	except Exception as err:
		rospy.logerr(err)
