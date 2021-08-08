#! /usr/bin/python
import os
import json
import cognitive_face as CF

import httplib, urllib, base64

# read azure key from config
config_path = os.environ['HOME'] + '/.lejuconfig/config.json'
with open(config_path, 'r') as file:
	azure_config = json.load(file)
	subscription_key = azure_config['face_key']
assert subscription_key
CF.Key.set(subscription_key)

def face_cognition(image_path):
	BASE_URL = 'https://southeastasia.api.cognitive.microsoft.com/face/v1.0/'
	CF.BaseUrl.set(BASE_URL)

	result = CF.face.detect(image_path, attributes='age,gender,emotion')
	return json.dumps(result)


def face_verify(faceId1, faceId2):
	# print("params are: ", faceId1, faceId2)

	headers = {
	    # Request headers
	    'Content-Type': 'application/json',
	    'Ocp-Apim-Subscription-Key': subscription_key,
	}
	params = urllib.urlencode({})
	body = json.dumps({
	    "faceId1": faceId1,
	    "faceId2": faceId2
	})

	conn = httplib.HTTPSConnection('southeastasia.api.cognitive.microsoft.com')
	conn.request("POST", "/face/v1.0/verify?%s" % params, body, headers)
	response = conn.getresponse()
	data = json.loads(response.read())
	# print(data)
	conn.close()

	if data.get('isIdentical'):
		result = 1
	else:
		result = 0

	return result, data.get('confidence')


if __name__ == '__main__':
	# result = face_cognition()
	# print(result)

	face_verify("5b200e38-b16b-4727-a0d5-c09e03940c6c", "108f54e3-af8e-46d2-afde-39140b464073")