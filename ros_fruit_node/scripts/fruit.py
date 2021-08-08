# coding=utf-8
import os
import json
import requests
import base64

config_path = os.environ['HOME'] + '/.lejuconfig/config.json'
with open(config_path, 'r') as file:
    fruit_config = json.load(file)
    client_id = fruit_config['fruit_id']
    client_secret = fruit_config['fruit_secret']

assert client_id,client_secret


def fetch_token():
    token_url = 'https://aip.baidubce.com/oauth/2.0/token'
    host = token_url+"?grant_type=client_credentials&client_id="+client_id+"&client_secret="+client_secret
    token_response = requests.get(host)
    if token_response:
        result = token_response.json()
        if ('access_token' in result.keys() and 'scope' in result.keys()):
            if not 'brain_all_scope' in result['scope'].split(' '):
                print(('please ensure has check the ability'))
                exit()
            return result['access_token']
    else:
        print(('please overwrite the correct API_KEY and SECRET_KEY'))
        exit()


def read_file(image_path):
    with open(image_path, 'rb') as f:
        file_content = f.read()
    img = base64.b64encode(file_content)
    return img


def fruit_cognition(image_path):
    request_url = "https://aip.baidubce.com/rest/2.0/image-classify/v1/classify/ingredient"
    img = read_file(image_path)
    params = {"image":img,"top_num":1}
    access_token = fetch_token()
    request_url = request_url + "?access_token=" + access_token
    headers = {'content-type': 'application/x-www-form-urlencoded'}
    response = requests.post(request_url, data=params, headers=headers)
    if response:
        result_json = response.json()
        return result_json["result"][0]["name"].encode("utf-8")
    else:
        return "request error"


if __name__ == '__main__':
    result = fruit_cognition("./apple.jpeg")
    print(result)
