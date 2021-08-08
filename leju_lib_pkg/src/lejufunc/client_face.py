#!/usr/bin/env python
import rospy
import rospkg
import json
import time
from client_logger import *
from ros_speech_node.srv import FaceDetectService, FaceVerifyService

FACE_PATH = rospkg.RosPack().get_path("ros_actions_node") + "/scripts/faces/"


def face_service(path=None):
    if path is None:
        image_path = ""
    else:
        image_path = path
    try:
        rospy.wait_for_service('ros_vision_node/face_detect', timeout=2)
        face_recognition = rospy.ServiceProxy('ros_vision_node/face_detect', FaceDetectService)
        response = face_recognition(image_path)
        if response.result == "Error":
            return []
        else:
            return json.loads(response.result)
    except Exception as err:
        serror(err)
        return []


def face_filter(face_list):
    target_face = face_list[0]
    target_face_size = target_face.get('location').get('width') * target_face.get('location').get('height')

    if len(face_list) > 1:
        for face in face_list:
            current_face_size = face.get('location').get('width') * face.get('location').get('height')
            if current_face_size > target_face_size:
                target_face = face
                target_face_size = current_face_size

    return target_face


def age_check(age):
    if age > 0 and age <= 6:
        result = "child"
    elif age > 6 and age <= 17:
        result = "teen"
    elif age > 17 and age <= 40:
        result = "youth"
    elif age > 40 and age <= 65:
        result = "middle-age"
    else:
        result = "elder"

    return result


def face_age(age):
    """
    Age check

    Args:
      age: child, teen, youth, middle-age, elder

    Returns:
      True / False for age check
    """
    face_list = face_service()
    if len(face_list) == 0:
        return False
    else:
        target_face = face_filter(face_list)
        current_age = target_face.get('age')
        if age == age_check(current_age):
            return True
        else:
            return False


def face_gender(gender):
    """
    Gender check

    Args:
      gender: male / female

    Returns:
      True / False for gender check
    """

    face_list = face_service()
    if len(face_list) == 0:
        return False
    else:
        target_face = face_filter(face_list)
        current_gender = target_face.get('gender')['type']
        if gender == current_gender:
            return True
        else:
            return False


def face_emotion(emotion):
    """
    Emotion check

    Args:
      emotion: angry, disgust, fear, happy, sad, surprise, neutral, pouty, grimace

    Returns:
      True / False for emotion check
    """

    face_list = face_service()
    if len(face_list) == 0:
        return False
    else:
        target_face = face_filter(face_list)
        result = target_face.get('emotion')['type']
        if result == emotion:
            return True
        else:
            return False


def face_detect(timeout, gender=None, age=None, emotion=None):
    """
    Face in next several seconds

    Args:
      timeout: time for detect face

    Returns:
      True if face detected, False if no face detected
    """

    start_time = time.time()
    result = False

    while True:
        face_list = face_service()
        if len(face_list) > 0:
            tmp_result = True
            target_face = face_filter(face_list)
            if tmp_result and gender is not None:
                current_gender = target_face.get('gender')['type']
                tmp_result = (current_gender == gender)
            if tmp_result and age is not None:
                current_age = age_check(target_face.get('age'))
                tmp_result = (current_age == age)
            if tmp_result and emotion is not None:
                current_emotion = target_face.get('emotion')['type']
                tmp_result = (current_emotion == emotion)

            result = tmp_result

        cur_time = time.time()
        if result or cur_time - start_time > timeout:
            break

    return result


def face_verify(file_name):
    """
    Verify whether two faces belong to a same person

    Args:
      filename: upload image file name

    Returns:
      True if belongs to a same person, else false
    """

    file_path = "%s%s" % (FACE_PATH, file_name)

    # detect default face
    face_list = face_service(file_path)
    if len(face_list) == 0:
        return False
    else:
        target_face = face_filter(face_list)
        faceId1 = target_face.get('face_token')

    # detect camera image
    face_list = face_service()
    if len(face_list) == 0:
        return False
    else:
        target_face = face_filter(face_list)
        faceId2 = target_face.get('face_token')

    try:
        rospy.wait_for_service('ros_vision_node/face_verify', timeout=2)
        face_verify_client = rospy.ServiceProxy('ros_vision_node/face_verify', FaceVerifyService)
        res = face_verify_client(faceId1, faceId2)
        if res.result > 75.0:
            return True
        else:
            return False
    except Exception as err:
        serror(err)
        return False


def face_count():
    """
    Return count of recognized faces, max face number: 10
    """

    face_list = face_service()
    return len(face_list)


if __name__ == '__main__':
    rospy.init_node('face_test', anonymous=True)

    try:
        # result = face_age("teen")
        # if result:
        # 	print("You are teenager")
        # else:
        # 	print("You are not teenager")

        # result = face_gender("male")
        # if result:
        # 	print("You are male")
        # else:
        # 	print("You are female")

        # result = face_emotion("happiness")
        # if result:
        # 	print("You are happy")
        # else:
        # 	print("You are not happy")

        result = face_detect(10, "male", "youth", "neutral")
        # result = face_verify("test.jpeg")
        print(result)
    except Exception as err:
        rospy.logerr(err)
