import os
import json
import glob
import cv2
import face_recognition


def say_hello(name):
    msg = "hello {} !".format(name)


class Face(object):
    def __init__(self):
        self.save_dir = '/home/lemon/robot_ros_application/faces/'
        self.persons = []
        self.known_face_encodings = []
        self.scale = 0.25

    def _central(self, locations):
        """find central of faces

        :param locations:
        :return:
        """
        centers = []
        for (top, right, bottom, left) in locations:
            top = top / self.scale
            right = right / self.scale
            bottom = bottom / self.scale
            left = left / self.scale
            center = (top + bottom) / 2, (left + right) / 2
            centers.append(center)
        return centers

    def read_encoding(self):
        """read saved face imgs

        :return:
        """
        # json_lists = glob.glob(os.path.join(self.save_dir, "*.json"))
        json_lists = []
        for file in os.listdir(self.save_dir):
            if file.endswith(".json"):
                json_lists.append(os.path.join(self.save_dir, file))

        # print(self.save_dir, json_lists)
        for json_file in json_lists:
            with open(json_file, 'r') as fp:
                person = json.load(fp)
                face_img = face_recognition. \
                    load_image_file(os.path.join(self.save_dir, person['img']))
                face_encoding = face_recognition. \
                    face_encodings(face_img)[0]
                self.persons.append(person)
                self.known_face_encodings.append(face_encoding)

    def locate(self, frame):
        """Locate faces of current frame

        :param frame:
        :return:
        """
        small_frame = cv2.resize(frame, (0, 0), fx=self.scale, fy=self.scale)
        rgb_small_frame = small_frame[:, :, ::-1]
        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_centrals = self._central(face_locations)
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)
        face_names = []
        for face_encoding in face_encodings:
            matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
            name = "Unknown"
            if True in matches:
                first_match_index = matches.index(True)
                name = self.persons[first_match_index]['name']
            face_names.append(name)
        return zip(face_centrals, face_names)


if __name__ == '__main__':
    pass
