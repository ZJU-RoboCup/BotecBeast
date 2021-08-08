import face_recognition
import cv2


def face_location(frame, frame_center):
    scale = 0.25
    small_frame = cv2.resize(frame, (0, 0), fx=scale, fy=scale)
    face_locations = face_recognition.face_locations(small_frame)
    if len(face_locations) > 0:
        y0, x0, y1, x1 = face_locations[0]
        face_x = (x0 + x1) // 2
        face_y = (y0 + y1) // 2
        return face_x, face_y
    return frame_center
