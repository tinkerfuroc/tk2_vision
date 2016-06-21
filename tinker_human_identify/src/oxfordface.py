from apikeys import FACE_KEY, EMOTION_KEY
from projectoxford import Client
import cv2

_tmp_filename = '/tmp/324ajow_we.jpg'

def find_faces(img):
    face_client = Client.Face(FACE_KEY)
    cv2.imwrite(_tmp_filename, img)
    face_options = {
            'path' : _tmp_filename,
            'analyzesAge' : False,
            'analyzesGender' : True 
            }
    detect_results = face_client.detect(face_options)
    return {r['faceId'] : r for r in detect_results]}

def detect_same_face_in_img(img, detect_face_id):
    face_client = Client.Face(FACE_KEY)
    found_faces = find_faces(img)
    img_face_ids = found_faces.keys()
    simlar_results = face_client.similar(detect_face_id, img_face_ids)
    return [r['faceId'] for r in simlar_results 
            if is_same_person(r['faceId'], detect_face_id)], found_faces

def is_same_person(img, face_id1, face_id2):
    face_client = Client.Face(FACE_KEY)
    return face_client.verify(face_id1, face_id2)['isIdentical']
