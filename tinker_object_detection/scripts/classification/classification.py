import numpy as np
import tensorflow as tf
import vgg16
import cv2
import sys
import utils
try:
    import cPickle as pickle
except ImportError:
    import pickle


def get_vgg_func():
    vgg = vgg16.Vgg16()
    with tf.device('/cpu:0'):
        images = tf.placeholder("float", [1, 224, 224, 3])
        vgg.build(images)
    
    def vgg_func(sess, img):
        h, w = img.shape[:2]  
        if h != 224 or w != 224:
            img = resize_center_keep_ratio(img, (224, 224))
        batch = img.reshape((1, 224, 224, 3))
        feed_dict = {images: batch}
        return sess.run([vgg.fc6, vgg.prob], feed_dict=feed_dict)
    return vgg_func 


def load_bet(bet_filename):
    bet = pickle.load(open(bet_filename))
    # A bias to prefer children nodes in single-chain paths
    bet['infogain'] -= np.array(bet['preferences']) * 0.1
    return bet


def resize_center_keep_ratio(img, target_shape):
    tw, th = target_shape
    h, w = img.shape[:2]
    f = min(float(tw) / w, float(th) / h)
    img = cv2.resize(img, (0, 0), fx=f, fy=f)
    h, w, c = img.shape
    pad_l = (tw - w) / 2
    pad_r = tw - w - pad_l
    pad_t = (th - h) / 2
    pad_b = th - h - pad_t
    img = cv2.copyMakeBorder(img, pad_t, pad_b, pad_l, pad_r, 
            cv2.BORDER_CONSTANT, value=[0] * c)
    return img


img = cv2.imread(sys.argv[1])
vgg_func = get_vgg_func()
synset = [l.strip() for l in open('./tensorflow-vgg/synset.txt').readlines()]
with tf.Session() as sess:
    fc6, prob = vgg_func(sess, img)
    prob = prob[0]
    for v in utils.get_prob_top(prob, synset):
        print(v)
    print('=================')
    for v in utils.get_label_on_tree(prob, load_bet('./imagenet.bet.pickle')):
        print(v)

