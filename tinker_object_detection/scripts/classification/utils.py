import skimage
import skimage.io
import skimage.transform
import numpy as np


def get_prob_top(prob, synset, top_n=5):
    # print prob
    pred = np.argsort(prob)[::-1]
    return [(synset[pred[i]], prob[pred[i]]) for i in range(top_n)]


def get_label_on_tree(prob, bet, top_n=5):
    expected_infogain = np.dot(bet['probmat'], prob[bet['idmapping']])
    expected_infogain *= bet['infogain']
    infogain_sort = expected_infogain.argsort()[::-1]
    bet_result = [(bet['words'][v], '%.5f' % expected_infogain[v]) \
            for v in infogain_sort[:top_n]]
    return bet_result

