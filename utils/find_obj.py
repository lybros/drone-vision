#!/usr/bin/env python

"""
Feature-based image matching sample.

Note, that you will need the https://github.com/Itseez/opencv_contrib repo for SIFT and SURF

USAGE
    python find_obj.py [
        --feature=<sift|surf|orb|akaze|brisk>[-flann]
        --fwd=<input_images_dir>
        --bck=<input_images_dir>
        --out=<output_images_dir>]
        [ <image_to_find> ]

    --feature - Feature to use. Can be sift, surf, orb or brisk. Append '-flann'
               to feature name to use Flann-based matcher instead bruteforce.
    --fwd - input images dir
    --bck - if not null compare all from fwd with bck
    --out - directory will contain comparing images with matches
    pos arg - image that you want to find on images from <input_images_dir> [use if --bck=null]
"""

import os
import cv2
import numpy as np

FLANN_INDEX_KDTREE = 1  # bug: flann enums are missing
FLANN_INDEX_LSH = 6


def clock():
    return cv2.getTickCount() / cv2.getTickFrequency()


class Timer:
    def __init__(self, message):
        self.message = message
        self.start = clock()

    def finish(self):
        print "\n{0}: {1:.3f} s\n".format(self.message, (clock() - self.start)) + "-" * 50


def init_feature(name):
    chunks = name.split('-')
    if chunks[0] == 'sift':
        detector = cv2.xfeatures2d.SIFT_create()
        norm = cv2.NORM_L2
    elif chunks[0] == 'surf':
        detector = cv2.xfeatures2d.SURF_create(800)
        norm = cv2.NORM_L2
    elif chunks[0] == 'orb':
        detector = cv2.ORB_create(400)
        norm = cv2.NORM_HAMMING
    elif chunks[0] == 'akaze':
        detector = cv2.AKAZE_create()
        norm = cv2.NORM_HAMMING
    elif chunks[0] == 'brisk':
        detector = cv2.BRISK_create()
        norm = cv2.NORM_HAMMING
    else:
        return None, None
    if 'flann' in chunks:
        if norm == cv2.NORM_L2:
            flann_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        else:
            flann_params = dict(algorithm=FLANN_INDEX_LSH,
                                table_number=6,  # 12
                                key_size=12,  # 20
                                multi_probe_level=1)  # 2
        matcher = cv2.FlannBasedMatcher(flann_params, {})  # bug : need to pass empty dict (#1329)
    else:
        matcher = cv2.BFMatcher(norm)
    return detector, matcher


def filter_matches(kp1, kp2, matches, ratio=0.75):
    mkp1, mkp2 = [], []
    for m in matches:
        if len(m) == 2 and m[0].distance < m[1].distance * ratio:
            m = m[0]
            mkp1.append(kp1[m.queryIdx])
            mkp2.append(kp2[m.trainIdx])
    p1 = np.float32([kp.pt for kp in mkp1])
    p2 = np.float32([kp.pt for kp in mkp2])
    kp_pairs = zip(mkp1, mkp2)
    return p1, p2, kp_pairs


def explore_match(img1, img2, kp_pairs, status=None, H=None):
    h1, w1 = img1.shape[:2]
    h2, w2 = img2.shape[:2]
    vis = np.zeros((max(h1, h2), w1 + w2), np.uint8)
    vis[:h1, :w1] = img1
    vis[:h2, w1:w1 + w2] = img2
    vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)

    if H is not None:
        corners = np.float32([[0, 0], [w1, 0], [w1, h1], [0, h1]])
        corners = np.int32(cv2.perspectiveTransform(corners.reshape(1, -1, 2), H).reshape(-1, 2) + (w1, 0))
        cv2.polylines(vis, [corners], True, (255, 255, 255))

    if status is None:
        status = np.ones(len(kp_pairs), np.bool_)
    p1 = np.int32([kpp[0].pt for kpp in kp_pairs])
    p2 = np.int32([kpp[1].pt for kpp in kp_pairs]) + (w1, 0)

    green = (0, 255, 0)
    red = (0, 0, 255)
    for (x1, y1), (x2, y2), inlier in zip(p1, p2, status):
        if inlier:
            col = green
            cv2.circle(vis, (x1, y1), 2, col, -1)
            cv2.circle(vis, (x2, y2), 2, col, -1)
        else:
            col = red
            r = 2
            thickness = 3
            cv2.line(vis, (x1 - r, y1 - r), (x1 + r, y1 + r), col, thickness)
            cv2.line(vis, (x1 - r, y1 + r), (x1 + r, y1 - r), col, thickness)
            cv2.line(vis, (x2 - r, y2 - r), (x2 + r, y2 + r), col, thickness)
            cv2.line(vis, (x2 - r, y2 + r), (x2 + r, y2 - r), col, thickness)
    for (x1, y1), (x2, y2), inlier in zip(p1, p2, status):
        if inlier:
            cv2.line(vis, (x1, y1), (x2, y2), green)

    return vis


def find_obj(img1, img2, detector, matcher, img1name='img1', img2name='img2'):
    kp1, desc1 = detector.detectAndCompute(img1, None)
    kp2, desc2 = detector.detectAndCompute(img2, None)
    print '{0} - {1} features, {2} - {3} features'.format(img1name, len(kp1), img2name, len(kp2))

    raw_matches = matcher.knnMatch(desc1, trainDescriptors=desc2, k=2)  # 2
    p1, p2, kp_pairs = filter_matches(kp1, kp2, raw_matches)
    if len(p1) < 4:
        print '{0} matches found, not enough for homography estimation'.format(len(p1))
        return np.array([]), 0

    H, status = cv2.findHomography(p1, p2, cv2.RANSAC, 5.0)
    if H is None or status is None:
        return np.array([]), 0

    print '{0} / {1}  inliers/matched'.format(np.sum(status), len(status))
    return explore_match(img1, img2, kp_pairs, status, H), np.sum(status) / len(status)


if __name__ == '__main__':
    print __doc__

    import sys
    import getopt


    def read_image(name):
        img = cv2.imread(name, 0)

        if img is None:
            print 'Failed to load image:', name
            sys.exit(1)
        else:
            return img


    total = Timer("TOTAL")
    opts, args = getopt.getopt(sys.argv[1:], '', ['feature=', 'bck=', 'fwd=', 'out='])
    opts = dict(opts)
    feature_name = opts.get('--feature', 'orb')
    forward_dir = opts.get('--fwd', 'fwd/')
    back_dir = opts.get('--bck', '')
    out_dir = opts.get('--out', 'out/')
    if not back_dir and len(args) < 1:
        print 'positional argument image is required'
        sys.exit(1)

    res = {
        'score': 0,
        'image': None
    }
    forward_images = os.listdir(forward_dir)
    back_images = os.listdir(back_dir) if back_dir else [args[0]]
    detector, matcher = init_feature(feature_name)
    matrics = []

    if detector is None:
        print 'unknown feature:', feature_name
        sys.exit(1)

    print 'using', feature_name, '\n', '-' * 50

    for img1_name in forward_images:

        img1 = read_image(os.path.join(forward_dir, img1_name))
        results = []

        for img2_name in back_images:
            timer = Timer("procces ({0} & {1})".format(img1_name, img2_name))

            img2 = read_image(os.path.join(back_dir, img2_name))
            vis, score = find_obj(img1, img2, detector, matcher, img1_name, img2_name)
            print 'score: {:.3f}'.format(score)
            timer.finish()
            results.append(score)

            if score > res['score']:
                res['score'] = score
                res['image'] = img2_name

            if vis.any():
                cv2.imwrite(os.path.join(out_dir, img1_name + img2_name), vis)

        print 'best match with image {0} with score={1}'.format(res['image'], res['score'])
        matrics.append(results)

    total.finish()
