print("importing...")
import cv2
import numpy as np
from sklearn.cluster import MiniBatchKMeans
#import Arm


def brightest_cluster_mask(bg, frame, n_clusters=2):
    diff = cv2.absdiff(bg, frame)
    diff = cv2.normalize(diff, None, 0, 255, cv2.NORM_MINMAX)

    h, w, c = diff.shape
    pixels = diff.reshape(-1, c).astype(np.float32)

    km = MiniBatchKMeans(n_clusters=n_clusters, batch_size=1024, n_init=3, random_state=0)
    labels = km.fit_predict(pixels)

    brightest = np.argmax(np.linalg.norm(km.cluster_centers_, axis=1))
    return ((labels == brightest) * 255).astype(np.uint8).reshape(h, w)

def contour_median_color(frame, mask, cnt, erode_px=3):
    m = np.zeros(frame.shape[:2], np.uint8)
    cv2.drawContours(m, [cnt], -1, 255, -1)  # filled contour
    m = cv2.bitwise_and(m, mask)             # only pixels the cluster mask kept
    if erode_px:
        m = cv2.erode(m, np.ones((erode_px * 2 + 1,) * 2, np.uint8))  # drop blended edge pixels
    pixels = frame[m > 0]                    # (N, 3) BGR (OpenCV order)
    if len(pixels) == 0:
        return None
    return np.median(pixels, axis=0).astype(np.uint8)[::-1]  # BGR -> RGB

# add the actual arm code in here
def send_left():
    print("sending left")

def send_right():
    print("sending right")

def reject():
    print("rejecting") 

ref_cols = [
    (21, 55, 33), # green
    (153, 29, 26) # red
    ]

# file names for the shapes, compared against an empty bg. color dont matter, only shape
ref_shapes = [
    "GL.jpg", # L
    "GZ.jpg", # Z
    "GT.jpg"  # T
    ]

# tuples of (color idx, shape idx) for each reference color and shape combination to accept in what direction
# all other combinations will be rejected
shapes_left  = [(0, 0)]
shapes_right = [(1, 2)]

# minimum of how similar the median color of the detected object must be to a reference color to be considered a match
# from 0 must be exactly the same, to 1 maximum distance in rgb space (sqrt(3*255^2) = 441.67)
color_similarity_threshold = 0.25

# this one just goes from 0 to no upper limit, lower is more similar
shape_similarity_threshold = 0.1

MAX_COLOR_DIST = np.sqrt(3 * 255 ** 2)


def largest_contour(mask):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return max(cnts, key=cv2.contourArea) if cnts else None


def load_image(path):
    img = cv2.imread(path)
    if img is None:
        raise FileNotFoundError(f"could not read image: {path}")
    return img


# uncomment these when ur using the camera
# and comment the load images

#input("press enter to capture background image...")
bg = load_image("bg.jpg")
#_, bg = cam.read()

#input("Captured background image, press enter to capture frame...")

#r, frame = cam.read()
frame = load_image("frame.jpg")

mask = brightest_cluster_mask(bg, frame)
cnt = largest_contour(mask)
if cnt is None:
    print("no object detected")
    reject()
    exit()

mean_col = contour_median_color(frame, mask, cnt)
if mean_col is None:
    print("object too small to measure color")
    reject()
    exit()
print("median color (RGB):", mean_col, "\n")

# find the closest reference color
dists = [np.linalg.norm(mean_col.astype(np.float32) - np.array(c, np.float32)) for c in ref_cols]
for ref_col, dist in zip(ref_cols, dists):
    print("distance to reference color", ref_col, ":", dist)
color_idx = int(np.argmin(dists))
if dists[color_idx] >= color_similarity_threshold * MAX_COLOR_DIST:
    print("color not recognized as any reference color")
    reject()
    exit()
print("color is similar to reference color", ref_cols[color_idx], "by",
      np.round(100 * (1 - dists[color_idx] / MAX_COLOR_DIST)), "%")

print("")

# find the closest reference shape
scores = []
for ref_shape in ref_shapes:
    ref_img = load_image(ref_shape)
    ref_cnt = largest_contour(brightest_cluster_mask(bg, ref_img))
    if ref_cnt is None:
        raise ValueError(f"no object found in reference shape image: {ref_shape}")
    score = cv2.matchShapes(cnt, ref_cnt, cv2.CONTOURS_MATCH_I1, 0.0)
    print("similarity to reference shape", ref_shape, ":", score)
    scores.append(score)
shape_idx = int(np.argmin(scores))
if scores[shape_idx] >= shape_similarity_threshold:
    print("shape not recognized as any reference shape")
    reject()
    exit()
print("shape is similar to reference shape", ref_shapes[shape_idx], "\n")

combo = (color_idx, shape_idx)
if combo in shapes_left:
    send_left()
elif combo in shapes_right:
    send_right()
else:
    print("shape and color recognized, but combination not recognized as any reference combination, rejecting")
    reject()