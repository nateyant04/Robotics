import cv2
import numpy as np
import pyvista as pv
from sklearn.cluster import MiniBatchKMeans

# i dont do blurring here because it works fine without but we should probably still do it in the real one
# cv2 also has a built in gauss blur with cv2.GaussianBlur(img, (5, 5), 0) turns out
# arrow keys to change steps and drag/zoom the 3d

# distinct colors (RGB) used for cluster labels
PALETTE = np.array([
    [230, 60, 60],
    [60, 140, 230],
    [80, 200, 90],
    [240, 200, 50],
    [180, 90, 220],
    [240, 140, 40],
    [50, 210, 210],
    [240, 240, 240],
], dtype=np.uint8)


def label_colors(labels):
    return PALETTE[np.asarray(labels) % len(PALETTE)]


def sample_indices(n_pixels, max_points=500000, seed=0):
    if n_pixels <= max_points:
        return np.arange(n_pixels)
    rng = np.random.default_rng(seed)
    return rng.choice(n_pixels, max_points, replace=False)


def image_to_cloud(img_bgr, idx):
    rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB).reshape(-1, 3)
    return rgb[idx].astype(np.uint8)


def image_to_texture(img_bgr):
    rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    # flip so the image is not upside down on the plane
    return pv.numpy_to_texture(np.ascontiguousarray(rgb[::-1]))


def show_steps(images, titles=None, display_images=None, point_colors=None,
               centroids=None, max_points=500000):
    """
    images:         per step BGR image; its pixels give the point positions
    display_images: per step BGR image shown on the left (default: images)
    point_colors:   per step None or (H, W, 3) RGB uint8 per-pixel colors
    centroids:      per step None or (positions (k, 3) in [0, 1], colors (k, 3) uint8)
    """
    n = len(images)
    h, w = images[0].shape[:2]
    if titles is None:
        titles = [f"step {i}" for i in range(n)]
    if display_images is None:
        display_images = images
    if point_colors is None:
        point_colors = [None] * n
    if centroids is None:
        centroids = [None] * n

    # same pixel subset for every step, so points correspond across steps
    idx = sample_indices(h * w, max_points)
    clouds = [image_to_cloud(im, idx) for im in images]
    colors = [
        clouds[k] if point_colors[k] is None
        else np.asarray(point_colors[k], dtype=np.uint8).reshape(-1, 3)[idx]
        for k in range(n)
    ]
    textures = [image_to_texture(im) for im in display_images]
    state = {"i": 0}

    pl = pv.Plotter(shape=(1, 2), window_size=(1600, 800))
    pl.set_background([0.1, 0.1, 0.1])

    # --- left: the image on a textured plane ---
    pl.subplot(0, 0)
    plane = pv.Plane(center=(0, 0, 0), direction=(0, 0, 1),
                     i_size=w, j_size=h)
    plane.texture_map_to_plane(inplace=True)
    img_actor = pl.add_mesh(plane, texture=textures[0], show_edges=False,
                            lighting=False)
    pl.add_text(titles[0], position="upper_left", font_size=12,
                color="white", name="title")
    pl.view_xy()
    pl.enable_parallel_projection()

    # --- right: the point cloud ---
    pl.subplot(0, 1)
    cloud = pv.PolyData(clouds[0].astype(np.float32) / 255.0)
    cloud["rgb"] = colors[0]
    pl.add_mesh(cloud, scalars="rgb", rgb=True, point_size=2,
                render_points_as_spheres=False, lighting=False)
    cube = pv.Cube(center=(0.5, 0.5, 0.5)).extract_all_edges()
    pl.add_mesh(cube, color="gray", line_width=1)
    pl.camera_position = [(2.2, 1.6, 2.2), (0.5, 0.5, 0.5), (0, 1, 0)]

    def set_centroids(k):
        pl.subplot(0, 1)
        pl.remove_actor("centroids")
        pl.remove_actor("centroid_labels")
        if centroids[k] is None:
            return
        pos, cols = centroids[k]
        pos = np.asarray(pos, dtype=np.float32)
        pts = pv.PolyData(pos)
        pts["rgb"] = np.asarray(cols, dtype=np.uint8)
        pl.add_mesh(pts, scalars="rgb", rgb=True, point_size=30,
                    render_points_as_spheres=True, lighting=False,
                    name="centroids")
        pl.add_point_labels(pos, [f"c{j}" for j in range(len(pos))],
                            font_size=14, text_color="white",
                            shape_color="black", shape_opacity=0.6,
                            show_points=False, always_visible=True,
                            name="centroid_labels")

    set_centroids(0)

    # --- mouse controls: 2D pan/zoom on the image, 3D rotate on the cloud ---
    # vtk only has one interactor style per window, so swap it depending on
    # which pane the mouse is over
    pl.iren.enable_custom_trackball_style(
        left="pan", shift_left="pan", control_left="pan",
        middle="pan", shift_middle="pan", control_middle="pan",
        right="dolly", shift_right="dolly", control_right="dolly",
    )
    style_2d = pl.iren.style
    pl.iren.enable_trackball_style()
    style_3d = pl.iren.style
    image_renderer = pl.renderers[0]

    def pick_style(*_):
        # don't swap in the middle of a drag
        if pl.iren.style.GetState() != 0:
            return
        want = style_2d if pl.iren.get_poked_renderer() is image_renderer else style_3d
        if pl.iren.style is not want:
            pl.iren.style = want

    pl.iren.add_observer("MouseMoveEvent", pick_style)

    # --- step navigation ---
    def goto(i):
        state["i"] = i % n
        k = state["i"]
        cloud.points = clouds[k].astype(np.float32) / 255.0
        cloud["rgb"] = colors[k]
        img_actor.texture = textures[k]
        pl.subplot(0, 0)
        pl.add_text(titles[k], position="upper_left", font_size=12,
                    color="white", name="title")
        set_centroids(k)
        pl.render()

    pl.add_key_event("Right", lambda: goto(state["i"] + 1))
    pl.add_key_event("Left", lambda: goto(state["i"] - 1))
    # fallbacks in case the arrow key names do not fire on your platform
    pl.add_key_event("d", lambda: goto(state["i"] + 1))
    pl.add_key_event("a", lambda: goto(state["i"] - 1))

    pl.show()


bg = cv2.imread("bg.jpg")
frame = cv2.imread("frame.jpg")

diff = cv2.absdiff(bg, frame)
diff = cv2.normalize(diff, None, 0, 255, cv2.NORM_MINMAX)

h, w, c = diff.shape
pixels = diff.reshape(-1, c).astype(np.float32)   # (H*W, 3), BGR order

km = MiniBatchKMeans(n_clusters=2, batch_size=4096, n_init=3, random_state=0)
labels = km.fit_predict(pixels)

label_img = labels.reshape(h, w)

# cluster visualization inputs
cluster_rgb = label_colors(label_img)                        # (H, W, 3) RGB
cluster_bgr = np.ascontiguousarray(cluster_rgb[..., ::-1])   # for the left pane
centers_rgb = km.cluster_centers_[:, ::-1] / 255.0           # BGR -> RGB, cloud coords
centroid_cols = label_colors(np.arange(km.n_clusters))

show_steps(
    [bg, frame, diff, diff],
    titles=["plate", "frame", "diff", "clusters"],
    display_images=[bg, frame, diff, cluster_bgr],
    point_colors=[None, None, None, cluster_rgb],
    centroids=[None, None, None, (centers_rgb, centroid_cols)],
)