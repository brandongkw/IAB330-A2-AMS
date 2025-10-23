import numpy as np

def _get_vec3(sample, key):
    v = sample[key]
    if isinstance(v, dict):
        return float(v.get("x", 0)), float(v.get("y", 0)), float(v.get("z", 0))
    # assume list/tuple
    return float(v[0]), float(v[1]), float(v[2])

def _feats(samples):
    ax, ay, az, gx, gy, gz = [], [], [], [], [], []
    for s in samples:
        x, y, z = _get_vec3(s, "accel")
        ax.append(x); ay.append(y); az.append(z)
        xg, yg, zg = _get_vec3(s, "gyro")
        gx.append(xg); gy.append(yg); gz.append(zg)

    ax = np.asarray(ax, dtype=float)
    ay = np.asarray(ay, dtype=float)
    az = np.asarray(az, dtype=float)
    gx = np.asarray(gx, dtype=float)
    gy = np.asarray(gy, dtype=float)
    gz = np.asarray(gz, dtype=float)

    amag = np.sqrt(ax**2 + ay**2 + az**2)
    gmag = np.sqrt(gx**2 + gy**2 + gz**2)

    def sstats(v):
        if v.size == 0:
            return [0.0, 0.0, 0.0, 0.0]
        return [float(np.mean(v)), float(np.std(v, ddof=1) if v.size > 1 else 0.0),
                float(np.min(v)), float(np.max(v))]

    # ax, ay, az, amag, gmag → 5 channels × 4 stats = 20 features
    return sstats(ax)+sstats(ay)+sstats(az)+sstats(amag)+sstats(gmag)

def compute_features(window):
    """
    window: {"wrist":[samples...], "ankle":[samples...]}
    sample: {"accel":{x,y,z} or [x,y,z], "gyro":{x,y,z} or [x,y,z]}
    returns numpy array of length 40: 20 (wrist) + 20 (ankle)
    """
    vec = []
    for limb in ("wrist", "ankle"):
        if limb in window and len(window[limb]) > 0:
            vec += _feats(window[limb])
        else:
            vec += [0.0]*20
    return np.array(vec, dtype=float)
