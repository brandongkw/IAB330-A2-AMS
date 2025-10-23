import os
import numpy as np
from joblib import load
from feature_extractor import compute_features

class _FallbackModel:
    def predict(self, X):
        out = []
        for row in X:
            # wrist amag std at index 13, ankle at 33
            wrist_am_std = row[13]
            ankle_am_std = row[33] if len(row) > 33 else wrist_am_std
            m = float(max(wrist_am_std, ankle_am_std))
            if m < 0.15:
                out.append(0)  # Idle
            elif m < 0.50:
                out.append(1)  # Walking
            else:
                out.append(2)  # Running
        return np.array(out)

    def predict_proba(self, X):
        preds = self.predict(X)
        proba = []
        for p in preds:
            arr = np.array([0.2, 0.2, 0.2], dtype=float)
            arr[int(p)] = 0.6
            proba.append(arr)
        return np.array(proba)

class InferenceEngine:
    def __init__(self, model_path, label_map=None):
        self.label_map = label_map or {0:"Idle", 1:"Walking", 2:"Running"}
        self.model = self._load_model(model_path)

    def _load_model(self, path):
        try:
            if path and os.path.exists(path):
                return load(path)
        except Exception:
            pass
        return _FallbackModel()

    def predict_window(self, window_dict):
        X = compute_features(window_dict).reshape(1, -1)
        y = int(self.model.predict(X)[0])
        label = self.label_map.get(y, str(y))
        try:
            proba = self.model.predict_proba(X)[0]
        except Exception:
            proba = None
        return label, proba
