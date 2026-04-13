python - <<'PY'
import mmcv
import numpy as np
from pathlib import Path

p = Path('/data/dl/BEVFormer_segmentation_detection/nuscenes_infos_temporal_test.pkl')
obj = mmcv.load(str(p))

for info in obj['infos']:
    for cam, c in info['cams'].items():
        c['cam_intrinsic'] = np.array(c['cam_intrinsic'], dtype=np.float32)

mmcv.dump(obj, str(p))
print('patched cam_intrinsic to np.ndarray:', p)
PY