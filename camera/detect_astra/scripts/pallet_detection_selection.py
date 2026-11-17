"""Suppress duplicate pallet classes on one object; retain separate pallets/Box."""
import math

PALLET_CLASSES = ('pallets', 'emty pallet')
# Count only candidates that can qualify semantic acquisition. A weak false
# positive must not veto a separate, otherwise valid pallet indefinitely.
MIN_PALLET_CONFIDENCE = .75


def unique_pallet_indices(boxes, classes, confidence, names):
    candidates = [i for i, cls in enumerate(classes)
                  if names[int(cls)].strip().lower() in PALLET_CLASSES]
    candidates.sort(key=lambda i: float(confidence[i]), reverse=True)
    kept = []
    for i in candidates:
        box = boxes[i]
        if (not all(math.isfinite(float(v)) for v in (*box, confidence[i]))
                or not MIN_PALLET_CONFIDENCE <= confidence[i] <= 1.
                or box[2] <= box[0] or box[3] <= box[1]):
            continue
        duplicate = False
        for j in kept:
            other = boxes[j]
            intersection = max(0., min(box[2],other[2])-max(box[0],other[0])) * max(
                0., min(box[3],other[3])-max(box[1],other[1]))
            union = ((box[2]-box[0])*(box[3]-box[1]) +
                     (other[2]-other[0])*(other[3]-other[1])-intersection)
            if intersection/union >= .8:
                duplicate = True
                break
        if not duplicate:
            kept.append(i)
    return kept
