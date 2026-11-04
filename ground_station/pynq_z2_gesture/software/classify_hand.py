"""Single-hand silhouette rules with wrist removal and explicit V-tip evidence.

PL still supplies the cleaned skin mask. These rules are not a trained model.
"""
from functools import lru_cache

import cv2
import numpy as np
from hand_native import native as _native

CLASSIFIER_VERSION = 'wrist_lobes_v3'


@lru_cache(maxsize=8)
def _geometry(shape):
    h, w = shape
    border = np.concatenate([
        np.c_[np.arange(w), np.zeros(w)],
        np.c_[np.full(h-1, w-1), np.arange(1, h)],
        np.c_[np.arange(w-2, -1, -1), np.full(w-1, h-1)],
        np.c_[np.zeros(max(0, h-2)), np.arange(h-2, 0, -1)],
    ]).astype(np.int32)
    yy, xx = np.indices(shape, dtype=np.float32)
    return border, xx, yy


def _palm(component):
    # Zero padding also treats a skin component touching the ROI as bounded.
    padded = cv2.copyMakeBorder(component,1,1,1,1,cv2.BORDER_CONSTANT,value=0)
    distance = cv2.distanceTransform(padded, cv2.DIST_L2, 5)[1:-1, 1:-1]
    _, radius, _, peak = cv2.minMaxLoc(distance)
    if _native is not None:
        native_peak = _native.peak(distance, radius*.98)
        if native_peak is not None: peak = native_peak
    else:
        py, px = np.nonzero(distance >= radius * .98)
        if len(px):
            peak = (float(px.mean()), float(py.mean()))
    return np.asarray(peak, dtype=np.float32), float(radius), distance


def _remove_wrist(component, palm, radius, distance):
    border, xx, yy = _geometry(component.shape)
    if _native is not None:
        contact_length,contact=_native.contact(component)
        if contact is None:return component,None,False,palm,radius
    else:
        edge = component[border[:, 1], border[:, 0]] > 0
        if not edge.any() or edge.all():
            return component, None, False, palm, radius
        # Start the circular perimeter at background, so corner-crossing contact
        # remains one run rather than two competing wrist candidates.
        shift = int(np.flatnonzero(~edge)[0])
        edge = np.roll(edge, -shift)
        border = np.roll(border, -shift, axis=0)
        changes = np.diff(np.r_[False, edge, False].astype(np.int8))
        starts, ends = np.flatnonzero(changes == 1), np.flatnonzero(changes == -1)
        longest = int(np.argmax(ends-starts))
        start, end = int(starts[longest]), int(ends[longest])
        contact_length=end-start
        contact=border[start:end].mean(axis=0).astype(np.float32)
    if contact_length < radius * .65:
        return component, None, False, palm, radius
    direction = contact - palm
    length = float(np.linalg.norm(direction))
    # A nearly equally thick forearm can win the global distance maximum.
    # Search the broad core furthest from the wrist entry, then retain its
    # local distance peak. Thin finger tips cannot enter this broad core.
    if _native is not None:
        anchor_y, anchor_x, square = _native.anchor(distance, contact, radius*.8)
        far_distance = float(np.sqrt(square))
    else:
        contact_distance = (xx-contact[0])**2 + (yy-contact[1])**2
        broad_core = distance >= radius*.8
        anchor_index = int(np.argmax(np.where(broad_core, contact_distance, -1)))
        anchor_y, anchor_x = np.unravel_index(anchor_index, component.shape)
        far_distance = float(np.sqrt(contact_distance[anchor_y, anchor_x]))
    if far_distance > length + radius*.75:
        if _native is not None:
            local_radius, local_mean = _native.local_peak(distance, anchor_x, anchor_y, (radius*.75)**2)
        else:
            neighbourhood = (xx-anchor_x)**2 + (yy-anchor_y)**2 <= (radius*.75)**2
            local_radius = float(np.max(distance[neighbourhood]))
            peak_y, peak_x = np.nonzero(neighbourhood & (distance >= local_radius*.98))
            local_mean = [peak_x.mean(), peak_y.mean()] if len(peak_x) else None
        if local_mean is not None and local_radius >= radius*.8:
            palm = np.asarray(local_mean, dtype=np.float32)
            radius = local_radius
            direction = contact - palm
            length = float(np.linalg.norm(direction))
    # A broad contact very close to the centre is a cropped hand, not a
    # sufficiently observed forearm to extrapolate a wrist direction.
    if length <= radius * 1.5:
        return component, None, False, palm, radius
    direction /= length
    if _native is not None:
        result = _native.cut(component, palm, direction, radius*1.1)
    else:
        projection = (xx-palm[0])*direction[0] + (yy-palm[1])*direction[1]
        result = component.copy()
        result[projection > radius * 1.1] = 0
    return result, direction, True, palm, radius


def _border_tip_support(tips, component, radius):
    if _native is not None:
        return _native.border_support(tips,component,max(3.0,radius*.75))
    h, w = component.shape
    supported = np.ones(len(tips), dtype=bool)
    touching = ((tips[:, 0] <= 0) | (tips[:, 0] >= w-1) |
                (tips[:, 1] <= 0) | (tips[:, 1] >= h-1))
    if not touching.any():
        return supported
    border, _, _ = _geometry(component.shape)
    edge = component[border[:, 1], border[:, 0]] > 0
    if edge.all():
        supported[touching] = False
        return supported
    shift = int(np.flatnonzero(~edge)[0])
    edge = np.roll(edge, -shift)
    border = np.roll(border, -shift, axis=0)
    changes = np.diff(np.r_[False, edge, False].astype(np.int8))
    lengths = np.zeros(len(edge), dtype=np.int32)
    for start, end in zip(np.flatnonzero(changes == 1), np.flatnonzero(changes == -1)):
        lengths[start:end] = end-start
    for index in np.flatnonzero(touching):
        nearest = int(np.argmin(np.sum((border-tips[index])**2, axis=1)))
        # A rounded fingertip may lightly touch the ROI. A broad, cut-off
        # hand/forearm edge cannot provide reliable fingertip evidence.
        supported[index] = 0 < lengths[nearest] <= max(3.0, radius*.75)
    return supported


def _long_finger_pair(contour, component, palm, radius, start, end, valley):
    """Require two separate, elongated branches beyond the palm/fork root.

    A bent fist can have a deep C-shaped thumb concavity. Its two sides are
    broad knuckles and a short thumb, rather than two long narrow fingers.
    """
    _, xx, yy = _geometry(component.shape)
    # A fork unusually far from the palm (including synthetic references)
    # must be cut beyond its valley before its two branches can separate.
    root_ratio = max(1.6, float(np.linalg.norm(valley-palm))/radius + .15)
    silhouette = np.zeros_like(component)
    cv2.drawContours(silhouette, [contour], -1, 255, -1)
    if _native is not None:
        radial_squared, outer = _native.outer(silhouette, palm, (root_ratio*radius)**2)
    else:
        radial_squared = (xx-palm[0])**2 + (yy-palm[1])**2
        outer = ((silhouette > 0) & (radial_squared >= (root_ratio*radius)**2)).astype(np.uint8)
    _, labels, stats, _ = cv2.connectedComponentsWithStats(outer, connectivity=8)
    first = int(labels[int(start[1]), int(start[0])])
    second = int(labels[int(end[1]), int(end[0])])
    if first == 0 or second == 0 or first == second:
        return False
    for label in (first, second):
        if stats[label, cv2.CC_STAT_AREA] < max(8.0, radius*radius*.05):
            return False
        py, px = np.nonzero(labels == label)
        points = np.column_stack((px, py)).astype(np.float32)
        sides = cv2.minAreaRect(points)[1]
        major, minor = max(sides)+1.0, min(sides)+1.0
        extension = float(np.sqrt(np.max(radial_squared[py, px])))/radius-root_ratio
        if extension < 1.2 or major/minor < 1.8 or minor > radius*1.25:
            return False
    return True


def _finger_gaps(contour, palm, radius, component, wrist_direction):
    indices = cv2.convexHull(contour, returnPoints=False)
    if indices is None or len(indices) < 4 or len(contour) < 4:
        return 0, 0, 0, 0, 0.0
    try:
        defects = cv2.convexityDefects(contour, indices)
    except cv2.error:
        return 0, 0, 0, 0, 0.0
    if defects is None:
        return 0, 0, 0, 0, 0.0
    defects = defects.reshape(-1, 4)
    defects = defects[defects[:, 3] > max(8.0, radius*.35)*256]
    if not len(defects):
        return 0, 0, 0, 0, 0.0
    points = contour[:, 0].astype(np.float32)
    start, end, valley = (points[defects[:, index]] for index in range(3))
    a, b = start-valley, end-valley
    norm = np.linalg.norm(a, axis=1)*np.linalg.norm(b, axis=1)
    cosine = np.divide(np.einsum('ij,ij->i', a, b), norm,
                       out=np.full(len(norm), -1.0), where=norm > 0)
    sr = np.linalg.norm(start-palm, axis=1)
    er = np.linalg.norm(end-palm, axis=1)
    vr = np.linalg.norm(valley-palm, axis=1)
    near = np.minimum(sr, er)
    valid = ((norm > 0) & (cosine > np.cos(np.deg2rad(100))) &
             (near > radius*1.4) & (vr < near*.85))
    # The cut forearm end cannot be a finger tip.
    for tips in (start, end):
        if wrist_direction is not None:
            valid &= ((tips-palm) @ wrist_direction < radius*.4)
    candidate_gaps = int(np.count_nonzero(valid))
    # Keep cropped-edge evidence separate. It cannot establish a V, but a
    # widely spread palm with several fully observed gaps can remain OPEN.
    for tips in (start, end):
        valid &= _border_tip_support(tips, component, radius)
    # A V additionally needs two long, distinct protrusions on the same side
    # of the palm. A lone concavity (especially a bent wrist) is insufficient.
    radial_cosine = np.divide(np.einsum('ij,ij->i', start-palm, end-palm), sr*er,
                              out=np.full(len(sr), -1.0), where=(sr*er) > 0)
    depth_ratio = defects[:, 3] / (256.0*radius)
    v_evidence = (valid & (near > radius*2.0) & (depth_ratio > .85) &
                  (near-vr > radius*.45) &
                  (radial_cosine > np.cos(np.deg2rad(110))) &
                  (radial_cosine < np.cos(np.deg2rad(15))))
    # OPEN uses the gap count directly. Only a single-gap V candidate needs
    # this additional connected-component measurement.
    if np.count_nonzero(valid) == 1:
        for index in np.flatnonzero(v_evidence):
            v_evidence[index] = _long_finger_pair(
                contour, component, palm, radius, start[index], end[index], valley[index])
    else:
        v_evidence[:] = False
    shallow = valid & (depth_ratio < .85) & (near < radius*2.0)
    max_depth_ratio = float(np.max(depth_ratio[valid])) if valid.any() else 0.0
    return (int(np.count_nonzero(valid)), int(np.count_nonzero(v_evidence)),
            candidate_gaps, int(np.count_nonzero(shallow)), max_depth_ratio)


def classify_clean(clean):
    """Return ordinary JSON-compatible measurements and a display contour."""
    if clean.ndim != 2 or min(clean.shape) < 2:
        raise ValueError('A two-dimensional skin mask of at least 2x2 is required')
    contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    info = dict(gesture='NONE', gaps=0, candidate_gaps=0, v_evidence=0,
                shallow_notches=0, max_gap_depth_ratio=0.0, max_radius_ratio=0.0,
                solidity=0.0, extent=0.0,
                source_area_fraction=0.0, wrist_clipped=False, palm_radius=0.0,
                palm=None, reason='NO_HAND')
    if not contours:
        return info, None
    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    info['source_area_fraction'] = round(area / clean.size, 4)
    if area < max(250, clean.size*.025):
        return info, None
    if info['source_area_fraction'] > .75:
        info.update(gesture='UNKNOWN', reason='ROI_FILLED')
        return info, contour
    component = np.zeros_like(clean)
    # Internal lighting holes do not change the exterior hand silhouette and
    # must not push the distance-transform palm centre into the forearm.
    cv2.drawContours(component, [contour], -1, 255, -1)
    palm, radius, distance = _palm(component)
    if radius < max(4.0, min(clean.shape)*.03):
        info.update(gesture='UNKNOWN', reason='NO_PALM')
        return info, contour
    hand, wrist, clipped, palm, radius = _remove_wrist(component, palm, radius, distance)
    info.update(palm=[round(float(v), 3) for v in palm], palm_radius=round(radius, 3))
    info['wrist_clipped'] = bool(clipped)
    if clipped:
        candidates, _ = cv2.findContours(hand, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        containing = [c for c in candidates if cv2.pointPolygonTest(c, tuple(map(float, palm)), False) >= 0]
        if not containing:
            info.update(gesture='UNKNOWN', reason='WRIST_CUT_FAILED')
            return info, contour
        contour = max(containing, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
    if hull_area <= 0:
        info.update(gesture='UNKNOWN', reason='DEGENERATE_CONTOUR')
        return info, contour
    x, y, w, h = cv2.boundingRect(contour)
    solidity, extent = area/hull_area, area/(w*h)
    gaps, v_evidence, candidate_gaps, shallow, gap_depth = _finger_gaps(
        contour, palm, radius, hand, wrist)
    max_radius = float(np.max(np.linalg.norm(contour[:, 0]-palm, axis=1)) / radius)
    perimeter = cv2.arcLength(contour, True)
    polygon = cv2.approxPolyDP(contour, perimeter*.012, True)
    polygon_object = len(polygon) <= 4 and solidity > .975
    if polygon_object:
        label, reason = 'UNKNOWN', 'POLYGON_OBJECT'
    elif (gaps >= 3 or (gaps >= 2 and candidate_gaps >= 4 and solidity < .8
                       and min(w, h) > radius*3.5)):
        label, reason = 'OPEN', 'OK'
    elif gaps == 1 and v_evidence == 1 and solidity < .90:
        label, reason = 'V', 'OK'
    elif (solidity >= .82 and extent >= .50 and max(w/h, h/w) < 1.65 and
          (gaps == 0 or (gaps == 1 and v_evidence == 0 and max_radius <= 2.6))):
        label = 'FIST'
        reason = ('SHALLOW_SHADOW_NOTCH' if shallow else 'COMPACT_FIST_NOTCH') if gaps else 'OK'
    else:
        label, reason = 'UNKNOWN', 'UNCLEAR_SHAPE'
    info.update(gesture=label, reason=reason, gaps=gaps, candidate_gaps=candidate_gaps,
                v_evidence=v_evidence,
                shallow_notches=shallow, max_gap_depth_ratio=round(gap_depth, 3),
                max_radius_ratio=round(max_radius, 3),
                solidity=round(solidity, 3), extent=round(extent, 3),
                bbox=[int(x), int(y), int(w), int(h)])
    return info, contour
