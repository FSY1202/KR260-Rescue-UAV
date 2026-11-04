"""Same-host vision and actuator protocol, independent of ROS and hardware."""
import json
import math
import os
from pathlib import Path
import re


def session(value):
    return isinstance(value, str) and re.fullmatch('[0-9a-f]{32}', value) is not None


def observation(status, now_ns):
    """Keep acquisition identity/time. HTTP receipt never renews an old frame."""
    if not isinstance(status, dict):
        raise ValueError('Expected detector status object')
    if status.get('state') != 'READY' or status.get('error') or status.get('source_metadata_error'):
        raise ValueError('Detector/source is not ready')
    detector, camera = status.get('detection_session_id'), status.get('source_camera_session_id')
    seq, frame, stamp = (status.get(k) for k in ('result_seq', 'source_frame_seq', 'source_frame_monotonic_ns'))
    if not session(detector) or not session(camera):
        raise ValueError('Missing detector/camera identity')
    if any(type(x) is not int for x in (seq, frame, stamp)) or seq <= 0 or frame < 0 or not 0 < stamp <= now_ns or now_ns-stamp > 2_000_000_000:
        raise ValueError('Invalid/stale/future source frame')
    scores = []
    for item in status.get('detections', []):
        score = item.get('confidence')
        if item.get('class_id') == 0 and type(score) in (int, float) and math.isfinite(score) and .25 <= score <= 1:
            box = item.get('xyxy', [])
            if len(box) == 4 and all(type(x) in (int, float) and math.isfinite(x) for x in box) and box[2] > box[0] and box[3] > box[1]:
                scores.append(score)
    return dict(detector_session=detector, camera_session=camera, result_seq=seq, frame_seq=frame,
                source_monotonic_ns=stamp, person_detected=bool(scores), confidence=max(scores, default=0.))


def authority(status, run_id, request_id, now):
    """Never execute a drop from preview, ground, stale or another mission."""
    try:
        age = now-status['updated_monotonic']
        velocity = status['actual_velocity_ned']
        if not isinstance(velocity, list) or len(velocity) != 3:
            return False
        speed = math.sqrt(sum(float(x)**2 for x in velocity))
        return (session(run_id) and status['run_id'] == run_id and 0 <= age < .8 and
                status['execute'] is True and status['triggered'] is True and
                status['armed'] is True and status['landed'] is False and status['nav_state'] == 14 and
                status['state'] == 'RESCUE_DROP' and status['drop_request_id'] == request_id and
                status['healthy'] is True and status['planning_healthy'] is True and
                math.isfinite(speed) and speed < .1)
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


class Journal:
    """Persist BEFORE enabling output. An uncertain restart is never replayed."""
    def __init__(self, directory):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)

    def claim(self, run_id, request_id):
        if not session(run_id) or type(request_id) is not int or not 0 < request_id < 2**32:
            raise ValueError('Invalid request identity')
        path = self.directory / (run_id+'_'+str(request_id)+'.json')
        try:
            with path.open('x', encoding='utf-8') as out:
                json.dump(dict(run_id=run_id, request_id=request_id, state='CLAIMED'), out)
                out.flush()
                os.fsync(out.fileno())
            # Linux fsync(directory) persists the directory entry as well.
            if os.name == 'posix':
                fd = os.open(self.directory, os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(fd)
                finally:
                    os.close(fd)
            return True
        except FileExistsError:
            return False
