// unity 전송

"""
radar_tcp_sender.py

Shared Memory → ClusterTracker → TCP → Unity

실행 방법:
  1. dummy_shared_memory_writer.py 먼저 실행
  2. python radar_tcp_sender.py [--config cluster_config_example.json]
  3. Unity에서 Play

패킷 구조:
  [4B: packet_length] [4B: track_count] [ ID(4B) + x(4B) + y(4B) + z(4B) ] × N
"""

import argparse
import json
import math
import mmap
import socket
import struct
import time
from dataclasses import dataclass, fields
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

# ──────────────────────────────────────────────
# Shared Memory 상수 (cluster_radar_shared_memory.py 와 동일)
# ──────────────────────────────────────────────
SHM_NAME = r"Local\RadarPointCloudSharedMemory"

MAGIC   = 0x52444350
VERSION = 1

MAX_POINTS  = 65536
MAX_TARGETS = 256

HEADER_STRUCT = struct.Struct("<IIIIIIIIIIIIQII16I")
POINT_STRUCT  = struct.Struct("<fffffi")
TARGET_STRUCT = struct.Struct("<ffIIffffff")

HEADER_SIZE    = HEADER_STRUCT.size
POINT_SIZE     = POINT_STRUCT.size
TARGET_SIZE    = TARGET_STRUCT.size
POINTS_OFFSET  = HEADER_SIZE
TARGETS_OFFSET = POINTS_OFFSET + MAX_POINTS * POINT_SIZE
TOTAL_SIZE     = TARGETS_OFFSET + MAX_TARGETS * TARGET_SIZE

STATE_STOPPED   = 0
STATE_RECEIVING = 1
STATE_PAUSED    = 2
STATE_ERROR     = 3

Point = Tuple[float, float, float, float, float, int]
Vec3  = Tuple[float, float, float]

# ──────────────────────────────────────────────
# TCP 설정
# ──────────────────────────────────────────────
TCP_HOST = "127.0.0.1"
TCP_PORT = 9000


# ──────────────────────────────────────────────
# 데이터 클래스 (cluster_radar_shared_memory.py 에서 복사)
# ──────────────────────────────────────────────
@dataclass(frozen=True)
class RadarFrame:
    state:       int
    frame_count: int
    timestamp_ns: int
    points:  List[Point]
    targets: List["RadarTarget"]


@dataclass(frozen=True)
class RadarTarget:
    x: float; y: float; status: int; target_id: int
    minx: float; maxx: float; miny: float; maxy: float; minz: float; maxz: float

    @property
    def box_center(self) -> Vec3:
        return (
            (self.minx + self.maxx) * 0.5,
            (self.miny + self.maxy) * 0.5,
            (self.minz + self.maxz) * 0.5,
        )


@dataclass(frozen=True)
class Detection:
    center:          Vec3
    box_min:         Vec3
    box_max:         Vec3
    doppler:         Optional[float]
    radar_target_id: Optional[int]
    point_count:     int
    source:          str

    @property
    def box_size(self) -> Vec3:
        return (
            self.box_max[0] - self.box_min[0],
            self.box_max[1] - self.box_min[1],
            self.box_max[2] - self.box_min[2],
        )


@dataclass
class Track:
    track_id:               int
    radar_target_id:        Optional[int]
    state:                  List[float]
    covariance:             List[List[float]]
    missed:                 int   = 0
    hits:                   int   = 1
    last_frame:             int   = 0
    last_observed_center:   Vec3  = (0.0, 0.0, 0.0)
    exit_candidate:         bool  = False
    last_exit_outward_speed: float = 0.0
    radar_id_confidence:    float = 0.0
    radar_id_candidate:     Optional[int] = None
    radar_id_candidate_hits: int  = 0

    @property
    def center(self) -> Vec3:
        return (self.state[0], self.state[1], self.state[2])

    @property
    def output_id(self) -> int:
        return self.radar_target_id if self.radar_target_id is not None else self.track_id


@dataclass
class TrackerConfig:
    dbscan_eps:                          float = 0.35
    dbscan_min_samples:                  int   = 8
    max_assignment_distance:             float = 0.75
    max_mahalanobis_distance:            float = 4.0
    exit_missed_frames:                  int   = 8
    process_noise:                       float = 0.8
    measurement_noise:                   float = 0.05
    doppler_noise:                       float = 0.35
    doppler_sign:                        float = 1.0
    power_threshold:                     float = 0.0
    max_points:                          int   = 12000
    center_method:                       str   = "weighted"
    min_detection_points:                int   = 0
    human_min_width:                     float = 0.0
    human_max_width:                     float = 1.6
    human_min_depth:                     float = 0.0
    human_max_depth:                     float = 1.8
    human_min_height:                    float = 0.0
    human_max_height:                    float = 2.4
    prefer_radar_targets:                bool  = True
    radar_id_match_scale:                float = 0.25
    radar_id_mismatch_penalty:           float = 2.0
    radar_id_switch_hits:                int   = 3
    radar_id_confidence_up:              float = 0.2
    radar_id_confidence_down:            float = 0.1
    output_radar_id:                     bool  = False
    output_lost_tracks:                  bool  = True
    exit_door_enabled:                   bool  = False
    exit_door_x_min:                     float = 0.0
    exit_door_x_max:                     float = 0.0
    exit_door_y_min:                     float = 0.0
    exit_door_y_max:                     float = 0.0
    exit_door_z_min:                     float = -10.0
    exit_door_z_max:                     float = 10.0
    exit_door_outward_x:                 float = -1.0
    exit_door_outward_y:                 float = 0.0
    exit_door_outward_z:                 float = 0.0
    exit_door_boundary_margin:           float = 0.25
    exit_door_min_outward_speed:         float = 0.15
    exit_door_require_outward_motion:    bool  = True
    exit_door_require_boundary_proximity: bool = True


# ──────────────────────────────────────────────
# Shared Memory 읽기
# ──────────────────────────────────────────────
def open_shared_memory() -> mmap.mmap:
    while True:
        try:
            return mmap.mmap(-1, TOTAL_SIZE, tagname=SHM_NAME, access=mmap.ACCESS_READ)
        except OSError as e:
            print(f"[shm] Shared Memory 대기 중: {e}")
            time.sleep(1.0)


def read_frame(mm: mmap.mmap) -> Optional[RadarFrame]:
    while True:
        header = HEADER_STRUCT.unpack_from(mm, 0)
        magic, version, header_size, total_size, sequence = header[:5]

        if magic != MAGIC or version != VERSION:
            return None
        if header_size != HEADER_SIZE or total_size != TOTAL_SIZE:
            raise RuntimeError("Shared Memory layout mismatch")
        if sequence & 1:           # 쓰기 중 → 재시도
            time.sleep(0)
            continue

        state         = header[5]
        frame_count   = header[6]
        timestamp_ns  = header[12]
        point_count   = min(header[8], MAX_POINTS)
        target_count  = min(header[9], MAX_TARGETS)
        points_offset = header[13]
        targets_offset= header[14]

        points = [
            POINT_STRUCT.unpack_from(mm, points_offset + i * POINT_SIZE)
            for i in range(point_count)
        ]
        targets_raw = [
            TARGET_STRUCT.unpack_from(mm, targets_offset + i * TARGET_SIZE)
            for i in range(target_count)
        ]
        targets = [RadarTarget(*t) for t in targets_raw]

        if HEADER_STRUCT.unpack_from(mm, 0)[4] == sequence:
            return RadarFrame(
                state=state,
                frame_count=frame_count,
                timestamp_ns=timestamp_ns,
                points=points,
                targets=targets,
            )


# ──────────────────────────────────────────────
# ClusterTracker (cluster_radar_shared_memory.py 핵심 로직 인라인)
# ──────────────────────────────────────────────
class ClusterTracker:
    def __init__(self, config: TrackerConfig):
        self.config = config
        self.tracks: Dict[int, Track] = {}
        self.next_track_id = 1
        self.last_timestamp_ns: Optional[int] = None

    def update(self, frame: RadarFrame) -> List[Tuple[int, float, float, float]]:
        dt = self._compute_dt(frame.timestamp_ns)
        detections = build_detections(frame, self.config)

        for track in self.tracks.values():
            kalman_predict(track, dt, self.config.process_noise)

        assignments, unmatched_tracks, unmatched_detections = assign_tracks(
            list(self.tracks.values()), detections,
            self.config.max_assignment_distance,
            self.config.max_mahalanobis_distance,
            self.config.measurement_noise,
            self.config.radar_id_match_scale,
            self.config.radar_id_mismatch_penalty,
        )

        track_list = list(self.tracks.values())
        for ti, di in assignments:
            track = track_list[ti]
            det   = detections[di]
            kalman_update_position(track, det.center, self.config.measurement_noise)
            if det.doppler is not None:
                kalman_update_doppler(track, det.center,
                                      det.doppler * self.config.doppler_sign,
                                      self.config.doppler_noise)
            update_track_radar_id(track, det, self.config)
            track.missed = 0
            track.hits  += 1
            track.last_frame = frame.frame_count
            track.last_observed_center = det.center
            update_exit_candidate(track, self.config)

        for ti in unmatched_tracks:
            track_list[ti].missed += 1
            track_list[ti].radar_id_confidence = max(
                0.0, track_list[ti].radar_id_confidence - self.config.radar_id_confidence_down)

        for di in unmatched_detections:
            self._create_track(detections[di], frame.frame_count)

        self._drop_stale_tracks()
        return self._to_track_list()

    def _to_track_list(self) -> List[Tuple[int, float, float, float]]:
        """Unity 전송용: [(id, x, y, z), ...] 형태로 반환"""
        result = []
        for track in sorted(self.tracks.values(), key=lambda t: t.track_id):
            if not self.config.output_lost_tracks and track.missed > 0:
                continue
            tid = track.radar_target_id if (
                self.config.output_radar_id and track.radar_target_id is not None
            ) else track.track_id
            x, y, z = track.center
            result.append((tid, x, y, z))
        return result

    def _compute_dt(self, timestamp_ns: int) -> float:
        if self.last_timestamp_ns is None:
            self.last_timestamp_ns = timestamp_ns
            return 1.0 / 30.0
        dt = (timestamp_ns - self.last_timestamp_ns) * 1e-9
        self.last_timestamp_ns = timestamp_ns
        return max(dt, 1e-6)

    def _create_track(self, det: Detection, frame_count: int):
        cx, cy, cz = det.center
        state = [cx, cy, cz, 0.0, 0.0, 0.0]
        cov   = [[1.0 if i == j else 0.0 for j in range(6)] for i in range(6)]
        track = Track(
            track_id=self.next_track_id,
            radar_target_id=det.radar_target_id,
            state=state,
            covariance=cov,
            last_frame=frame_count,
            last_observed_center=det.center,
        )
        self.tracks[self.next_track_id] = track
        self.next_track_id += 1

    def _drop_stale_tracks(self):
        stale = [tid for tid, t in self.tracks.items()
                 if t.missed >= self.config.exit_missed_frames]
        for tid in stale:
            del self.tracks[tid]


# ──────────────────────────────────────────────
# TCP 패킷 빌드 & 전송
# ──────────────────────────────────────────────
def build_packet(tracks: List[Tuple[int, float, float, float]]) -> bytes:
    """
    패킷 구조:
      [4B: packet_length] [4B: track_count] [16B: id+x+y+z] × N

    packet_length = 4 + 16 * N  (length 필드 자신은 미포함)
    """
    count  = len(tracks)
    body   = struct.pack("I", count)
    for (tid, x, y, z) in tracks:
        body += struct.pack("I3f", tid, x, y, z)
    return struct.pack("I", len(body)) + body   # length prefix 포함


def tcp_server_loop(config: TrackerConfig):
    """Unity가 접속하길 기다린 뒤 30fps로 트래킹 결과 전송"""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((TCP_HOST, TCP_PORT))
    server.listen(1)

    print("=" * 50)
    print("  레이더 TCP 브릿지 시작")
    print(f"  Unity 연결 대기 중... {TCP_HOST}:{TCP_PORT}")
    print("=" * 50)

    conn, addr = server.accept()
    print(f"  Unity 연결됨: {addr}")
    print("=" * 50)

    tracker  = ClusterTracker(config)
    mm       = open_shared_memory()
    last_frame: Optional[int] = None
    frame_num = 0

    try:
        while True:
            start = time.perf_counter()

            frame = read_frame(mm)
            if frame is None:
                print("[shm] producer 미초기화 — 대기 중")
                time.sleep(1.0)
                continue

            if frame.frame_count == last_frame:
                time.sleep(0.001)
                continue

            last_frame = frame.frame_count
            tracks     = tracker.update(frame)
            packet     = build_packet(tracks)

            try:
                conn.sendall(packet)
            except (BrokenPipeError, ConnectionResetError, OSError):
                print("[tcp] Unity 연결 끊김 — 재연결 대기")
                conn.close()
                conn, addr = server.accept()
                print(f"[tcp] Unity 재연결됨: {addr}")
                continue

            # 디버그 출력
            frame_num += 1
            print(f"\n[Frame {frame_num:05d}] shm_frame={frame.frame_count}  트래킹: {len(tracks)}개")
            for (tid, x, y, z) in tracks:
                print(f"  ID={tid:02d}  x={x:7.3f}  y={y:7.3f}  z={z:7.3f}")

            elapsed = time.perf_counter() - start
            sleep   = (1.0 / 30.0) - elapsed
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        print("\n[tcp] 종료")
    finally:
        mm.close()
        conn.close()
        server.close()


# ──────────────────────────────────────────────
# Config 로드 헬퍼
# ──────────────────────────────────────────────
def load_config(path: Optional[str]) -> TrackerConfig:
    if not path:
        return TrackerConfig()
    with open(path, encoding="utf-8") as f:
        raw = json.load(f)

    kw: Dict[str, Any] = {}
    def pull(section, mapping):
        sec = raw.get(section, {})
        for k, field in mapping.items():
            if k in sec:
                kw[field] = sec[k]

    pull("dbscan",       {"eps":"dbscan_eps","min_samples":"dbscan_min_samples","max_points":"max_points"})
    pull("tracking",     {"max_assignment_distance":"max_assignment_distance",
                          "max_mahalanobis_distance":"max_mahalanobis_distance",
                          "exit_missed_frames":"exit_missed_frames",
                          "process_noise":"process_noise","measurement_noise":"measurement_noise",
                          "doppler_noise":"doppler_noise","doppler_sign":"doppler_sign"})
    pull("radar_id",     {"prefer_targets":"prefer_radar_targets","match_scale":"radar_id_match_scale",
                          "mismatch_penalty":"radar_id_mismatch_penalty","switch_hits":"radar_id_switch_hits",
                          "confidence_up":"radar_id_confidence_up","confidence_down":"radar_id_confidence_down",
                          "output_radar_id":"output_radar_id","output_lost_tracks":"output_lost_tracks"})
    pull("human_filter", {"min_detection_points":"min_detection_points",
                          "min_width":"human_min_width","max_width":"human_max_width",
                          "min_depth":"human_min_depth","max_depth":"human_max_depth",
                          "min_height":"human_min_height","max_height":"human_max_height"})
    pull("exit_door",    {"enabled":"exit_door_enabled",
                          "x_min":"exit_door_x_min","x_max":"exit_door_x_max",
                          "y_min":"exit_door_y_min","y_max":"exit_door_y_max",
                          "z_min":"exit_door_z_min","z_max":"exit_door_z_max",
                          "outward_x":"exit_door_outward_x","outward_y":"exit_door_outward_y",
                          "outward_z":"exit_door_outward_z",
                          "boundary_margin":"exit_door_boundary_margin",
                          "min_outward_speed":"exit_door_min_outward_speed",
                          "require_outward_motion":"exit_door_require_outward_motion",
                          "require_boundary_proximity":"exit_door_require_boundary_proximity"})
    return TrackerConfig(**kw)


# ──────────────────────────────────────────────
# cluster_radar_shared_memory.py 에서 가져온 알고리즘 함수들
# (ClusterTracker 가 참조하는 순수 함수)
# ──────────────────────────────────────────────
def build_detections(frame: RadarFrame, cfg: TrackerConfig) -> List[Detection]:
    from collections import defaultdict
    import math

    filtered = [p for p in frame.points if p[4] >= cfg.power_threshold]
    if len(filtered) > cfg.max_points:
        filtered = filtered[:cfg.max_points]

    if not filtered:
        return []

    # target_id 기반 그룹핑 (prefer_radar_targets=True 일 때)
    if cfg.prefer_radar_targets and frame.targets:
        target_map: Dict[int, List[Point]] = defaultdict(list)
        for p in filtered:
            target_map[p[5]].append(p)

        detections = []
        for tgt in frame.targets:
            pts = target_map.get(tgt.target_id, [])
            if len(pts) < cfg.min_detection_points:
                continue
            cx, cy, cz = compute_center(pts, tgt, cfg.center_method)
            bmin = (tgt.minx, tgt.miny, tgt.minz)
            bmax = (tgt.maxx, tgt.maxy, tgt.maxz)
            w = tgt.maxx - tgt.minx
            d = tgt.maxy - tgt.miny
            h = tgt.maxz - tgt.minz
            if not (cfg.human_min_width  <= w <= cfg.human_max_width  and
                    cfg.human_min_depth  <= d <= cfg.human_max_depth  and
                    cfg.human_min_height <= h <= cfg.human_max_height):
                continue
            doppler = float(sum(p[3] for p in pts) / len(pts)) if pts else None
            detections.append(Detection(
                center=(cx, cy, cz), box_min=bmin, box_max=bmax,
                doppler=doppler, radar_target_id=tgt.target_id,
                point_count=len(pts), source="target",
            ))
        return detections

    # DBSCAN fallback
    return dbscan_detections(filtered, cfg)


def compute_center(pts: List[Point], tgt: RadarTarget, method: str) -> Vec3:
    if method == "box" or not pts:
        return tgt.box_center
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]; zs = [p[2] for p in pts]
    if method == "mean":
        return (sum(xs)/len(xs), sum(ys)/len(ys), sum(zs)/len(zs))
    # weighted by power
    powers = [p[4] for p in pts]
    total  = sum(powers) or 1.0
    return (
        sum(x*w for x,w in zip(xs,powers))/total,
        sum(y*w for y,w in zip(ys,powers))/total,
        sum(z*w for z,w in zip(zs,powers))/total,
    )


def dbscan_detections(pts: List[Point], cfg: TrackerConfig) -> List[Detection]:
    """간단한 DBSCAN 구현"""
    n   = len(pts)
    eps2 = cfg.dbscan_eps ** 2
    labels = [-1] * n
    cluster_id = 0

    def neighbors(idx):
        x0,y0,z0 = pts[idx][0], pts[idx][1], pts[idx][2]
        return [j for j in range(n)
                if (pts[j][0]-x0)**2+(pts[j][1]-y0)**2+(pts[j][2]-z0)**2 <= eps2]

    for i in range(n):
        if labels[i] != -1:
            continue
        nb = neighbors(i)
        if len(nb) < cfg.dbscan_min_samples:
            labels[i] = -2   # noise
            continue
        labels[i] = cluster_id
        seeds = set(nb) - {i}
        while seeds:
            j = seeds.pop()
            if labels[j] == -2:
                labels[j] = cluster_id
            if labels[j] != -1:
                continue
            labels[j] = cluster_id
            nb2 = neighbors(j)
            if len(nb2) >= cfg.dbscan_min_samples:
                seeds.update(nb2)
        cluster_id += 1

    detections = []
    for cid in range(cluster_id):
        cluster = [pts[i] for i in range(n) if labels[i] == cid]
        if len(cluster) < cfg.min_detection_points:
            continue
        xs=[p[0] for p in cluster]; ys=[p[1] for p in cluster]; zs=[p[2] for p in cluster]
        bmin=(min(xs),min(ys),min(zs)); bmax=(max(xs),max(ys),max(zs))
        w=bmax[0]-bmin[0]; d=bmax[1]-bmin[1]; h=bmax[2]-bmin[2]
        if not (cfg.human_min_width  <= w <= cfg.human_max_width  and
                cfg.human_min_depth  <= d <= cfg.human_max_depth  and
                cfg.human_min_height <= h <= cfg.human_max_height):
            continue
        cx=(bmin[0]+bmax[0])*0.5; cy=(bmin[1]+bmax[1])*0.5; cz=(bmin[2]+bmax[2])*0.5
        doppler = sum(p[3] for p in cluster)/len(cluster)
        detections.append(Detection(
            center=(cx,cy,cz), box_min=bmin, box_max=bmax,
            doppler=doppler, radar_target_id=None,
            point_count=len(cluster), source="dbscan",
        ))
    return detections


def kalman_predict(track: Track, dt: float, process_noise: float):
    s = track.state
    s[0] += s[3]*dt; s[1] += s[4]*dt; s[2] += s[5]*dt
    q = process_noise * dt
    for i in range(6):
        track.covariance[i][i] += q


def kalman_update_position(track: Track, center: Vec3, noise: float):
    for i in range(3):
        k = track.covariance[i][i] / (track.covariance[i][i] + noise)
        track.state[i]         += k * (center[i] - track.state[i])
        track.covariance[i][i] *= (1.0 - k)


def kalman_update_doppler(track: Track, center: Vec3, doppler: float, noise: float):
    cx,cy,cz = center
    dx=cx-track.state[0]; dy=cy-track.state[1]; dz=cz-track.state[2]
    dist = math.sqrt(dx*dx+dy*dy+dz*dz) or 1.0
    hx,hy,hz = dx/dist, dy/dist, dz/dist
    predicted = hx*track.state[3]+hy*track.state[4]+hz*track.state[5]
    innov = doppler - predicted
    for i,h in enumerate([hx,hy,hz]):
        k = track.covariance[i+3][i+3]*h / (track.covariance[i+3][i+3]*h*h + noise)
        track.state[i+3]         += k * innov
        track.covariance[i+3][i+3] *= (1.0 - k*h)


def assign_tracks(tracks, detections, max_dist, max_maha, meas_noise,
                  id_match_scale, id_mismatch_penalty):
    if not tracks or not detections:
        return [], list(range(len(tracks))), list(range(len(detections)))

    cost = []
    for t in tracks:
        row = []
        for d in detections:
            dist = math.sqrt(sum((a-b)**2 for a,b in zip(t.center, d.center)))
            penalty = 0.0
            if (t.radar_target_id is not None and d.radar_target_id is not None
                    and t.radar_target_id != d.radar_target_id):
                penalty = id_mismatch_penalty
            row.append(dist + penalty * id_match_scale)
        cost.append(row)

    # 헝가리안 알고리즘 대신 greedy 매칭 (충분히 빠름)
    assigned_t, assigned_d = set(), set()
    assignments = []
    pairs = sorted(((cost[ti][di], ti, di)
                    for ti in range(len(tracks))
                    for di in range(len(detections))), key=lambda x: x[0])
    for c, ti, di in pairs:
        if c > max_dist: break
        if ti in assigned_t or di in assigned_d: continue
        assignments.append((ti, di))
        assigned_t.add(ti); assigned_d.add(di)

    unmatched_t = [i for i in range(len(tracks))     if i not in assigned_t]
    unmatched_d = [i for i in range(len(detections)) if i not in assigned_d]
    return assignments, unmatched_t, unmatched_d


def update_track_radar_id(track: Track, det: Detection, cfg: TrackerConfig):
    if det.radar_target_id is None:
        return
    if track.radar_target_id == det.radar_target_id:
        track.radar_id_confidence = min(1.0, track.radar_id_confidence + cfg.radar_id_confidence_up)
        return
    if track.radar_id_candidate == det.radar_target_id:
        track.radar_id_candidate_hits += 1
    else:
        track.radar_id_candidate      = det.radar_target_id
        track.radar_id_candidate_hits = 1
    if track.radar_id_candidate_hits >= cfg.radar_id_switch_hits:
        track.radar_target_id         = track.radar_id_candidate
        track.radar_id_candidate      = None
        track.radar_id_candidate_hits = 0
        track.radar_id_confidence     = cfg.radar_id_confidence_up


def update_exit_candidate(track: Track, cfg: TrackerConfig):
    if not cfg.exit_door_enabled:
        return
    cx,cy,cz = track.center
    in_zone = (cfg.exit_door_x_min <= cx <= cfg.exit_door_x_max and
               cfg.exit_door_y_min <= cy <= cfg.exit_door_y_max and
               cfg.exit_door_z_min <= cz <= cfg.exit_door_z_max)
    track.exit_candidate = in_zone


# ──────────────────────────────────────────────
# Entry Point
# ──────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Radar Shared Memory → TCP → Unity")
    parser.add_argument("--config", default=None, help="cluster_config_example.json 경로")
    parser.add_argument("--host",   default=TCP_HOST)
    parser.add_argument("--port",   default=TCP_PORT, type=int)
    args = parser.parse_args()

    global TCP_HOST, TCP_PORT
    TCP_HOST = args.host
    TCP_PORT = args.port

    config = load_config(args.config)
    tcp_server_loop(config)


if __name__ == "__main__":
    main()
