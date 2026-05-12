#!/usr/bin/env python3
"""
jitter_imu_lidar_ros2.py — Jitter IMU e LiDAR via ROS2 (Unitree G1)
====================================================================
Mede jitter de:
  1. IMU   → extraído de /lowstate       (unitree_hg/LowState)
  2. LiDAR → /utlidar/cloud_livox_mid360 (sensor_msgs/PointCloud2)

O ambiente ROS2 + workspace Unitree é carregado automaticamente.

Uso:
  python3 jitter_imu_lidar_ros2.py --duration 120 --output ~/data/jitter

  # Somente IMU
  python3 jitter_imu_lidar_ros2.py --skip-lidar --duration 60

  # Ambos em paralelo
  python3 jitter_imu_lidar_ros2.py --parallel

  # Tópico LiDAR customizado
  python3 jitter_imu_lidar_ros2.py --lidar-topic /utlidar/cloud_livox_mid360
"""

# ──────────────────────────────────────────────────────────────────────────────
# Ambiente ROS2 + Unitree (deve vir ANTES de qualquer import rclpy/unitree)
# ──────────────────────────────────────────────────────────────────────────────
import os, sys

os.environ.setdefault('AMENT_PREFIX_PATH',
    '/home/u7/unitree_ros2/install/unitree_hg:'
    '/home/u7/unitree_ros2/install/unitree_go:'
    '/home/u7/unitree_ros2/install/unitree_api:'
    '/opt/ros/jazzy')
os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')

sys.path.insert(0, '/opt/ros/jazzy/lib/python3.12/site-packages')
sys.path.insert(0, '/home/u7/unitree_ros2/install/unitree_hg/local/lib/python3.12/dist-packages')
sys.path.insert(0, '/home/u7/unitree_ros2/install/unitree_go/local/lib/python3.12/dist-packages')
sys.path.insert(0, '/home/u7/unitree_ros2/install/unitree_api/local/lib/python3.12/dist-packages')

# ──────────────────────────────────────────────────────────────────────────────
# Imports padrão
# ──────────────────────────────────────────────────────────────────────────────
import argparse
import csv
import math
import threading
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                           DurabilityPolicy, HistoryPolicy)
    from sensor_msgs.msg import PointCloud2
    from unitree_hg.msg import LowState
except ImportError as e:
    print(f'[ERRO] Dependência não encontrada: {e}')
    print('  Certifique-se de ter compilado o workspace unitree_ros2.')
    sys.exit(1)


# ──────────────────────────────────────────────────────────────────────────────
# QoS profiles
# ──────────────────────────────────────────────────────────────────────────────

QOS_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

QOS_BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


# ──────────────────────────────────────────────────────────────────────────────
# Utilitários
# ──────────────────────────────────────────────────────────────────────────────

def mono() -> float:
    return time.monotonic()


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def stats(values):
    v = [x for x in values if not math.isnan(x)]
    if not v:
        return None
    v.sort()
    n   = len(v)
    avg = sum(v) / n
    std = math.sqrt(sum((x - avg) ** 2 for x in v) / n)
    return {
        'n':    n,
        'min':  v[0],
        'max':  v[-1],
        'mean': avg,
        'std':  std,
        'p50':  v[int(n * 0.50)],
        'p95':  v[int(n * 0.95)],
        'p99':  v[int(n * 0.99)],
    }


def print_stats(label: str, s: dict):
    if not s:
        print(f'  {label}: sem dados suficientes (mínimo 2 amostras)')
        return
    print(f'  {label}')
    print(f'    n={s["n"]}')
    print(f'    min={s["min"]*1e3:.3f} ms    max={s["max"]*1e3:.3f} ms')
    print(f'    mean={s["mean"]*1e3:.3f} ms  std={s["std"]*1e3:.3f} ms')
    print(f'    p50={s["p50"]*1e3:.3f} ms    p95={s["p95"]*1e3:.3f} ms    p99={s["p99"]*1e3:.3f} ms')


def save_csv(path: str, fields: list, rows: list):
    os.makedirs(os.path.dirname(path) if os.path.dirname(path) else '.', exist_ok=True)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    print(f'  CSV salvo: {path}  ({len(rows)} amostras)')


def progress_loop(bufs_labels: list, duration: float,
                  stop_event: threading.Event):
    """
    bufs_labels: lista de (buf, lock, label) para exibição simultânea.
    """
    t0 = mono()
    try:
        while not stop_event.is_set():
            elapsed = mono() - t0
            if duration > 0 and elapsed >= duration:
                break
            pct = f'{elapsed/duration*100:.0f}%' if duration > 0 else '∞'
            parts = []
            for buf, lock, label in bufs_labels:
                with lock:
                    n = len(buf)
                parts.append(f'{label}: {n:6d}')
            print(f'  [{elapsed:6.1f}s / {pct}]  ' + '   '.join(parts),
                  end='\r', flush=True)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print('\n  Interrompido pelo usuário.')
        stop_event.set()
    print()


# ──────────────────────────────────────────────────────────────────────────────
# Fase 1 — IMU via /lowstate  (unitree_hg/LowState)
# ──────────────────────────────────────────────────────────────────────────────

class ImuJitterNode(Node):
    """
    Extrai dados de IMU do tópico /lowstate (LowState da Unitree).
    Mede jitter de chegada (wall-clock) e jitter de tick.
    """

    def __init__(self, topic: str, buf: list, lock: threading.Lock,
                 stop_event: threading.Event):
        super().__init__('imu_jitter_node')
        self._buf   = buf
        self._lock  = lock
        self._stop  = stop_event
        self._last_recv = None
        self._last_tick = None

        self.create_subscription(LowState, topic, self._callback, QOS_RELIABLE)
        self.get_logger().info(f'Assinando IMU via LowState: {topic}')

    def _callback(self, msg: LowState):
        if self._stop.is_set():
            return

        t_recv = mono()
        tick   = msg.tick           # contador interno do robô (uint32, ms)
        acc    = msg.imu_state.accelerometer   # float[3]
        gyr    = msg.imu_state.gyroscope       # float[3]
        rpy    = msg.imu_state.rpy             # float[3]  (roll/pitch/yaw)
        quat   = msg.imu_state.quaternion      # float[4]

        with self._lock:
            inter_recv = (t_recv - self._last_recv) if self._last_recv is not None else float('nan')
            # tick em ms → converte para segundos para manter consistência
            inter_tick = ((tick - self._last_tick) * 1e-3) if self._last_tick is not None else float('nan')
            self._last_recv = t_recv
            self._last_tick = tick

            self._buf.append({
                't_recv':     t_recv,
                'tick':       tick,
                'inter_recv': inter_recv,   # jitter wall-clock (transporte DDS)
                'inter_tick': inter_tick,   # jitter declarado pelo robô (tick)
                'acc_x': acc[0], 'acc_y': acc[1], 'acc_z': acc[2],
                'gyr_x': gyr[0], 'gyr_y': gyr[1], 'gyr_z': gyr[2],
                'roll':  rpy[0], 'pitch': rpy[1], 'yaw':   rpy[2],
                'qw': quat[0],  'qx': quat[1], 'qy': quat[2], 'qz': quat[3],
            })


def run_imu(args):
    print('╔══════════════════════════════════════════════════════════════╗')
    print(f'║  FASE 1 — IMU via /lowstate  (unitree_hg/LowState)        ║')
    print('╚══════════════════════════════════════════════════════════════╝')

    buf  = []
    lock = threading.Lock()
    stop = threading.Event()

    node     = ImuJitterNode('/lowstate', buf, lock, stop)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_t = threading.Thread(target=executor.spin, daemon=True)
    spin_t.start()

    progress_loop([(buf, lock, 'amostras IMU')], args.duration, stop)

    stop.set()
    executor.shutdown(timeout_sec=1.0)
    node.destroy_node()
    spin_t.join(timeout=2.0)

    with lock:
        snap = list(buf)

    if not snap:
        print('  Nenhum dado recebido em /lowstate.')
        print('  Verifique:  ros2 topic hz /lowstate')
        return

    print('  ─────────────────────────────────────────────────────────────')
    print_stats('inter_recv  (jitter transporte ROS2/DDS)', stats([r['inter_recv'] for r in snap]))
    print()
    print_stats('inter_tick  (jitter tick do robô, em ms→s)', stats([r['inter_tick'] for r in snap]))
    print()

    fields = ['t_recv', 'tick', 'inter_recv', 'inter_tick',
              'acc_x', 'acc_y', 'acc_z',
              'gyr_x', 'gyr_y', 'gyr_z',
              'roll', 'pitch', 'yaw',
              'qw', 'qx', 'qy', 'qz']
    save_csv(os.path.join(args.output, 'imu_jitter.csv'), fields, snap)


# ──────────────────────────────────────────────────────────────────────────────
# Fase 2 — LiDAR  (sensor_msgs/PointCloud2)
# ──────────────────────────────────────────────────────────────────────────────

class LidarJitterNode(Node):
    """Mede jitter de chegada e de cabeçalho de scans LiDAR (PointCloud2)."""

    def __init__(self, topic: str, buf: list, lock: threading.Lock,
                 stop_event: threading.Event):
        super().__init__('lidar_jitter_node')
        self._buf   = buf
        self._lock  = lock
        self._stop  = stop_event
        self._last_recv  = None
        self._last_stamp = None

        self.create_subscription(PointCloud2, topic, self._callback, QOS_BEST_EFFORT)
        self.get_logger().info(f'Assinando LiDAR: {topic}')

    def _callback(self, msg: PointCloud2):
        if self._stop.is_set():
            return

        t_recv  = mono()
        stamp_s = stamp_to_sec(msg.header.stamp)

        try:
            n_points = (msg.row_step * msg.height) // msg.point_step
        except ZeroDivisionError:
            n_points = -1

        with self._lock:
            inter_recv  = (t_recv  - self._last_recv)  if self._last_recv  is not None else float('nan')
            inter_stamp = (stamp_s - self._last_stamp) if self._last_stamp is not None else float('nan')
            self._last_recv  = t_recv
            self._last_stamp = stamp_s

            self._buf.append({
                't_recv':      t_recv,
                'stamp_s':     stamp_s,
                'inter_recv':  inter_recv,
                'inter_stamp': inter_stamp,
                'n_points':    n_points,
            })


def run_lidar(args):
    print('╔══════════════════════════════════════════════════════════════╗')
    print(f'║  FASE 2 — LiDAR  ({args.lidar_topic:<36s})  ║')
    print('╚══════════════════════════════════════════════════════════════╝')

    buf  = []
    lock = threading.Lock()
    stop = threading.Event()

    node     = LidarJitterNode(args.lidar_topic, buf, lock, stop)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_t = threading.Thread(target=executor.spin, daemon=True)
    spin_t.start()

    progress_loop([(buf, lock, 'scans LiDAR')], args.duration, stop)

    stop.set()
    executor.shutdown(timeout_sec=1.0)
    node.destroy_node()
    spin_t.join(timeout=2.0)

    with lock:
        snap = list(buf)

    if not snap:
        print(f'  Nenhum dado recebido em {args.lidar_topic}.')
        print('  Verifique:  ros2 topic hz ' + args.lidar_topic)
        return

    pts     = [r['n_points'] for r in snap if r['n_points'] >= 0]
    avg_pts = sum(pts) / len(pts) if pts else float('nan')

    print('  ─────────────────────────────────────────────────────────────')
    print_stats('inter_recv   (jitter transporte ROS2/DDS)', stats([r['inter_recv']  for r in snap]))
    print()
    print_stats('inter_stamp  (jitter timestamp do sensor)', stats([r['inter_stamp'] for r in snap]))
    if not math.isnan(avg_pts):
        print(f'\n  Pontos por scan: média = {avg_pts:.0f}')
    print()

    fields = ['t_recv', 'stamp_s', 'inter_recv', 'inter_stamp', 'n_points']
    save_csv(os.path.join(args.output, 'lidar_jitter.csv'), fields, snap)


# ──────────────────────────────────────────────────────────────────────────────
# Modo paralelo — IMU + LiDAR simultaneamente
# ──────────────────────────────────────────────────────────────────────────────

def run_parallel(args):
    print('╔══════════════════════════════════════════════════════════════╗')
    print('║  MODO PARALELO — IMU + LiDAR simultaneamente               ║')
    print('╚══════════════════════════════════════════════════════════════╝')

    imu_buf,   imu_lock,   imu_stop   = [], threading.Lock(), threading.Event()
    lidar_buf, lidar_lock, lidar_stop = [], threading.Lock(), threading.Event()

    imu_node   = ImuJitterNode('/lowstate',        imu_buf,   imu_lock,   imu_stop)
    lidar_node = LidarJitterNode(args.lidar_topic, lidar_buf, lidar_lock, lidar_stop)

    executor = SingleThreadedExecutor()
    executor.add_node(imu_node)
    executor.add_node(lidar_node)

    spin_t = threading.Thread(target=executor.spin, daemon=True)
    spin_t.start()

    stop = threading.Event()
    progress_loop(
        [(imu_buf, imu_lock, 'IMU'), (lidar_buf, lidar_lock, 'LiDAR')],
        args.duration, stop
    )

    imu_stop.set()
    lidar_stop.set()
    executor.shutdown(timeout_sec=1.0)
    imu_node.destroy_node()
    lidar_node.destroy_node()
    spin_t.join(timeout=2.0)

    # ── IMU
    print('\n── Resultados IMU (/lowstate) ──────────────────────────────────')
    with imu_lock:
        imu_snap = list(imu_buf)
    if imu_snap:
        print_stats('inter_recv  (jitter transporte)', stats([r['inter_recv'] for r in imu_snap]))
        print()
        print_stats('inter_tick  (jitter tick robô)', stats([r['inter_tick'] for r in imu_snap]))
        save_csv(os.path.join(args.output, 'imu_jitter.csv'),
                 ['t_recv', 'tick', 'inter_recv', 'inter_tick',
                  'acc_x', 'acc_y', 'acc_z', 'gyr_x', 'gyr_y', 'gyr_z',
                  'roll', 'pitch', 'yaw', 'qw', 'qx', 'qy', 'qz'],
                 imu_snap)
    else:
        print('  Nenhum dado em /lowstate')

    # ── LiDAR
    print('\n── Resultados LiDAR ────────────────────────────────────────────')
    with lidar_lock:
        lidar_snap = list(lidar_buf)
    if lidar_snap:
        print_stats('inter_recv  (jitter transporte)', stats([r['inter_recv']  for r in lidar_snap]))
        print()
        print_stats('inter_stamp (jitter sensor)',     stats([r['inter_stamp'] for r in lidar_snap]))
        pts = [r['n_points'] for r in lidar_snap if r['n_points'] >= 0]
        if pts:
            print(f'\n  Pontos por scan: média = {sum(pts)/len(pts):.0f}')
        save_csv(os.path.join(args.output, 'lidar_jitter.csv'),
                 ['t_recv', 'stamp_s', 'inter_recv', 'inter_stamp', 'n_points'],
                 lidar_snap)
    else:
        print(f'  Nenhum dado em {args.lidar_topic}')



# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description='Mede jitter de IMU (via /lowstate) e LiDAR via ROS2 — Unitree G1',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument('--duration',    type=float, default=120.0,
                    help='Segundos de coleta por sensor (0 = infinito)')
    ap.add_argument('--output',      type=str,   default='.',
                    help='Diretório de saída para os CSV')
    ap.add_argument('--lidar-topic', type=str,   default='/utlidar/cloud_livox_mid360',
                    help='Tópico ROS2 para LiDAR (sensor_msgs/PointCloud2)')
    ap.add_argument('--skip-imu',    action='store_true', help='Pula fase IMU')
    ap.add_argument('--skip-lidar',  action='store_true', help='Pula fase LiDAR')
    ap.add_argument('--parallel',    action='store_true',
                    help='Coleta IMU e LiDAR simultaneamente')
    args = ap.parse_args()

    os.makedirs(args.output, exist_ok=True)

    # Suprime os WARNs do rmw_cyclonedds durante o init
    old_fd = os.dup(2)
    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull_fd, 2)

    rclpy.init()

    # Restaura stderr
    os.dup2(old_fd, 2)
    os.close(old_fd)
    os.close(devnull_fd)

    print(f'  IMU tópico  = /lowstate  (unitree_hg/LowState)')
    print(f'  LiDAR tópico= {args.lidar_topic}')
    print(f'  duration    = {args.duration}s')
    print(f'  output      = {args.output}')
    print()

    try:
        if args.parallel and not args.skip_imu and not args.skip_lidar:
            run_parallel(args)
        else:
            if not args.skip_imu:
                run_imu(args)
                print()
                if not args.skip_lidar:
                    print('  Aguardando 2s antes de iniciar o LiDAR...')
                    time.sleep(2)
                    print()
            if not args.skip_lidar:
                run_lidar(args)

    except KeyboardInterrupt:
        print('\n  Encerrado pelo usuário.')
    finally:
        rclpy.shutdown()

    print()
    print('═══════════════════════════════════════════════════════════════')
    print('  Concluído.')
    print('═══════════════════════════════════════════════════════════════')


if __name__ == '__main__':
    main()
