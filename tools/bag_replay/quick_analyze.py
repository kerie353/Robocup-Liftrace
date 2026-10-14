#!/usr/bin/env python3
"""快速 bag 解析 — 直接打印降落阶段关键数据。Windows Python 可用，纯 Python + lz4。"""
import struct, sys, json, os
from collections import defaultdict
import lz4.block

BAG = sys.argv[1]
if not BAG or not os.path.isfile(BAG):
    print("Usage: python quick_analyze.py <bag>")
    sys.exit(1)

# 只解析这些话题的消息体
TARGET_TOPICS = {
    '/mavros/local_position/odom': ('odom', None),
    '/navigation/local_odom': ('odom', None),
    '/mavros/local_position/pose': ('pose', None),
    '/navigation/local_pose': ('pose', None),
    '/mavros/state': ('state', None),
    '/mavros/extended_state': ('extended_state', None),
    '/navigation/mission_status': ('string', None),
    '/navigation/mission_command_raw': ('string', None),
    '/navigation/mission_result': ('string', None),
    '/patrol_control/external_landing_handoff': ('string', None),
    '/mavros/setpoint_position/local': ('pose', None),
    '/rosout_agg': ('rosout', None),
}

def read_header(f):
    header_len = struct.unpack('<I', f.read(4))[0]
    end = f.tell() + header_len
    fields = {}
    while f.tell() < end:
        if f.tell() + 4 > end: break
        kv_len = struct.unpack('<I', f.read(4))[0]
        if kv_len == 0 or f.tell() + kv_len > end: break
        kv_data = f.read(kv_len)
        eq = kv_data.find(b'=')
        if eq > 0:
            fields[kv_data[:eq].decode('utf-8', 'replace')] = kv_data[eq+1:]
    f.seek(end)
    return fields

def skip_records(buf, pos, count):
    """快速跳过 index records。"""
    for _ in range(count):
        if pos + 13 > len(buf): return pos
        idx_hdr_len = struct.unpack('<I', buf[pos:pos+4])[0]
        pos += 4 + idx_hdr_len
        if pos + 4 > len(buf): break
        num_deltas = struct.unpack('<I', buf[pos:pos+4])[0]
        pos += 4 + num_deltas * 12
    return pos

# ---------- 消息解析 ----------
def parse_stamp(data, pos):
    secs, nsecs = struct.unpack('<II', data[pos:pos+8])
    return secs + nsecs * 1e-9, pos + 8

def parse_odom_body(data):
    pos = 0
    pos += 4  # seq
    _, pos = parse_stamp(data, pos)
    fl = struct.unpack('<I', data[pos:pos+4])[0]; pos += 4 + fl  # frame
    cl = struct.unpack('<I', data[pos:pos+4])[0]; pos += 4 + cl  # child
    x, y, z = struct.unpack('<ddd', data[pos:pos+24]); pos += 24
    pos += 32  # quaternion
    pos += 72  # pose covariance
    vx, vy, vz = struct.unpack('<ddd', data[pos:pos+24]); pos += 24
    ax, ay, az = struct.unpack('<ddd', data[pos:pos+24])
    return {'x': x, 'y': y, 'z': z, 'vx': vx, 'vy': vy, 'vz': vz}

def parse_pose_body(data):
    pos = 0
    pos += 4  # seq
    _, pos = parse_stamp(data, pos)
    fl = struct.unpack('<I', data[pos:pos+4])[0]; pos += 4 + fl
    x, y, z = struct.unpack('<ddd', data[pos:pos+24])
    return {'x': x, 'y': y, 'z': z}

def parse_state_body(data):
    pos = 0
    def b():
        nonlocal pos
        v = data[pos] > 0; pos += 1; return v
    connected = b(); armed = b(); guided = b(); manual_input = b()
    ml = struct.unpack('<I', data[pos:pos+4])[0]; pos += 4
    mode = data[pos:pos+ml].decode('utf-8', 'replace'); pos += ml
    return {'connected': connected, 'armed': armed, 'mode': mode}

def parse_extended_body(data):
    return {'landed_state': data[0]}

def parse_string_body(data):
    l = struct.unpack('<I', data[:4])[0]
    return {'data': data[4:4+l].decode('utf-8', 'replace')}

def parse_rosout_body(data):
    pos = 4  # skip seq
    _, pos = parse_stamp(data, pos)
    fl = struct.unpack('<I', data[pos:pos+4])[0]; pos += 4 + fl
    level = data[pos]; pos += 1
    def rs():
        nonlocal pos
        l = struct.unpack('<I', data[pos:pos+4])[0]; pos += 4
        s = data[pos:pos+l].decode('utf-8', 'replace'); pos += l
        return s
    msg = rs()
    return {'level': level, 'msg': msg}

# ---------- 主解析 ----------
def main():
    results = defaultdict(list)
    total_odom = 0

    with open(BAG, 'rb') as f:
        magic = f.read(13)
        assert magic == b'#ROSBAG V2.0\n'
        fh_len = struct.unpack('<I', f.read(4))[0]
        fh_end = f.tell() + fh_len
        fh = {}
        while f.tell() < fh_end:
            kv_len = struct.unpack('<I', f.read(4))[0]
            kvd = f.read(kv_len)
            eq = kvd.find(b'=')
            if eq > 0: fh[kvd[:eq].decode('utf-8','replace')] = kvd[eq+1:]

        index_offset = struct.unpack('<q', fh['index_pos'])[0]
        conn_count = struct.unpack('<I', fh['conn_count'])[0]
        chunk_count = struct.unpack('<I', fh['chunk_count'])[0]
        print(f"Bag: {conn_count} conns, {chunk_count} chunks, size={os.path.getsize(BAG)/1e9:.2f}GB")

        # 读 connections
        connections = {}
        f.seek(index_offset)
        for _ in range(conn_count):
            h = read_header(f)
            cid = struct.unpack('<I', h['conn'])[0] if 'conn' in h else -1
            topic = h.get('topic', b'').decode('utf-8', 'replace')
            msg_type = h.get('type', b'').decode('utf-8', 'replace')
            connections[cid] = (topic, msg_type)
            dl = struct.unpack('<I', f.read(4))[0]
            f.seek(dl, 1)

        # 先从 index 里拿所有 chunk 的精确文件偏移，然后直接 seek
        chunk_offsets = []  # list of (file_offset, compressed_len, compression)
        f.seek(index_offset)
        # 跳过 connection records
        for _ in range(conn_count):
            h = read_header(f)
            dl = struct.unpack('<I', f.read(4))[0]
            f.seek(dl, 1)
        # 读取 chunk records
        for _ in range(chunk_count):
            h = read_header(f)
            chunk_pos = struct.unpack('<q', h['chunk_pos'])[0] if 'chunk_pos' in h else 0
            comp = h.get('compression', b'none').decode('utf-8', 'replace')
            dl = struct.unpack('<I', f.read(4))[0]
            # chunk record 的 index data 是 (conn_id, time_offset, offset) 列表，我们跳过
            f.seek(dl, 1)
            if chunk_pos > 0:
                chunk_offsets.append((chunk_pos, comp))

        print(f"  Got {len(chunk_offsets)} chunk offsets from index")
        file_size = index_offset

        start_time = None
        percent = 0

        for chunk_idx, (chunk_pos, comp) in enumerate(chunk_offsets):
            f.seek(chunk_pos)
            h = read_header(f)
            opb = h.get('op')
            if not opb or len(opb) < 1 or struct.unpack('<B', opb[:1])[0] != 0x05:
                continue

            dl_bytes = f.read(4)
            if len(dl_bytes) < 4: break
            dl = struct.unpack('<I', dl_bytes)[0]
            if dl == 0 or chunk_pos + 4 + dl > file_size:
                continue

            chunk_data = f.read(dl)
            try:
                buf = lz4.block.decompress(chunk_data) if comp == 'lz4' else chunk_data
            except:
                continue

            pos = 0
            while pos < len(buf):
                if pos + 5 > len(buf): break
                op = buf[pos]; pos += 1

                if op == 0x02:  # Data Record
                    if pos + 4 > len(buf): break
                    rh_len = struct.unpack('<I', buf[pos:pos+4])[0]
                    pos += 4
                    rh_end = pos + rh_len
                    rh = {}
                    while pos < rh_end:
                        if pos + 4 > len(buf): break
                        kl = struct.unpack('<I', buf[pos:pos+4])[0]
                        pos += 4
                        if kl == 0 or pos + kl > len(buf): break
                        kv = buf[pos:pos+kl]; pos += kl
                        eq = kv.find(b'=')
                        if eq > 0: rh[kv[:eq].decode('utf-8','replace')] = kv[eq+1:]
                    cid = struct.unpack('<I', rh['conn'])[0] if 'conn' in rh else -1
                    t_ns = struct.unpack('<Q', rh['time'])[0] if 'time' in rh else 0
                    if pos + 4 > len(buf): break
                    ml = struct.unpack('<I', buf[pos:pos+4])[0]
                    pos += 4
                    msg = buf[pos:pos+ml]
                    pos += ml

                    topic, msg_type = connections.get(cid, ('unknown', ''))
                    handler = TARGET_TOPICS.get(topic)
                    if handler:
                        t = t_ns * 1e-9
                        if start_time is None: start_time = t
                        kind = handler[0]
                        try:
                            if kind == 'odom': parsed = parse_odom_body(msg)
                            elif kind == 'pose': parsed = parse_pose_body(msg)
                            elif kind == 'state': parsed = parse_state_body(msg)
                            elif kind == 'extended_state': parsed = parse_extended_body(msg)
                            elif kind == 'string': parsed = parse_string_body(msg)
                            elif kind == 'rosout': parsed = parse_rosout_body(msg)
                            else: parsed = {'raw': msg[:40].hex()}
                        except:
                            parsed = {'raw': msg[:40].hex()}
                        results[topic].append((t, parsed))
                        if topic in ('/mavros/local_position/odom', '/navigation/local_odom'):
                            total_odom += 1

                elif op == 0x04:  # Index Record - fast skip
                    if pos + 4 > len(buf): break
                    ih_len = struct.unpack('<I', buf[pos:pos+4])[0]
                    pos += 4 + ih_len
                    if pos + 4 > len(buf): break
                    nd = struct.unpack('<I', buf[pos:pos+4])[0]
                    pos += 4 + nd * 12
                else:
                    break

            chunk_idx += 1
            pct = f.tell() / file_size * 100
            if pct - percent >= 10:
                percent = pct
                print(f"  {chunk_idx}/{chunk_count} chunks, {pct:.0f}%  (odom={total_odom})")

    print(f"\nDone: {chunk_idx} chunks processed, start={start_time}")
    for tp, ms in results.items():
        if ms:
            print(f"  {tp}: {len(ms)} msgs")
    return results, start_time

# ---------- 输出 ----------
def dump(results, start):
    if start is None:
        print("No data parsed"); return

    print(f"\n{'='*80}")
    print(f"BAG START: {start}")
    print(f"{'='*80}")

    # 1. 高度时间线 — 找最后 80 秒的 odom
    odom_topic = None
    for tp in ('/mavros/local_position/odom', '/navigation/local_odom',
               '/mavros/local_position/pose', '/navigation/local_pose'):
        if tp in results and results[tp]:
            odom_topic = tp; break

    if odom_topic:
        msgs = results[odom_topic]
        t_end = msgs[-1][0]
        t_start_win = t_end - 80  # 最后 80 秒
        window = [(t, m) for t, m in msgs if t >= t_start_win]
        # 均匀采样 <= 120 条
        if len(window) > 120:
            step = len(window) // 120
            window = window[::step]

        print(f"\n### {odom_topic}  最后 80 秒高度时间线")
        print(f"{'t_rel(s)':>10} {'x':>8} {'y':>8} {'z(本地Z)':>10} {'vx':>8} {'vy':>8} {'vz':>8}")
        for t, m in window:
            tr = t - start
            z = m.get('z', 0)
            vx = m.get('vx', 0); vy = m.get('vy', 0); vz = m.get('vz', 0)
            print(f"{tr:10.2f} {m.get('x',0):8.3f} {m.get('y',0):8.3f} {z:10.3f} {vx:8.3f} {vy:8.3f} {vz:8.3f}")

    # 2. setpoint (查看 OFFBOARD 期望高度)
    sp_topic = '/mavros/setpoint_position/local'
    if sp_topic in results and results[sp_topic]:
        msgs = results[sp_topic]
        t_end = msgs[-1][0]
        window = [(t, m) for t, m in msgs if t >= t_end - 60]
        if len(window) > 60:
            step = len(window) // 60; window = window[::step]
        print(f"\n### {sp_topic}  最后 60 秒期望高度")
        print(f"{'t_rel(s)':>10} {'x_cmd':>8} {'y_cmd':>8} {'z_cmd':>10}")
        for t, m in window:
            print(f"{t-start:10.2f} {m.get('x',0):8.3f} {m.get('y',0):8.3f} {m.get('z',0):10.3f}")

    # 3. MAVROS State 切换
    if '/mavros/state' in results:
        msgs = results['/mavros/state']
        last = None
        print(f"\n### /mavros/state  状态切换点")
        for t, m in msgs:
            sig = (m.get('armed'), m.get('mode'))
            if sig != last:
                print(f"  t={t-start:8.2f}s  armed={m.get('armed')}  mode={m.get('mode','')}")
                last = sig

    # 4. Extended State (landed_state)
    if '/mavros/extended_state' in results:
        states = {0:'UNK', 1:'ON_GROUND', 2:'IN_AIR', 3:'TAKEOFF', 4:'LANDING'}
        msgs = results['/mavros/extended_state']
        print(f"\n### /mavros/extended_state  着陆状态")
        for t, m in msgs:
            ls = m.get('landed_state', 0)
            print(f"  t={t-start:8.2f}s  landed_state={states.get(ls, ls)}")

    # 5. mission_status (JSON)
    if '/navigation/mission_status' in results:
        msgs = results['/navigation/mission_status']
        last_phase = None
        print(f"\n### /navigation/mission_status  阶段变化")
        for t, m in msgs:
            data = m.get('data', '')
            try:
                j = json.loads(data)
                phase = j.get('phase', '?')
                cmd = j.get('active_command', '?')
                if phase != last_phase:
                    print(f"  t={t-start:8.2f}s  phase={phase}  cmd={cmd}  commit={j.get('committed_slots','')}  fail={j.get('mission_failed')}")
                    last_phase = phase
            except: pass

    # 6. handoff
    if '/patrol_control/external_landing_handoff' in results:
        msgs = results['/patrol_control/external_landing_handoff']
        print(f"\n### /patrol_control/external_landing_handoff")
        for t, m in msgs:
            print(f"  t={t-start:8.2f}s  {m.get('data','')}")

    # 7. mission_command / result
    for tp in ['/navigation/mission_command_raw', '/navigation/mission_result']:
        if tp in results:
            msgs = results[tp]
            print(f"\n### {tp}")
            for t, m in msgs:
                print(f"  t={t-start:8.2f}s  {m.get('data','')[:200]}")

    # 8. rosout 关键词过滤
    if '/rosout_agg' in results:
        msgs = results['/rosout_agg']
        kw = ['landing', 'handoff', 'land', 'descend', 'auto_land', 'align', 'settle', 'hover', 'external', 'fail', 'stability', 'control_ready', 'CALLAND']
        level_map = {1:'DBG', 2:'INF', 4:'WRN', 8:'ERR', 16:'FTL'}
        print(f"\n### /rosout_agg  降落相关日志")
        for t, m in msgs:
            msg = m.get('msg', '').lower()
            if any(k in msg for k in kw):
                print(f"  t={t-start:8.2f}s [{level_map.get(m.get('level',0),'?')}] {m.get('msg','')[:250]}")

if __name__ == '__main__':
    r, s = main()
    dump(r, s)
