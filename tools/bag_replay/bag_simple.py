#!/usr/bin/env python3
"""彻底重写 bag 解析 — 先完全搞懂格式再写逻辑。"""
import struct, os, sys, json
from collections import defaultdict
import lz4.block

BAG = sys.argv[1] if len(sys.argv) > 1 else None
if not BAG:
    print("Usage: python bag_simple.py <bag>"); sys.exit(1)

def read_kv_header(data, pos):
    """从 buffer 某个偏移读 ROS bag 风格的 header（键值对序列）"""
    if pos + 4 > len(data): return None, pos
    hl = struct.unpack('<I', data[pos:pos+4])[0]
    pos += 4
    if hl == 0 or pos + hl > len(data): return None, pos + 4
    end = pos + hl
    fields = {}
    while pos < end:
        if pos + 4 > end: break
        kvl = struct.unpack('<I', data[pos:pos+4])[0]
        pos += 4
        if kvl == 0 or pos + kvl > end: break
        kv = data[pos:pos+kvl]; pos += kvl
        eq = kv.find(b'=')
        if eq > 0: fields[kv[:eq].decode('utf-8','replace')] = kv[eq+1:]
    return fields, end  # 返回 header 和 header 结束位置

def parse_ros_string(data, pos):
    l = struct.unpack('<I', data[pos:pos+4])[0]; pos += 4
    s = data[pos:pos+l].decode('utf-8','replace'); pos += l
    return s, pos

# ---------- 消息解析 ----------
def parse_odom(data):
    pos = 0
    pos += 4  # seq
    pos += 8  # stamp
    fl = struct.unpack('<I', data[pos:pos+4])[0]; pos += 4 + fl
    cl = struct.unpack('<I', data[pos:pos+4])[0]; pos += 4 + cl
    x, y, z = struct.unpack('<ddd', data[pos:pos+24]); pos += 24
    pos += 32  # quat
    pos += 72  # pose cov
    vx, vy, vz = struct.unpack('<ddd', data[pos:pos+24]); pos += 24
    pos += 24  # angular
    return {'x': x, 'y': y, 'z': z, 'vx': vx, 'vy': vy, 'vz': vz}

def parse_pose(data):
    pos = 4 + 8  # skip seq + stamp
    fl = struct.unpack('<I', data[pos:pos+4])[0]; pos += 4 + fl
    x, y, z = struct.unpack('<ddd', data[pos:pos+24])
    return {'x': x, 'y': y, 'z': z}

def parse_state(data):
    pos = 0
    connected = data[pos] > 0; pos += 1
    armed = data[pos] > 0; pos += 1
    guided = data[pos] > 0; pos += 1
    manual_input = data[pos] > 0; pos += 1
    ml = struct.unpack('<I', data[pos:pos+4])[0]; pos += 4
    mode = data[pos:pos+ml].decode('utf-8','replace')
    return {'armed': armed, 'mode': mode, 'connected': connected}

def parse_extended(data):
    return {'landed_state': data[0]}

def parse_string_msg(data):
    l = struct.unpack('<I', data[:4])[0]
    return {'data': data[4:4+l].decode('utf-8','replace')}

def parse_rosout_msg(data):
    pos = 4 + 8  # seq + stamp
    fl = struct.unpack('<I', data[pos:pos+4])[0]; pos += 4 + fl
    level = data[pos]; pos += 1
    msg, pos = parse_ros_string(data, pos)
    return {'level': level, 'msg': msg}

PARSERS = {
    ('/mavros/local_position/odom', 'nav_msgs/Odometry'): parse_odom,
    ('/navigation/local_odom', 'nav_msgs/Odometry'): parse_odom,
    ('/mavros/local_position/pose', 'geometry_msgs/PoseStamped'): parse_pose,
    ('/navigation/local_pose', 'geometry_msgs/PoseStamped'): parse_pose,
    ('/mavros/setpoint_position/local', 'geometry_msgs/PoseStamped'): parse_pose,
    ('/mavros/state', 'mavros_msgs/State'): parse_state,
    ('/mavros/extended_state', 'mavros_msgs/ExtendedState'): parse_extended,
    ('/navigation/mission_status', 'std_msgs/String'): parse_string_msg,
    ('/navigation/mission_command_raw', 'std_msgs/String'): parse_string_msg,
    ('/navigation/mission_result', 'std_msgs/String'): parse_string_msg,
    ('/patrol_control/external_landing_handoff', 'std_msgs/String'): parse_string_msg,
    ('/rosout_agg', 'rosgraph_msgs/Log'): parse_rosout_msg,
}

def main():
    with open(BAG, 'rb') as f:
        magic = f.read(13); assert magic == b'#ROSBAG V2.0\n'
        
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
        print(f"Bag: {conn_count} conns, {chunk_count} chunks, index={index_offset}")

        # -------- 读 index 区：先拿 connections --------
        connections = {}
        f.seek(index_offset)
        for _ in range(conn_count):
            data = f.read(4096)  # 先读到 buffer 里
            hdr, end = read_kv_header(data, 0)
            cid = struct.unpack('<I', hdr['conn'])[0] if 'conn' in hdr else -1
            topic = hdr.get('topic', b'').decode('utf-8','replace')
            msg_type = hdr.get('type', b'').decode('utf-8','replace')
            connections[cid] = (topic, msg_type)
            dl = struct.unpack('<I', data[end:end+4])[0]
            # 跳过：当前 header 的 4+hdr_len + 4+data_len 字节
            f.seek(4 + struct.unpack('<I', f.read(0) if False else b'') [0] if False else 0)
            # 更简单：从当前位置（刚读完 index record header + 4）再跳过 dl
            # 但我们已经从文件里读了 4+hdr_len+4 = end+4 字节
            # 需要 seek 到 end+4+dl
            actual_hdr_len = end - 0  # 从 data buffer 起点算，但我们从文件读到哪里了？
            # 好，让我简化 — 用 f.tell() + dl
            f.seek(dl, 1)  # 跳过 data portion

        # 不对，让我重写这个逻辑，从文件里一个字节一个字节读

    # -------- 重新来：从文件头一个一个读 --------
    with open(BAG, 'rb') as f:
        magic = f.read(13)
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

        # Connections — 从 index_offset 开始读 conn_count 个 Connection Record
        connections = {}
        f.seek(index_offset)
        for _ in range(conn_count):
            hl_bytes = f.read(4)
            hl = struct.unpack('<I', hl_bytes)[0]
            hdr_bytes = f.read(hl)
            # 手动解析
            hdr = {}
            p = 0
            while p < len(hdr_bytes):
                if p + 4 > len(hdr_bytes): break
                kl = struct.unpack('<I', hdr_bytes[p:p+4])[0]; p += 4
                if kl == 0 or p + kl > len(hdr_bytes): break
                kv = hdr_bytes[p:p+kl]; p += kl
                eq = kv.find(b'=')
                if eq > 0: hdr[kv[:eq].decode('utf-8','replace')] = kv[eq+1:]
            cid = struct.unpack('<I', hdr['conn'])[0] if 'conn' in hdr else -1
            topic = hdr.get('topic', b'').decode('utf-8','replace')
            msg_type = hdr.get('type', b'').decode('utf-8','replace')
            connections[cid] = (topic, msg_type)
            dl = struct.unpack('<I', f.read(4))[0]  # data_len
            f.seek(dl, 1)  # skip connection data

        # Chunk records — 拿每个 chunk 的文件偏移
        chunk_positions = []
        for _ in range(chunk_count):
            hl = struct.unpack('<I', f.read(4))[0]
            hdr_bytes = f.read(hl)
            hdr = {}
            p = 0
            while p < len(hdr_bytes):
                if p + 4 > len(hdr_bytes): break
                kl = struct.unpack('<I', hdr_bytes[p:p+4])[0]; p += 4
                if kl == 0 or p + kl > len(hdr_bytes): break
                kv = hdr_bytes[p:p+kl]; p += kl
                eq = kv.find(b'=')
                if eq > 0: hdr[kv[:eq].decode('utf-8','replace')] = kv[eq+1:]
            cpos = struct.unpack('<q', hdr['chunk_pos'])[0] if 'chunk_pos' in hdr else 0
            dl = struct.unpack('<I', f.read(4))[0]
            f.seek(dl, 1)
            if cpos > 0: chunk_positions.append(cpos)

        print(f"  Got {len(chunk_positions)} chunk positions")

        # -------- 遍历 chunks --------
        results = defaultdict(list)
        start_time = None
        odom_count = 0
        
        for ci, cpos in enumerate(chunk_positions):
            f.seek(cpos)
            # 读 chunk header
            hl = struct.unpack('<I', f.read(4))[0]
            hdr_bytes = f.read(hl)
            hdr = {}
            p = 0
            while p < len(hdr_bytes):
                if p + 4 > len(hdr_bytes): break
                kl = struct.unpack('<I', hdr_bytes[p:p+4])[0]; p += 4
                if kl == 0 or p + kl > len(hdr_bytes): break
                kv = hdr_bytes[p:p+kl]; p += kl
                eq = kv.find(b'=')
                if eq > 0: hdr[kv[:eq].decode('utf-8','replace')] = kv[eq+1:]
            
            op = hdr.get('op', b'').decode()
            comp = hdr.get('compression', b'none').decode()
            dl = struct.unpack('<I', f.read(4))[0]
            chunk_data = f.read(dl)
            
            if op != '5':  # 0x05 = CHUNK
                continue
            
            # 解压
            try:
                buf = lz4.block.decompress(chunk_data) if comp == 'lz4' else chunk_data
            except:
                continue
            
            # 遍历 chunk 内 record
            pos = 0
            while pos < len(buf):
                if pos >= len(buf): break
                opcode = buf[pos]; pos += 1
                
                if opcode == 0x02:  # Data Record
                    if pos + 4 > len(buf): break
                    rhl = struct.unpack('<I', buf[pos:pos+4])[0]; pos += 4
                    rhe = pos + rhl
                    rh = {}
                    while pos < rhe:
                        if pos + 4 > len(buf): break
                        kl = struct.unpack('<I', buf[pos:pos+4])[0]; pos += 4
                        if kl == 0 or pos + kl > rhe: break
                        kv = buf[pos:pos+kl]; pos += kl
                        eq = kv.find(b'=')
                        if eq > 0: rh[kv[:eq].decode('utf-8','replace')] = kv[eq+1:]
                    cid = struct.unpack('<I', rh['conn'])[0] if 'conn' in rh else -1
                    t_ns = struct.unpack('<Q', rh['time'])[0] if 'time' in rh else 0
                    if pos + 4 > len(buf): break
                    ml = struct.unpack('<I', buf[pos:pos+4])[0]; pos += 4
                    msg_data = buf[pos:pos+ml]; pos += ml
                    
                    topic, msg_type = connections.get(cid, ('unknown',''))
                    handler = PARSERS.get((topic, msg_type))
                    if handler:
                        t = t_ns * 1e-9
                        if start_time is None: start_time = t
                        try:
                            parsed = handler(msg_data)
                        except:
                            parsed = {'raw': msg_data[:40].hex()}
                        results[topic].append((t, parsed))
                        if 'odom' in topic: odom_count += 1
                
                elif opcode == 0x04:  # Index Record — 跳过
                    if pos + 4 > len(buf): break
                    ihl = struct.unpack('<I', buf[pos:pos+4])[0]; pos += 4 + ihl
                    if pos + 4 > len(buf): break
                    nd = struct.unpack('<I', buf[pos:pos+4])[0]; pos += 4 + nd * 12
                else:
                    break
            
            if (ci + 1) % 100 == 0:
                print(f"  chunk {ci+1}/{len(chunk_positions)}, odom={odom_count}")

        print(f"\nDone. start_time={start_time}, odom_count={odom_count}")
        for tp, ms in sorted(results.items()):
            print(f"  {tp}: {len(ms)} msgs")
        
        return results, start_time

def dump(results, start):
    if start is None:
        print("No data"); return
    print(f"\n{'='*80}\nBAG START = {start}\n{'='*80}")
    
    # 1. odom 高度时间线
    odom_tp = None
    for tp in ['/mavros/local_position/odom', '/navigation/local_odom',
               '/mavros/local_position/pose', '/navigation/local_pose']:
        if tp in results and results[tp]:
            odom_tp = tp; break
    
    if odom_tp:
        ms = results[odom_tp]
        t_end = ms[-1][0]
        win = [(t,m) for t,m in ms if t >= t_end - 80]
        if len(win) > 120:
            step = len(win) // 120; win = win[::step]
        print(f"\n### {odom_tp}  最后 80 秒高度")
        print(f"{'t(s)':>9} {'x':>8} {'y':>8} {'z_fc':>8} {'vx':>7} {'vy':>7} {'vz':>7}")
        for t,m in win:
            tr = t - start
            print(f"{tr:9.2f} {m.get('x',0):8.3f} {m.get('y',0):8.3f} {m.get('z',0):8.3f} {m.get('vx',0):7.3f} {m.get('vy',0):7.3f} {m.get('vz',0):7.3f}")
    
    # 2. setpoint
    if '/mavros/setpoint_position/local' in results:
        ms = results['/mavros/setpoint_position/local']
        t_end = ms[-1][0]
        win = [(t,m) for t,m in ms if t >= t_end - 60]
        if len(win) > 60: step = len(win)//60; win = win[::step]
        print(f"\n### setpoint z")
        for t,m in win:
            print(f"  t={t-start:8.2f}s  x={m.get('x',0):.3f} y={m.get('y',0):.3f} z_cmd={m.get('z',0):.3f}")

    # 3. state 切换
    if '/mavros/state' in results:
        ms = results['/mavros/state']
        last = None
        print(f"\n### mavros state")
        for t,m in ms:
            sig = (m.get('armed'), m.get('mode'))
            if sig != last:
                print(f"  t={t-start:8.2f}s  armed={m.get('armed')}  mode={m.get('mode','')}")
                last = sig

    # 4. extended state
    if '/mavros/extended_state' in results:
        st = {0:'UNK',1:'ON_GROUND',2:'IN_AIR',3:'TAKEOFF',4:'LANDING'}
        for t,m in results['/mavros/extended_state']:
            print(f"  t={t-start:8.2f}s  landed={st.get(m.get('landed_state',0),'?')}")

    # 5. mission_status 阶段变化
    if '/navigation/mission_status' in results:
        ms = results['/navigation/mission_status']
        last_p = None
        print(f"\n### mission_status phase changes")
        for t,m in ms:
            try:
                j = json.loads(m.get('data',''))
                p = j.get('phase','?'); cmd = j.get('active_command','?')
                if p != last_p:
                    print(f"  t={t-start:8.2f}s  phase={p} cmd={cmd} commit={j.get('committed_slots','')} fail={j.get('mission_failed')}")
                    last_p = p
            except: pass

    # 6. handoff
    for tp in ['/patrol_control/external_landing_handoff','/navigation/mission_command_raw','/navigation/mission_result']:
        if tp in results:
            print(f"\n### {tp}")
            for t,m in results[tp]:
                print(f"  t={t-start:8.2f}s  {m.get('data','')[:200]}")

    # 7. rosout 关键词
    if '/rosout_agg' in results:
        kw = ['land','handoff','descend','auto_land','align','settle','hover','external','fail','stability','control']
        lv = {1:'DBG',2:'INF',4:'WRN',8:'ERR',16:'FTL'}
        print(f"\n### rosout (landing keywords)")
        for t,m in results['/rosout_agg']:
            msg = m.get('msg','').lower()
            if any(k in msg for k in kw):
                print(f"  t={t-start:8.2f}s [{lv.get(m.get('level',0),'?')}] {m.get('msg','')[:250]}")

if __name__ == '__main__':
    r, s = main()
    dump(r, s)
