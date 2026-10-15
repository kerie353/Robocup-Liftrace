#!/usr/bin/env python3
"""最小化 bag 调试 — 看 chunk 内部到底是什么 opcode。"""
import struct, os
import lz4.block

BAG = "c:\\Users\\lihaoEN\\Downloads\\1014-0601-投递H2D2-H降落vhD2.bag"

with open(BAG, 'rb') as f:
    magic = f.read(13)
    print("Magic:", magic)
    fh_len = struct.unpack('<I', f.read(4))[0]
    fh_end = f.tell() + fh_len
    fh = {}
    while f.tell() < fh_end:
        kv_len = struct.unpack('<I', f.read(4))[0]
        kvd = f.read(kv_len)
        eq = kvd.find(b'=')
        if eq > 0: fh[kvd[:eq].decode('utf-8', 'replace')] = kvd[eq+1:]
    
    index_offset = struct.unpack('<q', fh['index_pos'])[0]
    conn_count = struct.unpack('<I', fh['conn_count'])[0]
    chunk_count = struct.unpack('<I', fh['chunk_count'])[0]
    print(f"index={index_offset}, conns={conn_count}, chunks={chunk_count}")

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

    connections = {}
    f.seek(index_offset)
    for i in range(conn_count):
        h = read_header(f)
        cid = struct.unpack('<I', h['conn'])[0] if 'conn' in h else -1
        topic = h.get('topic', b'').decode('utf-8', 'replace')
        connections[cid] = topic
        dl = struct.unpack('<I', f.read(4))[0]
        f.seek(dl, 1)

    print(f"\nConnection IDs -> topics:")
    for cid, topic in sorted(connections.items()):
        print(f"  {cid}: {topic}")

    # 读 chunk record 拿 offsets
    chunk_offsets = []
    f.seek(index_offset)
    for _ in range(conn_count):
        h = read_header(f)
        dl = struct.unpack('<I', f.read(4))[0]
        f.seek(dl, 1)
    for _ in range(chunk_count):
        h = read_header(f)
        chunk_pos = struct.unpack('<q', h['chunk_pos'])[0] if 'chunk_pos' in h else 0
        comp = h.get('compression', b'none').decode('utf-8', 'replace')
        dl = struct.unpack('<I', f.read(4))[0]
        f.seek(dl, 1)
        if chunk_pos > 0:
            chunk_offsets.append((chunk_pos, comp))

    # 检查前 5 个 chunk 的内部 opcode 分布
    print(f"\n=== 前 5 个 chunk 内部结构 ===")
    for ci, (chunk_pos, comp) in enumerate(chunk_offsets[:5]):
        f.seek(chunk_pos)
        h = read_header(f)
        dl = struct.unpack('<I', f.read(4))[0]
        chunk_data = f.read(dl)
        try:
            buf = lz4.block.decompress(chunk_data) if comp == 'lz4' else chunk_data
        except Exception as e:
            print(f"  chunk {ci}: decompress fail {e}"); continue

        op_counts = {}
        cids_seen = set()
        pos = 0
        records = 0
        while pos < len(buf) and records < 200:
            if pos >= len(buf): break
            op = buf[pos]; pos += 1
            op_counts[op] = op_counts.get(op, 0) + 1
            
            if op == 0x02:  # data
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
                cids_seen.add(cid)
                if pos + 4 > len(buf): break
                ml = struct.unpack('<I', buf[pos:pos+4])[0]
                pos += 4 + ml
                records += 1
            elif op == 0x04:  # index record
                if pos + 4 > len(buf): break
                ih_len = struct.unpack('<I', buf[pos:pos+4])[0]
                pos += 4 + ih_len
                if pos + 4 > len(buf): break
                nd = struct.unpack('<I', buf[pos:pos+4])[0]
                pos += 4 + nd * 12
                records += 1
            else:
                break

        print(f"  chunk {ci}: pos={chunk_pos}, opcodes={dict(op_counts)}, cids={sorted(cids_seen)[:10]}... total_records={records}")
        if cids_seen:
            for cid in sorted(cids_seen)[:10]:
                t = connections.get(cid, 'unknown')
                print(f"    cid {cid}: {t}")
