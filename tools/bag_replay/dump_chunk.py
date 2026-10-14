#!/usr/bin/env python3
"""dump 一个 chunk 解压后的原始字节看看到底是什么。"""
import struct, os
import lz4.block

BAG = "c:\\Users\\lihaoEN\\Downloads\\1014-0601-投递H2D2-H降落vhD2.bag"

with open(BAG, 'rb') as f:
    magic = f.read(13)
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

    # 读 chunk 0
    f.seek(13 + 4 + fh_len)
    h = read_header(f)
    print(f"Chunk 0 header: op={h.get(b'op', b'?').hex()}, compression={h.get(b'compression', b'none')}")
    dl = struct.unpack('<I', f.read(4))[0]
    chunk_data = f.read(dl)
    comp = h.get(b'compression', b'none').decode('utf-8', 'replace')
    print(f"  compressed_len={dl}, compression={comp}")

    if comp == 'lz4':
        buf = lz4.block.decompress(chunk_data)
    else:
        buf = chunk_data
    
    print(f"  decompressed_len={len(buf)}")
    print(f"  first 200 bytes hex: {buf[:200].hex()}")
    print()

    # 尝试搜索所有 opcode = 0x02 出现的位置
    print("  Scanning for opcode 0x02 (Data Record) positions...")
    found = []
    for i in range(len(buf) - 1):
        if buf[i] == 0x02:
            # 尝试解析 header_len
            if i + 5 <= len(buf):
                hlen = struct.unpack('<I', buf[i+1:i+5])[0]
                # 合理的 header_len 应该在 10-100 之间
                if 10 <= hlen <= 200 and i + 5 + hlen + 4 <= len(buf):
                    found.append((i, hlen))
                    if len(found) <= 5:
                        print(f"    pos={i}: header_len={hlen}, next_bytes={buf[i:i+20].hex()}")
    
    print(f"  Total potential 0x02 positions: {len(found)}")
    
    # 也搜 0x04
    print(f"\n  Scanning for opcode 0x04 (Index Record) positions...")
    idx_found = []
    for i in range(len(buf) - 1):
        if buf[i] == 0x04:
            if i + 5 <= len(buf):
                hlen = struct.unpack('<I', buf[i+1:i+5])[0]
                if 10 <= hlen <= 200 and i + 5 + hlen + 4 <= len(buf):
                    idx_found.append((i, hlen))
    
    print(f"  Total potential 0x04 positions: {len(idx_found)}")
    for pos, hlen in idx_found[:5]:
        print(f"    pos={pos}: header_len={hlen}")
