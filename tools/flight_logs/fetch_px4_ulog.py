#!/usr/bin/env python3
"""Read a closed PX4 ULog via MAVROS while connected and disarmed.

Lists FCU log directories with FTP, but downloads through LOG_REQUEST_DATA:
this aircraft's FTP Read returned empty data despite a successful Open.
Never changes flight/logging parameters, arms, writes or deletes FCU files.
The FCU may pause logging during a transfer; LOG_REQUEST_END always follows.
"""
import argparse
from datetime import datetime, timedelta, timezone
import json
import math
from pathlib import Path, PurePosixPath
import queue
import signal
import time


ULOG_MAGIC = b"ULog\x01\x12\x35"
CHUNK = 90  # MAVLink LOG_DATA payload
WINDOW = CHUNK * 256


def checked_directory(value):
    path = PurePosixPath(value)
    root = PurePosixPath("/fs/microsd/log")
    if ".." in path.parts or (path != root and root not in path.parents):
        raise ValueError("Only /fs/microsd/log and its children are accepted")
    return str(path)


def accept_chunk(log_id, start, stop, received, message):
    """Collect exact requested chunks; reject wrong IDs, offsets and truncation."""
    offset = message.offset
    if message.id != log_id or not start <= offset < stop or (offset-start) % CHUNK:
        return False
    payload = bytes(message.data)
    if len(payload) != min(CHUNK, stop-offset):
        return False
    if offset in received and received[offset] != payload:
        raise RuntimeError("Conflicting retransmission at offset {}".format(offset))
    received[offset] = payload
    return True


def entry_record(entry):
    seconds = entry.time_utc.to_sec()
    valid = seconds >= 946684800  # Earlier dates generally mean no FCU UTC clock.
    return dict(log_id=entry.id, bytes=entry.size,
                time_utc=seconds,
                time_beijing=(datetime.fromtimestamp(seconds, timezone(timedelta(hours=8)))
                              .strftime("%Y-%m-%d %H:%M:%S") if valid else None))


def show_entries(entries, output_format):
    records = [entry_record(entries[key]) for key in sorted(entries)]
    if output_format == "json":
        print(json.dumps(records, indent=2, ensure_ascii=False), flush=True)
        return
    print("日志编号  飞控记录时间（北京时间）     大小 MiB / 字节", flush=True)
    for row in records:
        print("{:<8}  {:<26} {:>8.2f} / {}".format(
            row['log_id'], row['time_beijing'] or "时间未知（飞控未同步UTC）",
            row['bytes'] / 1048576., row['bytes']), flush=True)
    if records:
        print("共 {} 份日志；最新索引编号：{}。使用 --log-id 编号 --output 文件路径 下载。".format(
            len(records), records[-1]['log_id']), flush=True)
    else:
        print("飞控日志索引为空。", flush=True)


def collect_index(listing, entries, check, timeout, selected_id=None):
    """Retry an incomplete index; never mistake a partial listing for success."""
    deadline = time.monotonic() + timeout
    retry_at = 0.
    while time.monotonic() < deadline:
        check()
        now = time.monotonic()
        if now >= retry_at:
            start, stop = (selected_id, selected_id) if selected_id is not None else (0, 65535)
            if not listing(start, stop).success:
                raise RuntimeError("Log index request failed")
            retry_at = now + 2.
        snapshot = list(entries.values())
        if selected_id is not None and selected_id in entries:
            return
        if selected_id is None and snapshot:
            counts = {e.num_logs for e in snapshot}
            lasts = {e.last_log_num for e in snapshot}
            if counts == {0}:
                entries.clear()
                return
            if len(counts) == 1 and len(lasts) == 1:
                count = next(iter(counts))
                if len(snapshot) == count and max(e.id for e in snapshot) == next(iter(lasts)):
                    return
        time.sleep(.05)
    raise TimeoutError("日志索引未完整返回（收到 {} 条）；请检查 MAVROS log_transfer 插件/连接后重试。".format(len(entries)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", default="/mavros")
    parser.add_argument("--list-dir", default="/fs/microsd/log")
    parser.add_argument("--list", action="store_true", help="List downloadable log IDs via MAVLink")
    parser.add_argument("--format", choices=("table", "json"), default="json")
    parser.add_argument("--list-timeout", type=float, default=30)
    parser.add_argument("--log-id", type=int, help="Explicit ID from the FCU log index")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--timeout", type=float, default=600)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be finite and positive")
    if args.log_id is not None and not 0 <= args.log_id < 65535:
        parser.error("--log-id must be 0..65534")
    if args.log_id is not None and args.output is None:
        parser.error("--output is required for a download")
    if args.list and (args.log_id is not None or args.output is not None):
        parser.error("--list cannot be combined with --log-id or --output")
    if not math.isfinite(args.list_timeout) or args.list_timeout <= 0:
        parser.error("--list-timeout must be finite and positive")
    directory = checked_directory(args.list_dir)
    partial = args.output.with_name(args.output.name + ".part") if args.output else None
    if args.log_id is not None and (args.output.exists() or partial.exists()):
        parser.error("Output or partial exists; choose another path (nothing is overwritten)")

    import rospy
    from mavros_msgs.msg import State, LogData, LogEntry
    from mavros_msgs.srv import FileList, LogRequestList, LogRequestData, LogRequestEnd

    rospy.init_node("fetch_px4_ulog", anonymous=True, disable_signals=True)
    state = [None, 0.]
    def on_state(msg):
        state[:] = [msg, time.monotonic()]
    subscriber = rospy.Subscriber(args.namespace + "/state", State, on_state, queue_size=1)
    def require_ground():
        msg, received_at = state
        if msg is None or time.monotonic()-received_at > 3. or not msg.connected or msg.armed:
            raise RuntimeError("Fresh connected and disarmed FCU state required")
    until = time.monotonic()+6
    while state[0] is None and time.monotonic() < until:
        time.sleep(.05)
    require_ground()

    def service(name, cls):
        rospy.wait_for_service(name, timeout=5)
        return rospy.ServiceProxy(name, cls)
    if args.log_id is None and not args.list:
        result = service(args.namespace + "/ftp/list", FileList)(directory)
        if not result.success:
            raise RuntimeError("FTP list failed errno=" + str(result.r_errno))
        print(json.dumps([dict(name=f.name, type=f.type, size=f.size) for f in result.list], indent=2))
        return

    prefix = args.namespace + "/log_transfer/raw/"
    end = service(prefix + "log_request_end", LogRequestEnd)
    listing = service(prefix + "log_request_list", LogRequestList)
    request = service(prefix + "log_request_data", LogRequestData) if not args.list else None
    entries = {}
    messages = queue.Queue(maxsize=4096)
    def on_entry(msg):
        if args.list or msg.id == args.log_id:
            entries[msg.id] = msg
    def on_data(msg):
        if msg.id == args.log_id:
            try:
                messages.put_nowait(msg)
            except queue.Full:
                pass  # Missing chunks are requested again; memory remains bounded.
    entry_sub = rospy.Subscriber(prefix + "log_entry", LogEntry, on_entry, queue_size=4096)
    data_sub = rospy.Subscriber(prefix + "log_data", LogData, on_data, queue_size=4096) if not args.list else None
    def interrupted(signum, frame):
        raise KeyboardInterrupt("Interrupted; partial retained")
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    started = time.monotonic()
    last_notice = started
    def check():
        require_ground()
        if rospy.is_shutdown() or time.monotonic()-started > args.timeout:
            raise TimeoutError("Transfer stopped; partial retained")

    try:
        time.sleep(.3)
        check()
        if args.list:
            collect_index(listing, entries, check, args.list_timeout)
            show_entries(entries, args.format)
            return
        if not listing(args.log_id, args.log_id).success:
            raise RuntimeError("Log index request failed")
        entry_limit = time.monotonic()+8
        while args.log_id not in entries and time.monotonic() < entry_limit:
            check()
            time.sleep(.05)
        entry = entries.get(args.log_id)
        if entry is None or entry.size < 16:
            raise RuntimeError("No valid log index response for requested ID")
        print("ULOG_SELECTED id={} bytes={} fcu_time={}".format(
            entry.id, entry.size, entry.time_utc.to_sec()), flush=True)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with partial.open("xb") as stream:
            for start in range(0, entry.size, WINDOW):
                stop = min(start+WINDOW, entry.size)
                received = {}
                offsets = list(range(start, stop, CHUNK))
                while len(received) != len(offsets):
                    check()
                    missing = [p for p in offsets if p not in received]
                    first = missing[0]
                    last = first+CHUNK
                    while last < stop and last not in received:
                        last += CHUNK
                    if not request(entry.id, first, min(last, stop)-first).success:
                        raise RuntimeError("Log data request failed")
                    # Wait for packets or a short quiet period, then retry holes.
                    idle_until = time.monotonic()+2
                    while time.monotonic() < idle_until and len(received) < len(offsets):
                        check()
                        try:
                            msg = messages.get(timeout=.15)
                        except queue.Empty:
                            continue
                        if accept_chunk(entry.id, start, stop, received, msg):
                            idle_until = time.monotonic()+.5
                block = b"".join(received[p] for p in offsets)
                if start == 0 and not block.startswith(ULOG_MAGIC):
                    raise RuntimeError("Data is not a ULog; partial retained")
                stream.write(block)
                if time.monotonic()-last_notice >= 5:
                    print("ULOG_PROGRESS {}/{} bytes".format(stop, entry.size), flush=True)
                    last_notice = time.monotonic()
        if partial.stat().st_size != entry.size:
            raise RuntimeError("ULog size mismatch; partial retained")
        partial.rename(args.output)
        record = dict(log_id=entry.id, bytes=entry.size, fcu_time=entry.time_utc.to_sec(),
                      output=str(args.output), elapsed_sec=round(time.monotonic()-started, 3),
                      flight_parameters_changed=False, fcu_files_written=False,
                      flight_identity="unverified: match arming and flight motion with bag")
        args.output.with_name(args.output.name+".download.json").write_text(
            json.dumps(record, indent=2), encoding="utf-8")
        print(json.dumps(record), flush=True)
    finally:
        try:
            if not end().success:
                print("WARNING: LOG_REQUEST_END failed; check the FCU transfer state", flush=True)
        except Exception as exc:
            print("WARNING: cannot finish log transfer: " + str(exc), flush=True)


if __name__ == "__main__":
    main()