#!/usr/bin/env python3
"""
实机舵机调试后端节点 (Orange Pi 5 Plus)。

功能：
  - 提供 patrol_control/SetServoDuty 服务，支持对 rear/right/left 三个舵机通道
    设置任意 PWM duty cycle (0.5ms~2.5ms)。
  - 安全时序：disable -> set_duty -> enable -> 等待 -> disable。
  - 在非 Orange Pi 5 Plus 硬件上启动时会 fatal 退出。
  - 同时发布 /servo_debug/status ([Float32MultiArray]) 话题，
    数据为 [rear_duty_ns, right_duty_ns, left_duty_ns, rear_enabled, right_enabled, left_enabled]。
    供 GUI 轮询实时状态。

运行前提：
  1. Orange Pi 5 Plus (40 pin header)
  2. 已运行 sudo bash init_pwm.sh (设置 PWM 权限和 period)
  3. 节点对 /sys/class/pwm/pwmchipN/pwm0/{duty_cycle,enable} 有可写权限

通道映射 (BoardWiring.h):
  slot=1 (rear) : febf0000.pwm   GPIO3_B5 PWM12_M0   Pin16
  slot=2 (right): febf0020.pwm   GPIO1_D6 PWM14_M2   Pin7
  slot=3 (left) : febf0010.pwm   GPIO3_B6 PWM13_M0   Pin18
"""

import os
import re
import time
import rospy
from std_msgs.msg import Float32MultiArray
from patrol_control.srv import SetServoDuty, SetServoDutyResponse


PWM_PERIOD_NS = 20_000_000   # 20ms
DUTY_MIN_NS = 500_000         # 0.5ms
DUTY_MAX_NS = 2_500_000       # 2.5ms

# BoardWiring.h 中的设备名，按 slot 1/2/3 顺序
EXPECTED_DEVICES = ["febf0000.pwm", "febf0020.pwm", "febf0010.pwm"]

# SlotCalibration.h 中的参考值
INITIAL_DUTY_NS = [1_700_000, 1_000_000, 1_100_000]  # locked
RELEASE_DUTY_NS = [2_100_000, 2_100_000, 2_100_000]  # release


def _read_text(path):
    try:
        with open(path, "r") as f:
            return f.read().strip()
    except (OSError, IOError):
        return None


def _write_text(path, value):
    try:
        with open(path, "w") as f:
            f.write(str(value))
            f.flush()
        # readback 验证
        actual = _read_text(path)
        return actual == str(value)
    except (OSError, IOError) as exc:
        rospy.logerr("PWM write failed %s=%s: %s", path, value, exc)
        return False


def _find_pwm_chip(device_name):
    """通过设备名 (如 febf0000.pwm) 查找对应的 pwmchipN 路径。"""
    for chip in range(64):
        base = f"/sys/class/pwm/pwmchip{chip}"
        if not os.path.isdir(base):
            continue
        try:
            resolved = os.path.realpath(base)
        except OSError:
            continue
        if f"/{device_name}/" in resolved:
            return f"{base}/pwm0"
    return None


def _is_orange_pi_5_plus():
    model = _read_text("/proc/device-tree/model")
    if model is None:
        return False
    return "OPi 5 Plus" in model or "Orange Pi 5 Plus" in model


class PWMChannel:
    """单个 PWM 通道的封装。"""

    def __init__(self, pwm_path, slot_name):
        self.pwm_path = pwm_path
        self.slot_name = slot_name  # "rear"/"right"/"left"
        self._enabled = False
        self._current_duty = 0

    @property
    def enable_path(self):
        return f"{self.pwm_path}/enable"

    @property
    def duty_path(self):
        return f"{self.pwm_path}/duty_cycle"

    def read_enable(self):
        val = _read_text(self.enable_path)
        return val == "1"

    def read_duty(self):
        val = _read_text(self.duty_path)
        try:
            return int(val) if val is not None else 0
        except ValueError:
            return 0

    def set_enable(self, enabled):
        ok = _write_text(self.enable_path, "1" if enabled else "0")
        if ok:
            self._enabled = enabled
        return ok

    def set_duty(self, duty_ns):
        if not DUTY_MIN_NS <= duty_ns <= DUTY_MAX_NS:
            rospy.logwarn("duty %d 超出安全范围 [%d, %d]", duty_ns, DUTY_MIN_NS, DUTY_MAX_NS)
        ok = _write_text(self.duty_path, str(duty_ns))
        if ok:
            self._current_duty = duty_ns
        return ok

    def safe_pulse(self, duty_ns, hold=False, wait_sec=0.3):
        """
        安全 PWM 脉冲: disable -> set_duty -> enable -> 等待 -> [disable 或保持 enable]。
        返回 (success, actual_duty, message)。
        """
        # 1. disable
        if not self.set_enable(False):
            return False, 0, "failed to disable before set_duty"
        time.sleep(0.02)

        # 2. set duty
        if not self.set_duty(duty_ns):
            return False, 0, f"failed to set duty to {duty_ns}"

        # 3. enable
        if not self.set_enable(True):
            self.set_enable(False)
            return False, 0, "failed to enable after set_duty"

        # 4. 短暂等待让舵机动作
        if wait_sec > 0:
            time.sleep(wait_sec)

        # 5. disable (除非要求保持)
        if not hold:
            if not self.set_enable(False):
                rospy.logwarn("failed to disable after pulse (slot=%s)", self.slot_name)

        actual = self.read_duty()
        return True, actual, "ok"


class ServoDebugNode:
    """舵机调试 ROS 节点。"""

    def __init__(self):
        rospy.init_node("servo_debug_node", anonymous=False)

        # 硬件检测
        if not _is_orange_pi_5_plus():
            rospy.logfatal("servo_debug_node 只能运行在 Orange Pi 5 Plus 上。")
            raise SystemExit(1)

        rospy.loginfo("Orange Pi 5 Plus detected, searching PWM chips...")

        self.channels = []  # 按 slot_index 排序 (0=rear, 1=right, 2=left)
        slot_names = ["rear", "right", "left"]

        for i, dev in enumerate(EXPECTED_DEVICES):
            pwm_path = _find_pwm_chip(dev)
            if pwm_path is None:
                rospy.logfatal("PWM 设备 %s 未找到；请检查 device-tree overlay 和 init_pwm.sh", dev)
                raise SystemExit(1)

            # 检查必要文件可写
            for fname in ("period", "duty_cycle", "polarity", "enable"):
                fpath = f"{pwm_path}/{fname}"
                if not os.access(fpath, os.W_OK):
                    rospy.logfatal("PWM 文件不可写: %s。请先运行 sudo bash init_pwm.sh", fpath)
                    raise SystemExit(1)

            # 只读检查 period 和 polarity
            period_val = _read_text(f"{pwm_path}/period")
            polarity_val = _read_text(f"{pwm_path}/polarity")
            if period_val != str(PWM_PERIOD_NS):
                rospy.logwarn("PWM %s: period=%s (期望 %d)。init_pwm.sh 是否已运行？",
                              pwm_path, period_val, PWM_PERIOD_NS)
            if polarity_val != "normal":
                rospy.logwarn("PWM %s: polarity=%s (期望 normal)", pwm_path, polarity_val)

            ch = PWMChannel(pwm_path, slot_names[i])
            self.channels.append(ch)
            rospy.loginfo("slot %d (%s): %s, duty=%s, enable=%s",
                          i + 1, slot_names[i], pwm_path,
                          ch.read_duty(), ch.read_enable())

        # ROS 服务
        rospy.Service("SetServoDuty", SetServoDuty, self._handle_set_duty)
        rospy.loginfo("Service /SetServoDuty ready.")

        # 状态发布
        self.status_pub = rospy.Publisher("servo_debug/status", Float32MultiArray, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self._publish_status)  # 10Hz

        rospy.loginfo("servo_debug_node 启动完成，等待远程调试连接。")

    def _handle_set_duty(self, req):
        resp = SetServoDutyResponse()
        slot = req.slot
        duty = int(req.duty_cycle_ns)
        hold = bool(req.hold)

        if slot < 1 or slot > 3:
            resp.success = False
            resp.message = f"slot 必须是 1,2,3; got {slot}"
            return resp

        if duty < DUTY_MIN_NS or duty > DUTY_MAX_NS:
            resp.success = False
            resp.message = f"duty_cycle_ns {duty} 超出安全范围 [{DUTY_MIN_NS}, {DUTY_MAX_NS}]"
            return resp

        ch = self.channels[slot - 1]
        rospy.loginfo("SET slot=%d(%s) duty=%d hold=%s", slot, ch.slot_name, duty, hold)
        ok, actual, msg = ch.safe_pulse(duty, hold=hold, wait_sec=0.3)
        resp.success = ok
        resp.message = msg
        resp.actual_duty_ns = actual
        if ok:
            rospy.loginfo("  -> ok (actual duty=%d)", actual)
        else:
            rospy.logerr("  -> FAIL: %s", msg)
        return resp

    def _publish_status(self, event):
        """发布当前所有通道的 duty 和 enable 状态。"""
        msg = Float32MultiArray()
        # [rear_duty, right_duty, left_duty, rear_en, right_en, left_en]
        data = []
        for ch in self.channels:
            data.append(float(ch.read_duty()))
            data.append(1.0 if ch.read_enable() else 0.0)
        msg.data = data
        self.status_pub.publish(msg)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        ServoDebugNode().run()
    except SystemExit:
        pass
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logfatal("servo_debug_node 异常退出: %s", exc)
        raise
