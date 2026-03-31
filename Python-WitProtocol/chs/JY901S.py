# coding: UTF-8
"""
JY901S sample with forward direction and plane-hit output.
"""

import math
import platform
import time

import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver

welcome = """
欢迎使用维特智能示例程序    Welcome to the Wit-Motoin sample program
"""

IMU_POSITION_NWU = [0.0, 0.0, 1.2]
TARGET_PLANE_W = 5.3


def _normalize(vector):
    length = math.sqrt(sum(component * component for component in vector))
    if length < 1e-9:
        return None
    return [component / length for component in vector]


def solve_forward_in_neu(angle_x_deg, angle_z_deg):
    """
    Keep the current user-defined forward-direction convention unchanged.
    """
    pitch = math.radians(angle_x_deg)
    yaw = math.radians(angle_z_deg)
    return _normalize([
        math.cos(yaw) * math.cos(pitch),
        math.sin(yaw) * math.cos(pitch),
        math.sin(pitch),
    ])


def solve_plane_intersection(origin, forward, plane_w):
    if forward is None:
        return None

    direction_w = forward[1]
    if abs(direction_w) < 1e-9:
        return None

    distance = (plane_w - origin[1]) / direction_w
    if distance < 0:
        return None

    return [
        origin[i] + distance * forward[i]
        for i in range(3)
    ]


def readConfig(device):
    tVals = device.readReg(0x02, 3)
    if len(tVals) > 0:
        print("返回结果：" + str(tVals))
    else:
        print("无返回")

    tVals = device.readReg(0x23, 2)
    if len(tVals) > 0:
        print("返回结果：" + str(tVals))
    else:
        print("无返回")


def setConfig(device):
    device.unlock()
    time.sleep(0.1)
    device.writeReg(0x03, 6)
    time.sleep(0.1)
    device.writeReg(0x23, 0)
    time.sleep(0.1)
    device.writeReg(0x24, 0)
    time.sleep(0.1)
    device.save()


def AccelerationCalibration(device):
    device.AccelerationCalibration()
    print("加计校准结束")


def FiledCalibration(device):
    device.BeginFiledCalibration()
    if input("请分别绕XYZ轴慢速转动一圈，三轴转圈完成后，结束校准（Y/N)？").lower() == "y":
        device.EndFiledCalibration()
        print("结束磁场校准")


def onUpdate(device):
    angle_x = device.getDeviceData("angleX")
    angle_y = device.getDeviceData("angleY")
    angle_z = device.getDeviceData("angleZ")
    forward_text = "None,None,None"
    hit_text = "None,None,None"

    if angle_x is not None and angle_z is not None:
        forward = solve_forward_in_neu(angle_x, angle_z)
        if forward is not None:
            hit_point = solve_plane_intersection(IMU_POSITION_NWU, forward, TARGET_PLANE_W)
            forward_text = "{:.3f},{:.3f},{:.3f}".format(*forward)
            if hit_point is not None:
                displayed_hit_point = [hit_point[0], 0.0, hit_point[2]]
                hit_text = "{:.3f},{:.3f},{:.3f}".format(*displayed_hit_point)

    print(
        "芯片时间:" + str(device.getDeviceData("Chiptime")),
        "温度:" + str(device.getDeviceData("temperature")),
        "加速度:" + str(device.getDeviceData("accX")) + "," + str(device.getDeviceData("accY")) + "," + str(device.getDeviceData("accZ")),
        "角速度:" + str(device.getDeviceData("gyroX")) + "," + str(device.getDeviceData("gyroY")) + "," + str(device.getDeviceData("gyroZ")),
        "角度:" + str(angle_x) + "," + str(angle_y) + "," + str(angle_z),
        "磁场:" + str(device.getDeviceData("magX")) + "," + str(device.getDeviceData("magY")) + "," + str(device.getDeviceData("magZ")),
        "经度:" + str(device.getDeviceData("lon")) + " 纬度:" + str(device.getDeviceData("lat")),
        "航向角:" + str(device.getDeviceData("Yaw")) + " 地速:" + str(device.getDeviceData("Speed")),
        "四元数:" + str(device.getDeviceData("q1")) + "," + str(device.getDeviceData("q2")) + "," + str(device.getDeviceData("q3")) + "," + str(device.getDeviceData("q4")),
        "+Y前向:" + forward_text,
        "平面交点:" + hit_text,
    )


if __name__ == "__main__":
    print(welcome)

    device = deviceModel.DeviceModel(
        "我的JY901",
        WitProtocolResolver(),
        JY901SDataProcessor(),
        "51_0",
    )

    if platform.system().lower() == "linux":
        device.serialConfig.portName = "/dev/ttyUSB0"
    else:
        device.serialConfig.portName = "COM19"

    device.serialConfig.baud = 115200
    device.openDevice()
    if not device.isOpen:
        print("串口打开失败，请检查端口号和占用情况")
        raise SystemExit(1)

    readConfig(device)
    device.dataProcessor.onVarChanged.append(onUpdate)

    print("正在采集数据，按回车键即可退出。")
    try:
        input()
    except KeyboardInterrupt:
        print("收到中断，正在关闭设备。")
    finally:
        device.closeDevice()
