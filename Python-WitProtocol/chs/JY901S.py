# coding: UTF-8
"""
JY901S sample with a simple NEU forward viewer.

The GUI only shows the body +Y axis (forward) expressed in NEU:
- N: North
- W: West
- U: Up
"""

import datetime
import math
import platform
import threading
import time

import lib.device_model as deviceModel
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver

welcome = """
欢迎使用维特智能示例程序    Welcome to the Wit-Motoin sample program
"""

_writeF = None
_IsWriteF = False
_pose_lock = threading.Lock()
_latest_forward_neu = None


def _normalize(vector):
    length = math.sqrt(sum(component * component for component in vector))
    if length < 1e-9:
        return None
    return [component / length for component in vector]


def solve_forward_in_neu(angle_x_deg, angle_z_deg):
    """
    Convert pitch/yaw to the body +Y axis in NEU.

    Assumptions used here:
    - angleX is pitch, positive when the nose goes up
    - angleZ is yaw, clockwise from North
    - body +Y is the forward direction
    """
    pitch = math.radians(angle_x_deg)
    yaw = math.radians(angle_z_deg)
    return _normalize([
        math.cos(yaw) * math.cos(pitch),
        math.sin(yaw) * math.cos(pitch),
        math.sin(pitch),
    ])


class NeuForwardViewer:
    def __init__(self):
        self.fig = plt.figure("JY901S +Y Forward in NEU")
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.fig.canvas.mpl_connect("close_event", self._on_close)
        self._closed = False
        self.anim = FuncAnimation(self.fig, self._update_frame, interval=100, cache_frame_data=False)

    def _on_close(self, _event):
        self._closed = True

    def _draw_arrow(self, origin, vector, color, label, linewidth=2.0):
        self.ax.quiver(
            origin[0],
            origin[1],
            origin[2],
            vector[0],
            vector[1],
            vector[2],
            color=color,
            linewidth=linewidth,
            arrow_length_ratio=0.12,
        )
        tip = [origin[i] + vector[i] for i in range(3)]
        self.ax.text(tip[0], tip[1], tip[2], label, color=color)

    def _update_frame(self, _frame_index):
        self.ax.cla()
        self.ax.set_title("Body +Y Forward in NEU")
        self.ax.set_xlabel("N")
        self.ax.set_ylabel("W")
        self.ax.set_zlabel("U")
        self.ax.set_xlim(-1.2, 1.2)
        self.ax.set_ylim(-1.2, 1.2)
        self.ax.set_zlim(-1.2, 1.2)
        self.ax.set_box_aspect((1, 1, 1))

        origin = [0.0, 0.0, 0.0]
        self._draw_arrow(origin, [1.0, 0.0, 0.0], "tab:red", "N")
        self._draw_arrow(origin, [0.0, 1.0, 0.0], "tab:green", "W")
        self._draw_arrow(origin, [0.0, 0.0, 1.0], "tab:blue", "U")

        with _pose_lock:
            forward = _latest_forward_neu[:] if _latest_forward_neu is not None else None

        if forward is None:
            self.ax.text2D(0.03, 0.95, "Waiting for IMU data...", transform=self.ax.transAxes)
            return

        self._draw_arrow(origin, forward, "black", "+Y forward", linewidth=3.0)
        self.ax.text2D(
            0.03,
            0.95,
            "Forward(NEU)=({:.3f}, {:.3f}, {:.3f})".format(*forward),
            transform=self.ax.transAxes,
        )

    def show(self):
        plt.show()

    @property
    def closed(self):
        return self._closed


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
    global _latest_forward_neu

    angle_x = device.getDeviceData("angleX")
    angle_y = device.getDeviceData("angleY")
    angle_z = device.getDeviceData("angleZ")
    forward_text = "None,None,None"
    forward_components = [None, None, None]

    if angle_x is not None and angle_z is not None:
        forward = solve_forward_in_neu(angle_x, angle_z)
        if forward is not None:
            with _pose_lock:
                _latest_forward_neu = forward
            forward_components = forward
            forward_text = "{:.3f},{:.3f},{:.3f}".format(*forward)

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
        "+Y前向(NEU):" + forward_text,
    )

    if _IsWriteF:
        temp_str = " " + str(device.getDeviceData("Chiptime"))
        temp_str += "\t" + str(device.getDeviceData("accX")) + "\t" + str(device.getDeviceData("accY")) + "\t" + str(device.getDeviceData("accZ"))
        temp_str += "\t" + str(device.getDeviceData("gyroX")) + "\t" + str(device.getDeviceData("gyroY")) + "\t" + str(device.getDeviceData("gyroZ"))
        temp_str += "\t" + str(device.getDeviceData("angleX")) + "\t" + str(device.getDeviceData("angleY")) + "\t" + str(device.getDeviceData("angleZ"))
        temp_str += "\t" + str(device.getDeviceData("temperature"))
        temp_str += "\t" + str(device.getDeviceData("magX")) + "\t" + str(device.getDeviceData("magY")) + "\t" + str(device.getDeviceData("magZ"))
        temp_str += "\t" + str(device.getDeviceData("lon")) + "\t" + str(device.getDeviceData("lat"))
        temp_str += "\t" + str(device.getDeviceData("Yaw")) + "\t" + str(device.getDeviceData("Speed"))
        temp_str += "\t" + str(device.getDeviceData("q1")) + "\t" + str(device.getDeviceData("q2"))
        temp_str += "\t" + str(device.getDeviceData("q3")) + "\t" + str(device.getDeviceData("q4"))
        temp_str += "\t" + str(forward_components[0]) + "\t" + str(forward_components[1]) + "\t" + str(forward_components[2])
        temp_str += "\r\n"
        _writeF.write(temp_str)
        _writeF.flush()


def startRecord():
    global _writeF
    global _IsWriteF

    filename = datetime.datetime.now().strftime("%Y%m%d%H%M%S") + ".txt"
    _writeF = open(filename, "w", encoding="utf-8")
    _IsWriteF = True

    header = "Chiptime"
    header += "\tax(g)\tay(g)\taz(g)"
    header += "\twx(deg/s)\twy(deg/s)\twz(deg/s)"
    header += "\tAngleX(deg)\tAngleY(deg)\tAngleZ(deg)"
    header += "\tT(degC)"
    header += "\tmagx\tmagy\tmagz"
    header += "\tlon\tlat"
    header += "\tYaw\tSpeed"
    header += "\tq1\tq2\tq3\tq4"
    header += "\tForwardN\tForwardE\tForwardU"
    header += "\r\n"
    _writeF.write(header)
    _writeF.flush()
    print("开始记录数据: " + filename)


def endRecord():
    global _writeF
    global _IsWriteF

    _IsWriteF = False
    if _writeF is not None:
        _writeF.close()
        _writeF = None
    print("结束记录数据")


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

    startRecord()
    print("正在采集数据，关闭图形窗口即可退出。")
    viewer = NeuForwardViewer()

    try:
        viewer.show()
    except KeyboardInterrupt:
        print("收到中断，正在关闭设备。")
    finally:
        device.closeDevice()
        if _IsWriteF:
            endRecord()
