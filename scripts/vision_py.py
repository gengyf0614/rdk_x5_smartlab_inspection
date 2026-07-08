#!/usr/bin/env python3
"""
lab_inspection 视觉识别节点 (Python 双模型并集版 + 华为云全链路)
- 接收 /trigger_inspection 服务 → 拍照 → 双模型推理并集 → 华为云上报
- 模型A: best.bin(擅长检测 dmm)
- 模型B: bestn.bin (擅长检测 psu)
- 并集策略: 同类别 IoU > merge_iou 取置信度高的
- 华为云: L610 4G 模组 AT 命令 → MQTT 连接 → 属性上报
- 大模型: 千问 VL 对杂乱区域二次研判
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from lab_inspection.srv import TriggerInspection
from std_srvs.srv import Trigger
import numpy as np
import cv2
import os
import time
import threading
import json
import base64
import serial
import re
from openai import OpenAI

CLASS_NAMES = [
    'chair_untidy', 'osc_on', 'osc_off',
    'siggen_on', 'siggen_off', 'psu_on',
    'psu_off', 'dmm_on', 'dmm_off', 'mess'
]

CLASS_CHINESE = {
    'chair_untidy': '过道椅子未归位',
    'osc_on':       '示波器开机',
    'osc_off':      '示波器关机',
    'siggen_on':    '信号发生器开机',
    'siggen_off':   '信号发生器关机',
    'psu_on':       '电源开机',
    'psu_off':      '电源关机',
    'dmm_on':       '万用表开机',
    'dmm_off':      '万用表关机',
    'mess':         '桌面杂乱区域'
}

ON_OFF_PAIRS = [(1, 2), (3, 4), (5, 6), (7, 8)]
INPUT_SIZE = 640


class DualModelVision(Node):
    def __init__(self):
        super().__init__('vision_py')

        # ==================== 模型参数 ====================
        self.declare_parameter('model_a', 'models/best.onnx')
        self.declare_parameter('model_b', 'models/617.onnx')
        self.declare_parameter('conf_thres', 0.15)
        self.declare_parameter('iou_thres', 0.45)
        self.declare_parameter('merge_iou', 0.5)
        self.declare_parameter('camera_topic', '/image')

        self.model_a_path = self.get_parameter('model_a').value
        self.model_b_path = self.get_parameter('model_b').value
        self.conf_thres = self.get_parameter('conf_thres').value
        self.iou_thres = self.get_parameter('iou_thres').value
        self.merge_iou = self.get_parameter('merge_iou').value
        self.camera_topic = self.get_parameter('camera_topic').value

        # ==================== 华为云 + L610 参数 ====================
        self.declare_parameter('huawei_mqtt_server', '517c1b2cc8.st1.iotda-device.cn-north-4.myhuaweicloud.com')
        self.declare_parameter('huawei_mqtt_port', 1883)
        self.declare_parameter('device_id', '6a172279cbb0cf6bb95f1880_lab_inspection')
        self.declare_parameter('device_secret', 'g1234567')
        self.declare_parameter('qwen_api_key', 'sk-33a2d028f38847b6a625f3f3efdcac84')
        self.declare_parameter('uart_device', '/dev/ttyUSB7')
        self.declare_parameter('uart_baudrate', 115200)
        self.declare_parameter('forgotten_items_dict', ['手机', '水杯', '耳机', '鼠标', '笔', '眼镜'])

        self.huawei_mqtt_server = self.get_parameter('huawei_mqtt_server').value
        self.huawei_mqtt_port = self.get_parameter('huawei_mqtt_port').value
        self.device_id = self.get_parameter('device_id').value
        self.device_secret = self.get_parameter('device_secret').value
        self.qwen_api_key = self.get_parameter('qwen_api_key').value
        self.uart_device = self.get_parameter('uart_device').value
        self.uart_baudrate = self.get_parameter('uart_baudrate').value
        self.forgotten_items_dict = self.get_parameter('forgotten_items_dict').value
        self.mqtt_connected = False
        self.uart_lock = threading.Lock()

        # 记录 1 号工位凳子归位状态，2/3/4 工位上报
        self.last_chair_status = 0

        # ==================== 加载 ONNX 模型 ====================
        import onnxruntime as ort
        self.get_logger().info(f'加载模型A: {self.model_a_path}')
        self.sess_a = ort.InferenceSession(self.model_a_path, providers=['CPUExecutionProvider'])
        self.in_a = self.sess_a.get_inputs()[0].name
        self.get_logger().info(f'加载模型B: {self.model_b_path}')
        self.sess_b = ort.InferenceSession(self.model_b_path, providers=['CPUExecutionProvider'])
        self.in_b = self.sess_b.get_inputs()[0].name

        # ==================== 初始化 L610 串口 ====================
        self.uart_fd = None
        self._init_l610_uart()

        # ==================== 摄像头订阅 ====================
        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_frame_time = 0.0
        self.frame_count = 0
        self.frame_lock = threading.Lock()
        self.sub = self.create_subscription(Image, self.camera_topic, self._img_cb, 5)

        # ==================== ROS 服务 ====================
        self.srv = self.create_service(
            TriggerInspection, '/trigger_inspection', self._handle_inspection)
        self.connect_srv = self.create_service(
            Trigger, '/connect_huawei_cloud', self._connect_cloud_cb)
        self.disconnect_srv = self.create_service(
            Trigger, '/disconnect_huawei_cloud', self._disconnect_cloud_cb)

        self.get_logger().info('双模型视觉节点已就绪 (华为云+L610+千问全链路)')

    # ==================== L610 串口初始化 ====================
    def _init_l610_uart(self):
        """初始化 L610 4G 模组串口"""
        try:
            self.uart_fd = serial.Serial(
                port=self.uart_device,
                baudrate=self.uart_baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.5,
                xonxoff=False,   # 禁用软件流控 (匹配C++ ~IXON|IXOFF|IXANY)
                rtscts=False,    # 禁用硬件流控
                dsrdtr=False     # 禁用DSR/DTR
            )
            self.get_logger().info(f'L610 串口已打开: {self.uart_device} @ {self.uart_baudrate}')
        except Exception as e:
            self.get_logger().error(f'无法打开串口 {self.uart_device}: {e}')
            self.uart_fd = None
            return

        # 关闭回显
        self._send_at('ATE0', 'OK', 1000)
        # 检查模块响应
        if not self._send_at('AT', 'OK', 1000):
            self.get_logger().error('L610 模块无应答! 请检查串口接线')
            self.uart_fd = None
            return
        self.get_logger().info('L610 模块响应正常')

        # 检查 SIM 卡
        if not self._send_at('AT+CPIN?', 'READY', 3000):
            self.get_logger().error('SIM 卡未就绪!')
            return

        # 检查信号质量
        reply = self._send_at('AT+CSQ', 'CSQ:', 3000, get_reply=True)
        if reply:
            self.get_logger().info(f'信号质量: {reply.strip()}')

        # 检查网络注册
        reply = self._send_at('AT+CEREG?', '+CEREG:', 5000, get_reply=True)
        if reply:
            if '0,1' in reply or '0,5' in reply:
                self.get_logger().info('网络已注册 (EPS附着成功)')
            else:
                self.get_logger().warn(f'网络未注册: {reply.strip()}')

        self.get_logger().info('L610 初始化完成')

    # ==================== AT 命令转义 ====================
    @staticmethod
    def _escape_at(s):
        """AT 命令字符串转义: 转义 " \ , """
        result = []
        for c in s:
            if c == '"':
                result.append('\\"')
            elif c == '\\':
                result.append('\\\\')
            elif c == ',':
                result.append('\\,')
            else:
                result.append(c)
        return ''.join(result)

    @staticmethod
    def _escape_quoted_at(s):
        """引号内字符串转义: 只转义 " \ """
        result = []
        for c in s:
            if c == '"':
                result.append('\\"')
            elif c == '\\':
                result.append('\\\\')
            else:
                result.append(c)
        return ''.join(result)

    # ==================== AT 命令收发 ====================
    def _send_at(self, cmd, expected='OK', timeout_ms=1000, get_reply=False):
        """发送 AT 命令并等待期望回复"""
        if self.uart_fd is None:
            return None if get_reply else False
        with self.uart_lock:
            try:
                self.uart_fd.reset_input_buffer()
                self.uart_fd.reset_output_buffer()
                full_cmd = cmd + '\r\n'
                self.uart_fd.write(full_cmd.encode())

                reply = ''
                deadline = time.time() + timeout_ms / 1000.0
                while time.time() < deadline:
                    if self.uart_fd.in_waiting > 0:
                        chunk = self.uart_fd.read(self.uart_fd.in_waiting).decode('utf-8', errors='replace')
                        reply += chunk
                        if expected in reply:
                            return reply if get_reply else True
                    time.sleep(0.01)

                if reply:
                    self.get_logger().warn(f'AT超时: {cmd} -> 收到: {reply[:200]}')
                else:
                    self.get_logger().warn(f'AT超时(无响应): {cmd}')
                return reply if get_reply else False
            except Exception as e:
                self.get_logger().error(f'AT命令异常: {cmd}, {e}')
                return None if get_reply else False

    # ==================== PDP 激活 ====================
    def _ensure_pdp_active(self):
        """确保 PDP 已激活并有 IP"""
        reply = self._send_at('AT+MIPCALL?', '+MIPCALL:', 3000, get_reply=True)
        if reply and ('+MIPCALL: 1,' in reply or '+MIPCALL:1,' in reply):
            self.get_logger().info('PDP 已有 IP，无需重新激活')
            return True

        self.get_logger().info('PDP 未激活，发送 AT+MIPCALL=1 ...')
        self._send_at('AT+MIPCALL=1', '', 2000)  # 不等待特定回复，等待 URC

        # 等 +MIPCALL URC
        if self.uart_fd:
            with self.uart_lock:
                try:
                    self.uart_fd.reset_input_buffer()
                except:
                    pass
        deadline = time.time() + 15.0
        urc_buf = ''
        while time.time() < deadline:
            try:
                if self.uart_fd and self.uart_fd.in_waiting > 0:
                    with self.uart_lock:
                        chunk = self.uart_fd.read(self.uart_fd.in_waiting).decode('utf-8', errors='replace')
                    urc_buf += chunk
                    if '+MIPCALL:' in urc_buf and '+MIPCALL: 0' not in urc_buf:
                        self.get_logger().info(f'PDP 激活成功: {urc_buf.strip()[:200]}')
                        return True
                    if 'ERROR' in urc_buf:
                        self.get_logger().warn(f'PDP 激活失败: {urc_buf.strip()[:200]}')
                        return False
            except:
                pass
            time.sleep(0.1)
        self.get_logger().warn(f'PDP 激活超时，收到的URC: {urc_buf[:200]}')
        return False

    # ==================== 华为云 MQTT 连接 ====================
    def connect_huawei_cloud(self):
        """通过 L610 AT+HMCON 连接华为云 IoTDA"""
        if self.mqtt_connected:
            self.get_logger().info('华为云已连接')
            return True
        if self.uart_fd is None:
            self.get_logger().error('串口未打开，无法连接华为云')
            return False

        # 1. 确保 PDP 激活
        if not self._ensure_pdp_active():
            self.get_logger().error('PDP 激活失败，无法连接华为云')

        # 2. 构建 AT+HMCON 命令
        hmcon = (
            f'AT+HMCON=0,60,'
            f'"{self._escape_at(self.huawei_mqtt_server)}",'
            f'"{self._escape_at(str(self.huawei_mqtt_port))}",'
            f'"{self._escape_at(self.device_id)}",'
            f'"{self._escape_at(self.device_secret)}",'
            f'0'
        )
        self.get_logger().info(
            f'正在连接华为云: {self.huawei_mqtt_server}:{self.huawei_mqtt_port}, '
            f'device={self.device_id}'
        )

        if not self._send_at(hmcon, '+HMCON OK', 20000):
            self.get_logger().error(
                'HMCON 连接失败! 请检查: 1)SIM卡有流量 2)服务器地址正确 3)设备已注册 4)密钥正确'
            )
            return False

        self.mqtt_connected = True
        self.get_logger().info('华为云 MQTT 连接成功 (+HMCON OK)')
        return True

    # ==================== 华为云 MQTT 断开 ====================
    def disconnect_huawei_cloud(self):
        """断开华为云 MQTT 并释放 IP"""
        if not self.mqtt_connected:
            self.get_logger().info('华为云未连接，跳过断开')
            return

        # 断开 MQTT
        self.get_logger().info('断开 MQTT: AT+HMDISC=0')
        if self._send_at('AT+HMDISC=0', '+HMDIS OK', 10000):
            self.get_logger().info('MQTT 已断开 (+HMDIS OK)')
        else:
            self.get_logger().warn('HMDISC 无响应')
        self.mqtt_connected = False

        # 释放 IP
        self.get_logger().info('释放 PDP: AT+MIPCALL=0')
        self._send_at('AT+MIPCALL=0', '+MIPCALL:', 10000)

    # ==================== 华为云属性上报 ====================
    def report_to_huawei_cloud(self, payload, payload_len: int = None):
        """通过 AT+HMPUB 上报属性到华为云 IoTDA（直接串口I/O，类C++实现）。
        payload 可以是 dict 或已经构造好的 JSON 字符串。
        """
        if not self.mqtt_connected:
            self.get_logger().error('华为云未连接，上报失败! 请先调用 /connect_huawei_cloud')
            return False
        if self.uart_fd is None:
            self.get_logger().error('串口未打开')
            self.mqtt_connected = False
            return False

        if isinstance(payload, dict):
            json_str = json.dumps(payload, ensure_ascii=False)
        else:
            json_str = payload

        topic = f'$oc/devices/{self.device_id}/sys/properties/report'
        if payload_len is None:
            payload_len = len(json_str.encode('utf-8'))

        actual_len = len(json_str.encode('utf-8'))
        if actual_len != payload_len:
            self.get_logger().warn(f'payload length mismatch: expected {payload_len}, actual {actual_len}')
            payload_len = actual_len

        self.get_logger().info(f'上报: topic={topic}, len={payload_len}')
        self.get_logger().info(f'Payload: {json_str[:500]}')

        escaped_json = self._escape_quoted_at(json_str)
        pub_cmd = (
            f'AT+HMPUB=1,'
            f'"{self._escape_at(topic)}",'
            f'{payload_len},'
            f'"{escaped_json}"'
        )
        self.get_logger().info(f'AT+HMPUB(前200字符): {pub_cmd[:200]}')

        with self.uart_lock:
            try:
                # 清空缓冲区（与 C++ tcflush(TCIOFLUSH) 等价）
                self.uart_fd.reset_input_buffer()
                self.uart_fd.reset_output_buffer()

                # 写入 AT+HMPUB 命令（\r\n 作为命令终止符）
                full_cmd = pub_cmd + '\r\n'
                self.uart_fd.write(full_cmd.encode())
                self.uart_fd.flush()  # 确保数据发出

                # 读取响应（类C++的read循环，检查多种终止条件）
                reply = ''
                deadline = time.time() + 10.0
                ok = False
                while time.time() < deadline:
                    if self.uart_fd.in_waiting > 0:
                        chunk = self.uart_fd.read(self.uart_fd.in_waiting)
                        try:
                            chunk_str = chunk.decode('utf-8', errors='replace')
                        except:
                            chunk_str = chunk.decode('latin-1', errors='replace')
                        reply += chunk_str
                        # ★ 关键: 匹配 +HMPUB OK（注意空格，区别于 AT+HMPUB= 回显）
                        if '+HMPUB OK' in reply:
                            ok = True
                            break
                        if '+HMPUB ERR' in reply:
                            self.get_logger().warn(f'HMPUB返回错误, 完整响应: {reply.strip()[:300]}')
                            break
                        if 'ERROR' in reply:
                            self.get_logger().warn(f'HMPUB命令被拒绝, 收到: {reply.strip()[:300]}')
                            break
                    time.sleep(0.02)

                self.get_logger().info(f'HMPUB原始响应(前300字符): {reply.strip()[:300]}')

            except Exception as e:
                self.get_logger().error(f'HMPUB串口异常: {e}')
                self.mqtt_connected = False
                return False

        if ok:
            self.get_logger().info('>>> 华为云数据上报成功 (+HMPUB OK)')
            return True
        else:
            self.get_logger().error(f'>>> 华为云上报失败! 完整响应: {reply.strip()[:400]}')
            self.mqtt_connected = False
            return False

    # ==================== 千问大模型 API ====================
    def call_qwen_llm(self, img_bgr):
        """调用千问 VL 大模型进行二次研判（使用 OpenAI 兼容 SDK，参考 image_upload_analyzer 模式）"""
        if not self.qwen_api_key:
            self.get_logger().warn('千问 API Key 未配置，跳过大模型研判')
            return ''

        # 图片编码为 JPEG 并转 base64（质量 90，对齐 image_upload_analyzer）
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        _, buf = cv2.imencode('.jpg', img_bgr, encode_param)
        base64_image = base64.b64encode(buf).decode('utf-8')

        # 构建遗忘物品提示
        dict_str = ''.join(f'「{item}」' for item in self.forgotten_items_dict)
        prompt = (
            '你是一个实验室巡检助手。请分析图中的桌面区域，判断是否存在异常。\n\n'
            '【判断规则】\n'
            f'1. 如果图中包含以下常用物品：{dict_str}，请回复格式：「遗落物品: [物品名称1]」\n'
            '2. 如果图中存在杂乱线缆、垃圾等实验室专属的凌乱的物品，请回复：「桌面杂乱1」\n'
            '3. 如果图片内容正常、整洁，没有明显问题，请回复：「无异常1」\n\n'
            '【重要】\n'
            '- 只回复上述三种格式之一，不要添加任何解释\n'
            '- 物品名称使用中文\n'
            '- 如果有多个物品，只回复最主要的一个\n'
        )

        try:
            # 初始化 OpenAI 客户端，指向 DashScope 兼容 API（与 Ark SDK 模式一致）
            self.get_logger().info('正在调用千问大模型分析图像数据...')
            client = OpenAI(
                api_key=self.qwen_api_key,
                base_url='https://dashscope.aliyuncs.com/compatible-mode/v1',
            )
            resp = client.chat.completions.create(
                model='qwen-vl-plus',
                messages=[{
                    'role': 'user',
                    'content': [
                        # 使用数据 URI 方案传递 Base64 编码的图像
                        {'type': 'image_url', 'image_url': {'url': f'data:image/jpeg;base64,{base64_image}'}},
                        {'type': 'text', 'text': prompt},
                    ],
                }],
            )

            # 提取并整理研判结果
            result = resp.choices[0].message.content
            result = result.replace('\n', '').replace('\r', '').strip()
            self.get_logger().info(f'千问研判: {result}')
            return result

        except Exception as e:
            self.get_logger().error(f'千问大模型分析过程中发生错误: {str(e)}')
        return ''

    def _parse_qwen_mess_result(self, result: str):
        """解析千问返回结果，生成 has_mess 和 mess_detail 上报值。"""
        normalized = result.strip()
        if normalized.startswith('遗落物品:'):
            item = normalized.split(':', 1)[1].strip()
            if item == '水杯':
                return 0, 0, f'遗落物品: {item}'
            if item == '手机':
                return 0, 1, f'遗落物品: {item}'
            return 0, 0, f'遗落物品: {item}'
        if '桌面杂乱' in normalized:
            return 1, None, '桌面杂乱'
        if '无异常' in normalized:
            return 0, None, ''
        return None, None, ''

    def _build_property_payload(self, prop_name: str, value, station_id=None):
        """构造单个属性上报的 JSON 字符串和固定长度。"""
        if prop_name == 'station_id':
            payload = '{"services":[{"service_id":"lab_inspection","properties":{"station_id":%d}}]}' % station_id
            return payload, len(payload.encode('utf-8'))

        prop_value = 1 if int(value) == 1 else 0
        if prop_name in ('chair_status', 'oscilloscope', 'siggen', 'psu', 'dmm', 'has_mess'):
            payload = '{"services":[{"service_id":"lab_inspection","properties":{"%s":%d}}]}' % (
                prop_name, prop_value)
            return payload, len(payload.encode('utf-8'))

        if prop_name == 'mess_detail':
            msg_value = 1 if int(value) == 1 else 0
            payload = '{"services":[{"service_id":"lab_inspection","properties":{"mess_detail":%d}}]}' % msg_value
            return payload, len(payload.encode('utf-8'))

        raise ValueError(f'Unsupported property: {prop_name}')

    # ==================== 华为云连接管理服务回调 ====================
    def _connect_cloud_cb(self, request, response):
        self.get_logger().info('收到云端连接请求')
        ok = self.connect_huawei_cloud()
        response.success = ok
        response.message = '华为云连接成功' if ok else '华为云连接失败'
        return response

    def _disconnect_cloud_cb(self, request, response):
        self.get_logger().info('收到云端断开请求')
        self.disconnect_huawei_cloud()
        response.success = True
        response.message = '华为云已断开'
        return response

    def _img_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.frame_lock:
                self.latest_frame = frame
                self.latest_frame_time = time.time()
                self.frame_count += 1
        except Exception as e:
            self.get_logger().error(f'图像解码失败: {e}')

    def _preprocess(self, img):
        h, w = img.shape[:2]
        s = min(h, w)
        ox, oy = (w - s) // 2, (h - s) // 2
        crop = img[oy:oy + s, ox:ox + s]
        resized = cv2.resize(crop, (INPUT_SIZE, INPUT_SIZE))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        chw = rgb.transpose(2, 0, 1).astype(np.float32) / 255.0
        return np.expand_dims(chw, 0), s, ox, oy, h, w

    def _infer_one(self, sess, in_name, tensor):
        out = sess.run(None, {in_name: tensor})[0]
        pred = out[0].T
        boxes_cx = pred[:, :4]
        cls_scores = pred[:, 4:]
        scores = np.max(cls_scores, 1)
        ids = np.argmax(cls_scores, 1)
        mask = scores > self.conf_thres
        return boxes_cx[mask], scores[mask], ids[mask]

    def _map_boxes(self, boxes_cxcywh, s, ox, oy, iw, ih):
        if len(boxes_cxcywh) == 0:
            return np.empty((0, 4))
        cx, cy, bw, bh = boxes_cxcywh[:, 0], boxes_cxcywh[:, 1], boxes_cxcywh[:, 2], boxes_cxcywh[:, 3]
        x1, y1 = cx - bw / 2, cy - bh / 2
        x2, y2 = cx + bw / 2, cy + bh / 2
        boxes = np.stack([x1, y1, x2, y2], axis=1)
        scale = s / INPUT_SIZE
        boxes[:, [0, 2]] = boxes[:, [0, 2]] * scale + ox
        boxes[:, [1, 3]] = boxes[:, [1, 3]] * scale + oy
        boxes[:, 0] = np.clip(boxes[:, 0], 0, iw)
        boxes[:, 1] = np.clip(boxes[:, 1], 0, ih)
        boxes[:, 2] = np.clip(boxes[:, 2], 0, iw)
        boxes[:, 3] = np.clip(boxes[:, 3], 0, ih)
        return boxes

    def _nms(self, boxes, scores, ids):
        if len(boxes) == 0:
            return np.empty((0, 4)), np.empty((0,)), np.empty((0,))
        indices = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), self.conf_thres, self.iou_thres)
        if isinstance(indices, tuple):
            indices = indices[0]
        if len(indices) == 0:
            return np.empty((0, 4)), np.empty((0,)), np.empty((0,))
        idx = indices.flatten()
        return boxes[idx], scores[idx], ids[idx]

    def _iou(self, a, b):
        x1 = max(a[0], b[0]); y1 = max(a[1], b[1])
        x2 = min(a[2], b[2]); y2 = min(a[3], b[3])
        inter = max(0, x2 - x1) * max(0, y2 - y1)
        area_a = (a[2] - a[0]) * (a[3] - a[1])
        area_b = (b[2] - b[0]) * (b[3] - b[1])
        return inter / (area_a + area_b - inter + 1e-6)

    def _merge(self, boxes_a, scores_a, ids_a, boxes_b, scores_b, ids_b):
        if len(boxes_a) == 0:
            return boxes_b, scores_b, ids_b
        if len(boxes_b) == 0:
            return boxes_a, scores_a, ids_a

        all_boxes = np.vstack([boxes_a, boxes_b])
        all_scores = np.hstack([scores_a, scores_b])
        all_ids = np.hstack([ids_a, ids_b])

        keep = list(range(len(all_boxes)))
        for i in range(len(all_boxes)):
            if i not in keep:
                continue
            for j in range(i + 1, len(all_boxes)):
                if j not in keep:
                    continue
                if all_ids[i] != all_ids[j]:
                    continue
                if self._iou(all_boxes[i], all_boxes[j]) > self.merge_iou:
                    if all_scores[i] >= all_scores[j]:
                        keep.remove(j)
                    else:
                        keep.remove(i)
                        break
        keep = sorted(keep)
        return all_boxes[keep], all_scores[keep], all_ids[keep]

    def _apply_on_priority(self, ids, scores, boxes):
        if len(ids) == 0:
            return ids, scores, boxes
        present = set(int(i) for i in ids)
        mask = np.ones(len(ids), dtype=bool)
        for on_idx, off_idx in ON_OFF_PAIRS:
            if on_idx in present and off_idx in present:
                for i, cid in enumerate(ids):
                    if int(cid) == off_idx:
                        mask[i] = False
        return ids[mask], scores[mask], boxes[mask]

    def _handle_inspection(self, request, response):
        station_id = request.station_id
        self.get_logger().info(f'======> 巡检触发 工位[{station_id}]')

        with self.frame_lock:
            if self.latest_frame is None:
                response.success = False
                response.message = '错误: 尚未收到摄像头图像'
                return response
            frame_age = time.time() - self.latest_frame_time
            img = self.latest_frame.copy()
            fc = self.frame_count

        self.get_logger().info(
            f'拍照: 帧#{fc} 已缓存{frame_age:.1f}s 尺寸{img.shape[1]}x{img.shape[0]}')
        os.makedirs('/tmp/inspection_snapshots', exist_ok=True)
        snap = f'/tmp/inspection_snapshots/station{station_id}_{int(time.time())}.jpg'
        cv2.imwrite(snap, img)
        self.get_logger().info(f'快照已保存: {snap}')

        tensor, s, ox, oy, ih, iw = self._preprocess(img)

        t0 = time.time()
        boxes_cx_a, scores_a, ids_a = self._infer_one(self.sess_a, self.in_a, tensor)
        boxes_cx_b, scores_b, ids_b = self._infer_one(self.sess_b, self.in_b, tensor)

        boxes_a = self._map_boxes(boxes_cx_a, s, ox, oy, iw, ih)
        boxes_b = self._map_boxes(boxes_cx_b, s, ox, oy, iw, ih)

        boxes_a, scores_a, ids_a = self._nms(boxes_a, scores_a, ids_a)
        boxes_b, scores_b, ids_b = self._nms(boxes_b, scores_b, ids_b)

        boxes, scores, ids = self._merge(boxes_a, scores_a, ids_a,
                                          boxes_b, scores_b, ids_b)

        ids, scores, boxes = self._apply_on_priority(ids, scores, boxes)

        elapsed = (time.time() - t0) * 1000
        self.get_logger().info(
            f'推理完成 [{elapsed:.0f}ms] '
            f'Ax{len(boxes_a)}+Bx{len(boxes_b)}->{len(boxes)}')

        has_chair = False
        has_abnormal = False
        report_items = []
        messy_rois = []  # ROI for LLM judgment
        status = {'chair': 'normal', 'osc': 'UNKNOWN', 'siggen': 'UNKNOWN',
                  'psu': 'UNKNOWN', 'dmm': 'UNKNOWN', 'mess': False}

        for i in range(len(boxes)):
            cid = int(ids[i])
            name = CLASS_NAMES[cid]
            conf = scores[i]
            self.get_logger().info(f'  检测: {name} conf={conf:.3f}')

            if cid == 0:
                has_chair = True
                has_abnormal = True
                status['chair'] = 'untidy'
                report_items.append('过道椅子未归位')
            elif cid == 1:
                has_abnormal = True
                status['osc'] = 'ON'
                report_items.append('示波器未关机')
            elif cid == 2:
                status['osc'] = 'OFF'
            elif cid == 3:
                has_abnormal = True
                status['siggen'] = 'ON'
                report_items.append('信号发生器未关机')
            elif cid == 4:
                status['siggen'] = 'OFF'
            elif cid == 5:
                has_abnormal = True
                status['psu'] = 'ON'
                report_items.append('稳压电源未关机')
            elif cid == 6:
                status['psu'] = 'OFF'
            elif cid == 7:
                has_abnormal = True
                status['dmm'] = 'ON'
                report_items.append('万用表未关机')
            elif cid == 8:
                status['dmm'] = 'OFF'
            elif cid == 9:
                has_abnormal = True
                status['mess'] = True
                # 保存 ROI 用于千问大模型研判
                x1, y1, x2, y2 = boxes[i].astype(int)
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(iw-1, x2), min(ih-1, y2)
                if x2 - x1 > 10 and y2 - y1 > 10:
                    messy_rois.append(img[y1:y2, x1:x2])

        if station_id == 1:
            self.last_chair_status = 1 if status['chair'] == 'untidy' else 0
            self.get_logger().info(f'1号工位凳子状态已记录: {self.last_chair_status}')
            report_station_id = None
        else:
            report_station_id = {2: 101, 3: 102, 4: 103}.get(station_id, station_id)
            self.get_logger().info(f'工位{station_id}上报 station_id={report_station_id}')

        # ==================== 千问大模型研判杂乱区域 ====================
        mess_prop = None
        mess_detail_prop = None
        messy_judgment = ''
        for roi in messy_rois:
            result = self.call_qwen_llm(roi)
            if result and result not in ('未知杂物', '网络波动: 连云失败', ''):
                has_mess, mess_detail, msg = self._parse_qwen_mess_result(result)
                if has_mess is not None:
                    messy_judgment = msg
                    mess_prop = has_mess
                    mess_detail_prop = mess_detail
                    if msg:
                        report_items.append(msg)
                    break

        if mess_prop is None:
            if status['mess']:
                mess_prop = 1
                report_items.append('桌面杂乱')
            else:
                mess_prop = 0

        if station_id != 1:
            chair_status_prop = self.last_chair_status
        else:
            chair_status_prop = None

        # ==================== 上报华为云 ====================
        if report_station_id is not None:
            report_commands = []
            report_commands.append(self._build_property_payload('station_id', report_station_id, station_id=report_station_id))
            if chair_status_prop is not None:
                report_commands.append(self._build_property_payload('chair_status', chair_status_prop))
            report_commands.append(self._build_property_payload('oscilloscope', 1 if status['osc'] == 'ON' else 0))
            report_commands.append(self._build_property_payload('siggen', 1 if status['siggen'] == 'ON' else 0))
            report_commands.append(self._build_property_payload('psu', 1 if status['psu'] == 'ON' else 0))
            report_commands.append(self._build_property_payload('dmm', 1 if status['dmm'] == 'ON' else 0))
            report_commands.append(self._build_property_payload('has_mess', mess_prop))
            if mess_detail_prop is not None:
                report_commands.append(self._build_property_payload('mess_detail', mess_detail_prop))

            # 异步上报（一条属性一条指令，按巡检顺序上传）
            threading.Thread(
                target=self._async_report,
                args=(report_commands,),
                daemon=True
            ).start()
        else:
            self.get_logger().info('工位1检测点不上传属性，仅记录椅子状态')

        # ==================== 日志汇总 ====================
        self.get_logger().info(
            f'设备状态 - 工位{station_id}: 椅子={status["chair"]}, '
            f'示波器={status["osc"]}, 信号源={status["siggen"]}, '
            f'电源={status["psu"]}, 万用表={status["dmm"]}, '
            f'杂乱={status["mess"]}, 异常={has_abnormal or has_chair}'
        )

        # ==================== 构建响应 ====================
        if not has_abnormal and not has_chair:
            response.success = True
            response.message = f'工位{station_id}正常: 全部关机+椅子归位'
            self.get_logger().info(f'工位{station_id} 巡检通过')
            return response

        response.success = True
        detail = '、'.join(report_items) if report_items else '检测到异常状态'
        response.message = f'工位{station_id}异常: {detail}'

        self.get_logger().info(f'巡检完成: {response.message}')
        return response

    def _async_report(self, report_commands):
        """异步上报华为云（避免阻塞巡检响应）。支持多条属性指令顺序上报。"""
        try:
            for json_str, payload_len in report_commands:
                self.report_to_huawei_cloud(json_str, payload_len)
                time.sleep(0.05)
        except Exception as e:
            self.get_logger().error(f'异步上报异常: {e}')


def main():
    rclpy.init()
    node = DualModelVision()
    # 使用 MultiThreadedExecutor 支持阻塞 I/O（串口、HTTP）
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        # 退出前断开华为云
        if node.mqtt_connected:
            node.disconnect_huawei_cloud()
        if node.uart_fd:
            node.uart_fd.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
