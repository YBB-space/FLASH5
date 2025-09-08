"""
ICS FLASH5 block1
© HANWOOL All Rights Reserved
2025 YBB(ybb1833@naver.com)
"""
# ==== Standard Library ====
import math
import os
import random
import sys
import tempfile
import time
from datetime import datetime
from pathlib import Path

# ==== Third-Party ====
import numpy as np
import serial
from serial.tools import list_ports
import pyqtgraph as pg
import pyqtgraph.opengl as gl  # 3D 사용하지 않으면 삭제 가능

# ==== PyQt5 ====
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import (
    QObject,
    QThread,
    QTimer,
    QEasingCurve,
    QPropertyAnimation,
    QParallelAnimationGroup,
    pyqtSignal,
)
from PyQt5.QtGui import QColor, QIcon
from PyQt5.QtWidgets import QGraphicsOpacityEffect
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEngineProfile
from PyQt5.QtTest import QTest

# 이미지 파일 모음
intro_logo_img = Path(__file__).parent / "img" / "Flash_logo.png"
parameter1_gauge_img = Path(__file__).parent / "img" / "gauge" / "parameter1_gauge_01.png"
parameter2_gauge_img = Path(__file__).parent / "img" / "gauge" / "parameter2_gauge_01.png"
info_alarm_icon_img = Path(__file__).parent / "img" / "info-alarm-icon.png"
warning_alarm_icon_img = Path(__file__).parent / "img" / "warning-alarm-icon.png"
emergency_alarm_icon_img = Path(__file__).parent / "img" / "emergency-alarm-icon.png"
blue_btn_img = Path(__file__).parent / "img" / "blue_btn.png"
blue_btn_mini_img = Path(__file__).parent / "img" / "blue_btn_mini.png"
red_btn_img = Path(__file__).parent / "img" / "red_btn.png"
setting_btn_img = Path(__file__).parent / "img" / "setting.png"
export_btn_img = Path(__file__).parent / "img" / "export.png"
connecting_img = Path(__file__).parent / "img" / "connecting-icon.png"
connected_img = Path(__file__).parent / "img" / "connected-icon.png"
NoSignal_img = Path(__file__).parent / "img" / "No_signal-icon.png"
re_connected_icon_img = Path(__file__).parent / "img" / "re_connected_icon.png"


#    자동으로 연결될 아두이노 장치를 필터링하는 키워드입니다.
#    사용 중인 기기에 따라 아래 변수를 수정하세요.
#    예시:
#      - "USB Serial" → CH340, 일부 Uno 계열
#      - "Arduino"     → 공식 Arduino 보드
#      - "ttyACM"      → 리눅스 환경, Uno 등
#      - "cu.usbmodem" → macOS 환경
DEVICE_KEYWORD = "USB Serial"  # ← 이 값을 수정하세요!!

# stdout 잠깐 비우기
sys_stdout = sys.stdout
sys.stdout = open(os.devnull, 'w')
import pygame


port = ""
ignition_signal = 0
map_plus_count = 0
program_info_count = 0 # 프로그램 인포 버튼 클릭 확인
mode_count = 1 # 모드 변경 버튼 클릭 확인
mode_flight_count = 0 # 플라이트 모드 확인
mode_TMS_count = 0 # TMS 모드 확인
control_count = 0 # 컨트롤 버튼 클릭 확인
export_count = 0 # 익스포트 버튼 클릭 확인
terminal_count = 0 # 터미널 버튼 클릭 확인
safty_count = 0 # 세이프티 버튼 클릭 확인
chart_count = 0 # 차트 버튼 클릭 확인
set_count = 0 # 설정 버튼 클릭 확인
simulation_mode = 0 #시뮬레이션 모드 확인
abort = 0 #비행 중단 확인
t = 0 # 시퀀스 기본 시간
t_set = 20 # 시퀀스 선택 시간
sequence = 0 # 시퀀스 진행 확인
I_S = 5 # 수동 점화 = 0 , 시퀀스 시작 = 1 , 데이터 리셋 = 2 , 발사대 기립 = 3 , IFP_mode = 4 , Nomal = 5
intro_exit = 0 # 인트로 화면 확인
sim_ig = 0 # 시뮬레이션 모드 점화 확인
VFS_count = 1 #VFS 활성화 확인
ADI_count = 0 # ADI 활성화 확인
set_mode_count = 0
setting_interface_count = 1 #설정 1번 모드 확인

#여기 주석 다셈 ㅇㅇ 꼭!!!!!
IFP_confirm_popup_count = 0
simulation_data_count = 0
data_safe_count = 0
detail_log_count = 0
sequence_manual_ig__count = 0
IFP_count = 1
re_seq_count = 1

avg_parameter1_2 = 0
avg_parameter2_2 = 0
max_parameter1 = 0
max_parameter2 = 0

pygame.init()
pygame.mixer.init()

# stdout 복원
sys.stdout = sys_stdout



class SerialReaderThread(QThread):
    new_data_signal = pyqtSignal(str)   # 정상 수신 라인
    disconnected    = pyqtSignal(str)   # 포트 끊김/오류
    error           = pyqtSignal(str)   # 기타 오류 (선택)

    def __init__(self, serial_connection, parent=None):
        super().__init__(parent)
        self.serial_connection = serial_connection
        self.is_running = True

    def stop(self):
        self.is_running = False

    def run(self):
        while self.is_running:
            try:
                # 포트가 닫혔거나 None일 수 있음
                if not self.serial_connection or not getattr(self.serial_connection, "is_open", True):
                    raise Exception("Serial port is not open")

                # 대기 데이터 없으면 잠깐 쉼
                if getattr(self.serial_connection, "in_waiting", 0) <= 0:
                    self.msleep(1)
                    continue

                line = self.serial_connection.readline()
                if not line:
                    continue

                if isinstance(line, (bytes, bytearray)):
                    serial_data = line.decode("utf-8", "ignore").strip()
                else:
                    serial_data = str(line).strip()

                if serial_data:
                    self.new_data_signal.emit(serial_data)

            except (OSError, Exception) as e:
                # macOS에서 장치 제거 시 OSError(Errno 6) → 여기로 옴
                # pyserial SerialException도 여기에서 잡히게 구성 가능
                self.disconnected.emit(str(e))
                break  # 루프 종료

        # 여기서 자연 종료
        

class TimeUpdateThread(QObject):
    time_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.timer = QTimer()
        self.timer.timeout.connect(self.emit_time)

    def start(self):
        self.timer.start(1000)  # 1초 간격

    def stop(self):
        self.timer.stop()

    def emit_time(self):
        current_time = datetime.now().strftime("%H:%M:%S")
        self.time_signal.emit(current_time)

class ClickableLabel(QtWidgets.QLabel):
    clicked = QtCore.pyqtSignal()  # 클릭 시그널 정의

    def mousePressEvent(self, event):
        self.clicked.emit()  # 클릭되면 시그널 발생
        super().mousePressEvent(event)  # QLabel 기본 동작도 유지


class Ui_MainWindow(object):

    def _load_leaflet_html(self):
        leaflet_html = r"""
        <!DOCTYPE html>
        <html>
        <head>
        <meta charset="utf-8"/>
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
        <style>
            :root{
            --bg:#0f1115;
            --panel:rgba(255,255,255,0.10);
            --text:#0f172a;
            --muted:#475569;
            --border:rgba(15,23,42,0.10);
            --shadow:0 10px 30px rgba(0,0,0,.18);
            --glass: blur(14px) saturate(180%);
            }
            html,body,#map{
            height:100%; margin:0; background:var(--bg);
            font-family:Inter,system-ui,Segoe UI,Roboto,Apple SD Gothic Neo,Arial,sans-serif;
            }
            #map{ position:relative; overflow:hidden; }

            #tint{
            position:absolute; inset:0; pointer-events:none;
            background:rgba(0,0,0,0.06); z-index:200;
            }

            .leaflet-control-attribution{
            background:rgba(255,255,255,.85); color:#334155; border:none;
            }
            .leaflet-container a{ color:#2563eb; }
            .leaflet-popup-content-wrapper{
            background:rgba(255,255,255,.92); color:#0f172a;
            backdrop-filter: var(--glass); border-radius:12px;
            }
            .leaflet-popup-tip{ background:rgba(255,255,255,.92); }

            /* HUD */
            .hud{
            position:absolute; top:14px; left:14px;
            z-index:1000; display:flex; gap:10px; align-items:center; flex-wrap:wrap;
            pointer-events: none;
            }
            .card{
            pointer-events: auto;
            background: var(--panel);
            border:1px solid var(--border);
            backdrop-filter: var(--glass);
            border-radius:12px; padding:10px 12px; box-shadow:var(--shadow);
            display:flex; align-items:center; gap:10px;
            }
            .coords{ font-weight:700; letter-spacing:.2px; color:var(--text);}
            .coords small{ color:var(--muted); font-weight:600; margin-left:8px; white-space:nowrap; }

            .pill-btn{
            pointer-events: auto;
            appearance:none; -webkit-appearance:none; border:1px solid var(--border);
            background: rgba(255,255,255,0.92);
            padding:8px 12px; border-radius:999px;
            color:#0f172a; box-shadow:var(--shadow);
            display:inline-flex; align-items:center; gap:6px; cursor:pointer;
            font: inherit;
            }
            .pill-btn:hover{ background:#ffffff; }

            /* 🚀 글래스 원 마커 */
            .rocket-circle{
            width: 48px; height: 48px; border-radius: 50%;
            backdrop-filter: blur(12px) saturate(180%);
            background: linear-gradient(180deg, rgba(255,255,255,0.40), rgba(255,255,255,0.18));
            border: 1px solid rgba(255,255,255,0.35);
            box-shadow:
                0 8px 20px rgba(0,0,0,0.25),
                inset 0 1px 0 rgba(255,255,255,0.5);
            display:flex; align-items:center; justify-content:center;
            }
            .rocket-circle .emoji{
            font-size: 24px; line-height: 1;
            filter: drop-shadow(0 1px 1px rgba(0,0,0,0.2));
            }
        </style>
        </head>
        <body>
        <div id="map"></div>
        <div id="tint"></div>

        <div class="hud">
            <div class="card">
            <span class="coords" id="coordText">🚀 35.154244, 128.092930</span>
            <small id="zoomText">zoom 12</small>
            </div>
            <button class="pill-btn" id="btnRecenter" type="button">↺ Recenter</button>
            <button class="pill-btn" id="btnCopy" type="button">⧉ Copy</button>
        </div>

        <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
        <script>
            document.addEventListener('DOMContentLoaded', function(){
            var defaultLat = 35.154244, defaultLon = 128.092930;
            var map = L.map('map', { zoomControl: false }).setView([defaultLat, defaultLon], 12);

            L.tileLayer('https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png', {
                subdomains:'abcd', maxZoom:19,
                attribution:'&copy; OSM contributors &copy; CARTO'
            }).addTo(map);

            // 🚀 글래스 원 아이콘
            var rocketIcon = L.divIcon({
                className: "rocket-icon",
                html: "<div class='rocket-circle'><span class='emoji'>🚀</span></div>",
                iconSize: [48, 48],
                iconAnchor: [24, 48]
            });

            var marker = L.marker([defaultLat, defaultLon], { icon: rocketIcon }).addTo(map)
                            .bindPopup("Launch Vehicle");

            var coordText = document.getElementById('coordText');
            var zoomText  = document.getElementById('zoomText');
            var btnCopy   = document.getElementById('btnCopy');
            var btnRecenter = document.getElementById('btnRecenter');

            function fmt(v){ return Number(v).toFixed(5); }
            function updateHudFromMarker(){
                var ll = marker.getLatLng();
                coordText.textContent = "🚀 " + fmt(ll.lat) + ", " + fmt(ll.lng);
                zoomText.textContent = "zoom " + map.getZoom();
            }
            updateHudFromMarker();

            map.on('zoomend', function(){
                zoomText.textContent = "zoom " + map.getZoom();
            });

            async function copyTextSafe(text){
                try{
                if (navigator.clipboard && window.isSecureContext){
                    await navigator.clipboard.writeText(text);
                    return true;
                }
                }catch(e){}
                try{
                var ta = document.createElement('textarea');
                ta.value = text;
                ta.style.position = 'fixed';
                ta.style.opacity = '0';
                document.body.appendChild(ta);
                ta.focus(); ta.select();
                var ok = document.execCommand('copy');
                document.body.removeChild(ta);
                return ok;
                }catch(e){ return false; }
            }

            btnCopy.addEventListener('click', async function(e){
                e.preventDefault();
                var ll = marker.getLatLng();
                var t = fmt(ll.lat) + ", " + fmt(ll.lng);
                var ok = await copyTextSafe(t);
                var prev = btnCopy.textContent;
                btnCopy.textContent = ok ? "✓ Copied" : "Copy failed";
                setTimeout(()=>btnCopy.textContent = prev, 900);
            });

            btnRecenter.addEventListener('click', function(e){
                e.preventDefault();
                map.setView([defaultLat, defaultLon], 12);
                marker.setLatLng([defaultLat, defaultLon]).closePopup();
                updateHudFromMarker();
            });

            window.setCenter = function(lat, lon, zoom){
                map.setView([lat, lon], zoom || map.getZoom());
                zoomText.textContent = "zoom " + map.getZoom();
            }
            window.setMarker = function(lat, lon){
                marker.setLatLng([lat, lon]);
                updateHudFromMarker();
            }
            window.setCoordText = function(lat, lon){
                marker.setLatLng([lat, lon]);
                updateHudFromMarker();
            }
            });
        </script>
        </body>
        </html>
        """
        self.map_view.setHtml(leaflet_html)


    def set_map_center_throttled(self, lat, lon, zoom=None):
        """텔레메트리 수신 시 과도한 JS 호출 방지용 스로틀러"""
        self._pending_latlon = (lat, lon, zoom)
        if not self._map_upd_timer.isActive():
            self._map_upd_timer.start()

    def _flush_map_update(self):
        if not self._pending_latlon:
            self._map_upd_timer.stop()
            return
        lat, lon, zoom = self._pending_latlon
        self._pending_latlon = None
        z = "undefined" if zoom is None else zoom
        js = f"window.setCenter({lat}, {lon}, {z});"
        self.map_view.page().runJavaScript(js)

    def __init__(self):
        self.ser = None

    def setupUi(self, MainWindow):

        MainWindow.setObjectName("MainWindow")
        MainWindow.setFixedSize(1470, 836)
        MainWindow.setStyleSheet("font:  15pt \"Inter\";\n"
"background-color: rgb(29,29,29);")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.Sequence_time_text = QtWidgets.QLabel(self.centralwidget)
        self.Sequence_time_text.setGeometry(QtCore.QRect(30, 670, 271, 51))
        font = QtGui.QFont()
        font.setFamily("Advent Pro SemiExpanded")
        font.setPointSize(55)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Sequence_time_text.setFont(font)
        self.Sequence_time_text.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"color: rgb(255, 255, 255);\n"
"font: 55pt \"Advent Pro SemiExpanded\";")
        self.Sequence_time_text.setFrameShadow(QtWidgets.QFrame.Plain)
        self.Sequence_time_text.setObjectName("Sequence_time_text")
        self.parameter1_box1 = QtWidgets.QLabel(self.centralwidget)
        self.parameter1_box1.setGeometry(QtCore.QRect(1105, 645, 161, 171))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter1_box1.setFont(font)
        self.parameter1_box1.setStyleSheet("border-radius :15px;\n"
"background-color: rgb(20,20,20);")
        self.parameter1_box1.setText("")
        self.parameter1_box1.setAlignment(QtCore.Qt.AlignCenter)
        self.parameter1_box1.setObjectName("parameter1_box1")
        self.back_grad_up = QtWidgets.QLabel(self.centralwidget)
        self.back_grad_up.setGeometry(QtCore.QRect(0, 0, 1471, 211))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.back_grad_up.setFont(font)
        self.back_grad_up.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.back_grad_up.setStyleSheet("font:  15pt \"Inter\";\n"
"color: rgb(255, 255, 255);\n"
"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(0, 0, 0, 200), stop:1 rgba(255, 255, 255, 0));")
        self.back_grad_up.setText("")
        self.back_grad_up.setAlignment(QtCore.Qt.AlignCenter)
        self.back_grad_up.setObjectName("back_grad_up")
        self.Abort_text = QtWidgets.QLabel(self.centralwidget)
        self.Abort_text.setGeometry(QtCore.QRect(610, 740, 251, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(10)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Abort_text.setFont(font)
        self.Abort_text.setStyleSheet("\n"
"color: rgb(255, 255, 255);\n"
"background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 400 10pt \"Inter\";\n"
"")
        self.Abort_text.setAlignment(QtCore.Qt.AlignCenter)
        self.Abort_text.setObjectName("Abort_text")
        self.Data_info_text = QtWidgets.QLabel(self.centralwidget)
        self.Data_info_text.setGeometry(QtCore.QRect(30, 720, 491, 51))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(30)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(7)
        self.Data_info_text.setFont(font)
        self.Data_info_text.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"color: rgb(255, 255, 255);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 63 30pt \"Inter\";")
        self.Data_info_text.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.Data_info_text.setObjectName("Data_info_text")
        self.flight_data_text = QtWidgets.QLabel(self.centralwidget)
        self.flight_data_text.setGeometry(QtCore.QRect(30, 760, 611, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(7)
        self.flight_data_text.setFont(font)
        self.flight_data_text.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"color: rgb(255, 255, 255);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 57 15pt \"Inter\";")
        self.flight_data_text.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.flight_data_text.setObjectName("flight_data_text")
        self.back_grad_down = QtWidgets.QLabel(self.centralwidget)
        self.back_grad_down.setGeometry(QtCore.QRect(0, 600, 1471, 241))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.back_grad_down.setFont(font)
        self.back_grad_down.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.back_grad_down.setStyleSheet("font:  15pt \"Inter\";\n"
"color: rgb(255, 255, 255);\n"
"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(255, 255, 255, 0), stop:1 rgba(0, 0, 0, 150));")
        self.back_grad_down.setText("")
        self.back_grad_down.setAlignment(QtCore.Qt.AlignCenter)
        self.back_grad_down.setObjectName("back_grad_down")
        self.Flight_interface_Time = QtWidgets.QLabel(self.centralwidget)
        self.Flight_interface_Time.setGeometry(QtCore.QRect(1230, 20, 131, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Flight_interface_Time.setFont(font)
        self.Flight_interface_Time.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font:  15pt \"Inter\";")
        self.Flight_interface_Time.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.Flight_interface_Time.setObjectName("Flight_interface_Time")
        self.Flight_interface_unSafty_btn = QtWidgets.QLabel(self.centralwidget)
        self.Flight_interface_unSafty_btn.setGeometry(QtCore.QRect(1252, 381, 17, 21))
        self.Flight_interface_unSafty_btn.setStyleSheet("background-color: rgba(255, 255, 255, 0);\n"
"font:  15pt \"Inter\";")
        self.Flight_interface_unSafty_btn.setText("")
        self.Flight_interface_unSafty_btn.setPixmap(QtGui.QPixmap("img/safty_unlcoked.png"))
        self.Flight_interface_unSafty_btn.setScaledContents(True)
        self.Flight_interface_unSafty_btn.setAlignment(QtCore.Qt.AlignCenter)
        self.Flight_interface_unSafty_btn.setObjectName("Flight_interface_unSafty_btn")
        self.Abort_Box = QtWidgets.QLabel(self.centralwidget)
        self.Abort_Box.setGeometry(QtCore.QRect(670, 775, 130, 51))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Abort_Box.setFont(font)
        self.Abort_Box.setStyleSheet("border-radius :13px;\n"
"background-color: rgb(0, 0, 0);")
        self.Abort_Box.setText("")
        self.Abort_Box.setAlignment(QtCore.Qt.AlignCenter)
        self.Abort_Box.setObjectName("Abort_Box")
        self.parameter1_gauge = QtWidgets.QLabel(self.centralwidget)
        self.parameter1_gauge.setGeometry(QtCore.QRect(1110, 650, 151, 161))
        self.parameter1_gauge.setCursor(QtGui.QCursor(QtCore.Qt.ArrowCursor))
        self.parameter1_gauge.setStyleSheet("background-color: rgba(255, 255, 255, 0);")
        self.parameter1_gauge.setText("")
        self.parameter1_gauge.setPixmap(QtGui.QPixmap(str(parameter1_gauge_img)))
        self.parameter1_gauge.setScaledContents(True)
        self.parameter1_gauge.setAlignment(QtCore.Qt.AlignCenter)
        self.parameter1_gauge.setObjectName("parameter1_gauge")
        self.control_box = QtWidgets.QLabel(self.centralwidget)
        self.control_box.setGeometry(QtCore.QRect(1040, 410, 400, 220))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.control_box.setFont(font)
        self.control_box.setStyleSheet("font:  15pt \"Inter\";\n"
"border-radius :15px;\n"
"background-color: rgb(20,20,20);")
        self.control_box.setText("")
        self.control_box.setAlignment(QtCore.Qt.AlignCenter)
        self.control_box.setObjectName("control_box")
        self.terminal_box = QtWidgets.QLabel(self.centralwidget)
        self.terminal_box.setGeometry(QtCore.QRect(1040, 60, 400, 340))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.terminal_box.setFont(font)
        self.terminal_box.setStyleSheet("border-radius :15px;\n"
"background-color: rgb(20,20,20);")
        self.terminal_box.setText("")
        self.terminal_box.setAlignment(QtCore.Qt.AlignCenter)
        self.terminal_box.setObjectName("terminal_box")
        self.parameter2_box1 = QtWidgets.QLabel(self.centralwidget)
        self.parameter2_box1.setGeometry(QtCore.QRect(1285, 645, 161, 171))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter2_box1.setFont(font)
        self.parameter2_box1.setStyleSheet("border-radius :15px;\n"
"background-color: rgb(20,20,20);")
        self.parameter2_box1.setText("")
        self.parameter2_box1.setAlignment(QtCore.Qt.AlignCenter)
        self.parameter2_box1.setObjectName("parameter2_box1")
        self.parameter2_gauge = QtWidgets.QLabel(self.centralwidget)
        self.parameter2_gauge.setGeometry(QtCore.QRect(1290, 650, 151, 161))
        self.parameter2_gauge.setStyleSheet("background-color: rgba(255, 255, 255, 0);")
        self.parameter2_gauge.setText("")
        self.parameter2_gauge.setPixmap(QtGui.QPixmap(str(parameter2_gauge_img)))
        self.parameter2_gauge.setScaledContents(True)
        self.parameter2_gauge.setAlignment(QtCore.Qt.AlignCenter)
        self.parameter2_gauge.setObjectName("parameter2_gauge")
        self.parameter_interface_box = QtWidgets.QLabel(self.centralwidget)
        self.parameter_interface_box.setGeometry(QtCore.QRect(690, 60, 340, 270))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter_interface_box.setFont(font)
        self.parameter_interface_box.setStyleSheet("border-radius :15px;\n"
"background-color: rgb(20,20,20);")
        self.parameter_interface_box.setText("")
        self.parameter_interface_box.setAlignment(QtCore.Qt.AlignCenter)
        self.parameter_interface_box.setObjectName("parameter_interface_box")
        self.map_main = QtWidgets.QLabel(self.centralwidget)
        self.map_main.setGeometry(QtCore.QRect(20, 60, 660, 270))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.map_main.setFont(font)
        self.map_main.setStyleSheet("border-radius :15px;\n"
"background-color: rgb(230, 230, 230);\n")
        self.map_main.setText("")
        self.map_main.setAlignment(QtCore.Qt.AlignCenter)
        self.map_main.setObjectName("map_main")











        # === 실제 지도(QWebEngineView) 붙이기 ===
        self.map_view = QWebEngineView(self.map_main)
        self.map_view.setObjectName("map_view")
        self.map_view.setGeometry(10, 10, self.map_main.width()-20, self.map_main.height()-20)
        self.map_view.setCursor(QtCore.Qt.PointingHandCursor)

        # 디스크 캐시 활성화 (네트워크 I/O 감소)
        profile = self.map_view.page().profile()
        profile.setHttpCacheType(QWebEngineProfile.DiskHttpCache)
        cache_dir = os.path.join(tempfile.gettempdir(), "flash5_webcache")
        os.makedirs(cache_dir, exist_ok=True)
        profile.setCachePath(cache_dir)
        profile.setHttpCacheMaximumSize(200 * 1024 * 1024)  # 200MB
        profile.setPersistentCookiesPolicy(QWebEngineProfile.ForcePersistentCookies)

        # 지도 HTML 주입 (여기서 '기본 좌표'를 35.152, 128.104로 세팅)
        self._load_leaflet_html()

        # 리사이즈 시 지도 뷰 크기만 가볍게 갱신
        def _on_map_main_resize(e):
            self.map_view.setGeometry(10, 10, self.map_main.width()-20, self.map_main.height()-20)
            QtWidgets.QLabel.resizeEvent(self.map_main, e)
        self.map_main.resizeEvent = _on_map_main_resize

        # JS 호출 스로틀러(텔레메트리 들어올 때 쓸 예정)
        self._map_upd_timer = QTimer(self.map_main)
        self._map_upd_timer.setInterval(75)  # 50~150 사이로 조절
        self._map_upd_timer.timeout.connect(self._flush_map_update)
        self._pending_latlon = None  # (lat, lon, zoom)







        self.parameter2_chart_box = QtWidgets.QLabel(self.centralwidget)
        self.parameter2_chart_box.setGeometry(QtCore.QRect(530, 340, 500, 290))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter2_chart_box.setFont(font)
        self.parameter2_chart_box.setStyleSheet("border-radius :15px;\n"
"background-color: rgb(20,20,20);")
        self.parameter2_chart_box.setText("")
        self.parameter2_chart_box.setAlignment(QtCore.Qt.AlignCenter)
        self.parameter2_chart_box.setObjectName("parameter2_chart_box")
        self.Flight_interface_connect = QtWidgets.QLabel(self.centralwidget)
        self.Flight_interface_connect.setGeometry(QtCore.QRect(1370, 20, 81, 22))
        self.Flight_interface_connect.setStyleSheet("background-color: rgba(255, 255, 255, 0);\n"
"font:  15pt \"Inter\";\n"
"")
        self.Flight_interface_connect.setText("")
        self.Flight_interface_connect.setPixmap(QtGui.QPixmap(str(connecting_img)))
        self.Flight_interface_connect.setScaledContents(True)
        self.Flight_interface_connect.setAlignment(QtCore.Qt.AlignCenter)
        self.Flight_interface_connect.setObjectName("Flight_interface_connect")
        self.alarm_box = QtWidgets.QLabel(self.centralwidget)
        self.alarm_box.setGeometry(QtCore.QRect(1120, 53, 331, 81))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.alarm_box.setFont(font)
        self.alarm_box.setStyleSheet("font:  15pt \"Inter\";\n"
"border-radius :7px;\n"
"border: 1px solid rgb(67, 160, 71);\n"
"background-color: rgb(232,245,233);")
        self.alarm_box.setText("")
        self.alarm_box.setAlignment(QtCore.Qt.AlignCenter)
        self.alarm_box.setObjectName("alarm_box")




        self.confirm_box = QtWidgets.QLabel(self.centralwidget)
        self.confirm_box.setGeometry(QtCore.QRect(0, 0, 1471, 841))
        font = QtGui.QFont()
        font.setFamily("AppleSDGothicNeoSB00")
        font.setPointSize(11)
        self.confirm_box.setFont(font)
        self.confirm_box.setStyleSheet("background-color: rgb(0, 0, 0,200);")
        self.confirm_box.setText("")
        self.confirm_box.setAlignment(QtCore.Qt.AlignCenter)
        self.confirm_box.setObjectName("confirm_box")
        self.confirm_logo = QtWidgets.QLabel(self.centralwidget)
        self.confirm_logo.setGeometry(QtCore.QRect(705, 340, 61, 61))
        self.confirm_logo.setStyleSheet("background-color: rgba(255, 255, 255, 0);")
        self.confirm_logo.setText("")
        sequence_img_path = Path(__file__).parent / "img" / "Sequence.png"
        self.confirm_logo.setPixmap(QtGui.QPixmap(str(sequence_img_path)))
        self.confirm_logo.setScaledContents(True)
        self.confirm_logo.setAlignment(QtCore.Qt.AlignCenter)
        self.confirm_logo.setObjectName("confirm_logo")
        self.confirm_text1 = QtWidgets.QLabel(self.centralwidget)
        self.confirm_text1.setGeometry(QtCore.QRect(605, 410, 261, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(20)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.confirm_text1.setFont(font)
        self.confirm_text1.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"color: rgb(255, 255, 255);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 20pt \"Inter\";")
        self.confirm_text1.setAlignment(QtCore.Qt.AlignCenter)
        self.confirm_text1.setObjectName("confirm_text1")
        
        self.confirm_text2 = QtWidgets.QLabel(self.centralwidget)
        self.confirm_text2.setGeometry(QtCore.QRect(545, 440, 381, 41))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(3)
        self.confirm_text2.setFont(font)
        self.confirm_text2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"color: rgb(255, 255, 255);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 25 15pt \"Inter\";")
        self.confirm_text2.setAlignment(QtCore.Qt.AlignCenter)
        self.confirm_text2.setObjectName("LP_text1")

        self.confirm_btn1 = ClickableLabel(self.centralwidget)
        self.confirm_btn1.setGeometry(QtCore.QRect(695, 500, 81, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.confirm_btn1.setFont(font)
        self.confirm_btn1.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.confirm_btn1.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 15pt \"Inter\";\n"
"color: rgb(28, 115, 255);")
        self.confirm_btn1.setAlignment(QtCore.Qt.AlignCenter)
        self.confirm_btn1.setObjectName("confirm_btn1")
        self.confirm_btn2 = ClickableLabel(self.centralwidget)
        self.confirm_btn2.setGeometry(QtCore.QRect(685, 490, 101, 41))
        font = QtGui.QFont()
        font.setFamily("AppleSDGothicNeoSB00")
        font.setPointSize(11)
        self.confirm_btn2.setFont(font)
        self.confirm_btn2.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.confirm_btn2.setStyleSheet("border-radius :9px;\n"
"background-color: rgb(54, 159, 255,60);")
        self.confirm_btn2.setText("")
        self.confirm_btn2.setAlignment(QtCore.Qt.AlignCenter)
        self.confirm_btn2.setObjectName("confirm_btn2")
        self.confirm_exit_btn = ClickableLabel(self.centralwidget)
        self.confirm_exit_btn.setGeometry(QtCore.QRect(675, 540, 121, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(20)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(3)
        self.confirm_exit_btn.setFont(font)
        self.confirm_exit_btn.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.confirm_exit_btn.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"color: rgb(178, 178, 178);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 25 11pt \"Inter\";")
        self.confirm_exit_btn.setAlignment(QtCore.Qt.AlignCenter)
        self.confirm_exit_btn.setObjectName("confirm_exit_btn")







        self.alarm_title = QtWidgets.QLabel(self.centralwidget)
        self.alarm_title.setGeometry(QtCore.QRect(1160, 63, 131, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(56)
        self.alarm_title.setFont(font)
        self.alarm_title.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"font: 450 12pt \"Inter\";\n"
"color: rgb(69, 69, 69);")
        self.alarm_title.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.alarm_title.setObjectName("alarm_title")
        self.alarm_text = QtWidgets.QLabel(self.centralwidget)
        self.alarm_text.setGeometry(QtCore.QRect(1160, 83, 271, 41))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(10)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(22)
        self.alarm_text.setFont(font)
        self.alarm_text.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"font: 180 10pt \"Inter\";\n"
"color: rgb(68, 68, 68);")
        self.alarm_text.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.alarm_text.setObjectName("alarm_text")
        self.alarm_icon = QtWidgets.QLabel(self.centralwidget)
        self.alarm_icon.setGeometry(QtCore.QRect(1135, 70, 16, 16))
        self.alarm_icon.setCursor(QtGui.QCursor(QtCore.Qt.WaitCursor))
        self.alarm_icon.setStyleSheet("font:  15pt \"Inter\";\n"
"background-color: rgba(255, 255, 255, 0);")
        self.alarm_icon.setText("")
        self.alarm_icon.setPixmap(QtGui.QPixmap(str(info_alarm_icon_img)))
        self.alarm_icon.setScaledContents(True)
        self.alarm_icon.setAlignment(QtCore.Qt.AlignCenter)
        self.alarm_icon.setObjectName("alarm_icon")
        self.alarm_time = QtWidgets.QLabel(self.centralwidget)
        self.alarm_time.setGeometry(QtCore.QRect(1400, 53, 51, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(10)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(43)
        self.alarm_time.setFont(font)
        self.alarm_time.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"font: 350 10pt \"Inter\";\n"
"color: rgb(118, 118, 118);")
        self.alarm_time.setAlignment(QtCore.Qt.AlignCenter)
        self.alarm_time.setObjectName("alarm_time")
        self.control_btn4_3 = ClickableLabel(self.centralwidget)
        self.control_btn4_3.setGeometry(QtCore.QRect(1110, 590, 51, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.control_btn4_3.setFont(font)
        self.control_btn4_3.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.control_btn4_3.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"font:  15pt \"Inter\";\n"
"color: rgb(211, 35, 0);")
        self.control_btn4_3.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.control_btn4_3.setObjectName("control_btn4_3")
        self.control_btn4_2 = ClickableLabel(self.centralwidget)
        self.control_btn4_2.setGeometry(QtCore.QRect(1060, 555, 61, 41))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(10)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(3)
        self.control_btn4_2.setFont(font)
        self.control_btn4_2.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.control_btn4_2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 25 10pt \"Inter\";\n"
"color: rgb(183, 142, 142);")
        self.control_btn4_2.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.control_btn4_2.setObjectName("control_btn4_2")
        self.control_btn1_2 = ClickableLabel(self.centralwidget)
        self.control_btn1_2.setGeometry(QtCore.QRect(1060, 475, 61, 41))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(10)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(3)
        self.control_btn1_2.setFont(font)
        self.control_btn1_2.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.control_btn1_2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(145, 201, 255);\n"
"font: 25 10pt \"Inter\";")
        self.control_btn1_2.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.control_btn1_2.setObjectName("control_btn1_2")
        self.control_btn1_3 = ClickableLabel(self.centralwidget)
        self.control_btn1_3.setGeometry(QtCore.QRect(1110, 510, 51, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.control_btn1_3.setFont(font)
        self.control_btn1_3.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.control_btn1_3.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"font:  15pt \"Inter\";\n"
"color: rgb(28, 115, 255);")
        self.control_btn1_3.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.control_btn1_3.setObjectName("control_btn1_3")
        self.control_btn5_2 = ClickableLabel(self.centralwidget)
        self.control_btn5_2.setGeometry(QtCore.QRect(1220, 551, 71, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(11)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.control_btn5_2.setFont(font)
        self.control_btn5_2.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.control_btn5_2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(28, 115, 255);\n"
"font:  11pt \"Inter\";\n"
"")
        self.control_btn5_2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.control_btn5_2.setObjectName("control_btn5_2")
        self.parameter1_chart_box = QtWidgets.QLabel(self.centralwidget)
        self.parameter1_chart_box.setGeometry(QtCore.QRect(20, 340, 500, 290))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter1_chart_box.setFont(font)
        self.parameter1_chart_box.setStyleSheet("border-radius :15px;\n"
"background-color: rgb(20,20,20);\n"
"")
        self.parameter1_chart_box.setText("")
        self.parameter1_chart_box.setAlignment(QtCore.Qt.AlignCenter)
        self.parameter1_chart_box.setObjectName("parameter1_chart_box")
        self.control_btn7_2 = ClickableLabel(self.centralwidget)
        self.control_btn7_2.setGeometry(QtCore.QRect(1220, 590, 71, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(11)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.control_btn7_2.setFont(font)
        self.control_btn7_2.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.control_btn7_2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(28, 115, 255);\n"
"font:  11pt \"Inter\";\n"
"")
        self.control_btn7_2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.control_btn7_2.setObjectName("control_btn7_2")
        self.control_btn6_2 = ClickableLabel(self.centralwidget)
        self.control_btn6_2.setGeometry(QtCore.QRect(1350, 551, 71, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(11)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.control_btn6_2.setFont(font)
        self.control_btn6_2.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.control_btn6_2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(28, 115, 255);\n"
"font:  11pt \"Inter\";\n"
"")
        self.control_btn6_2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.control_btn6_2.setObjectName("control_btn6_2")

        self.parameter1_chart_main = pg.PlotWidget(MainWindow) 
        self.parameter1_chart_main.setBackground((20, 20, 20))
        self.parameter1_chart_main.showGrid(x=True, y=False, alpha=0.3)
        self.parameter1_chart_main.setGeometry(QtCore.QRect(30, 380, 481, 241))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter1_chart_main.setFont(font)
        self.parameter1_chart_main.setStyleSheet("background-color: rgb(31, 31, 31);")
        self.parameter1_chart_main.setAlignment(QtCore.Qt.AlignCenter)
        self.parameter1_chart_main.setObjectName("parameter1_chart_main")

        self.parameter2_chart_main = pg.PlotWidget(MainWindow) 
        self.parameter2_chart_main.setBackground((20, 20, 20))
        self.parameter2_chart_main.showGrid(x=True, y=False, alpha=0.3)
        self.parameter2_chart_main.setGeometry(QtCore.QRect(540, 380, 481, 241))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter2_chart_main.setFont(font)
        self.parameter2_chart_main.setStyleSheet("background-color: rgb(31, 31, 31);")
        self.parameter2_chart_main.setAlignment(QtCore.Qt.AlignCenter)
        self.parameter2_chart_main.setObjectName("parameter2_chart_main")
        self.control_btn1_1 = ClickableLabel(self.centralwidget)
        self.control_btn1_1.setGeometry(QtCore.QRect(1050, 470, 121, 71))
        self.control_btn1_1.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.control_btn1_1.setStyleSheet("font:  15pt \"Inter\";\n"
"background-color: rgba(255, 255, 255, 0);")
        self.control_btn1_1.setText("")
        self.control_btn1_1.setPixmap(QtGui.QPixmap(str(blue_btn_img)))
        self.control_btn1_1.setScaledContents(True)
        self.control_btn1_1.setAlignment(QtCore.Qt.AlignCenter)
        self.control_btn1_1.setObjectName("control_btn1_1")
        self.control_btn4_1 = ClickableLabel(self.centralwidget)
        self.control_btn4_1.setGeometry(QtCore.QRect(1050, 550, 121, 71))
        self.control_btn4_1.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.control_btn4_1.setStyleSheet("font:  15pt \"Inter\";\n"
"background-color: rgba(255, 255, 255, 0);")
        self.control_btn4_1.setText("")
        self.control_btn4_1.setPixmap(QtGui.QPixmap(str(red_btn_img)))
        self.control_btn4_1.setScaledContents(True)
        self.control_btn4_1.setAlignment(QtCore.Qt.AlignCenter)
        self.control_btn4_1.setObjectName("control_btn4_1")
        self.control_btn2_1 = ClickableLabel(self.centralwidget)
        self.control_btn2_1.setGeometry(QtCore.QRect(1180, 470, 121, 71))
        self.control_btn2_1.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.control_btn2_1.setStyleSheet("font:  15pt \"Inter\";\n"
"background-color: rgba(255, 255, 255, 0);")
        self.control_btn2_1.setText("")
        self.control_btn2_1.setPixmap(QtGui.QPixmap(str(red_btn_img)))
        self.control_btn2_1.setScaledContents(True)
        self.control_btn2_1.setAlignment(QtCore.Qt.AlignCenter)
        self.control_btn2_1.setObjectName("control_btn2_1")
        self.control_btn2_2 = ClickableLabel(self.centralwidget)
        self.control_btn2_2.setGeometry(QtCore.QRect(1190, 475, 61, 41))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(10)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(3)
        self.control_btn2_2.setFont(font)
        self.control_btn2_2.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.control_btn2_2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 25 10pt \"Inter\";\n"
"color: rgb(183, 142, 142);")
        self.control_btn2_2.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.control_btn2_2.setObjectName("control_btn2_2")
        self.control_btn2_3 = ClickableLabel(self.centralwidget)
        self.control_btn2_3.setGeometry(QtCore.QRect(1240, 510, 51, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.control_btn2_3.setFont(font)
        self.control_btn2_3.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.control_btn2_3.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"font:  15pt \"Inter\";\n"
"color: rgb(211, 35, 0);")
        self.control_btn2_3.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.control_btn2_3.setObjectName("control_btn2_3")
        self.control_btn3_2 = ClickableLabel(self.centralwidget)
        self.control_btn3_2.setGeometry(QtCore.QRect(1320, 475, 61, 41))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(10)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(3)
        self.control_btn3_2.setFont(font)
        self.control_btn3_2.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.control_btn3_2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(145, 201, 255);\n"
"font: 25 10pt \"Inter\";")
        self.control_btn3_2.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.control_btn3_2.setObjectName("control_btn3_2")
        self.control_btn3_3 = ClickableLabel(self.centralwidget)
        self.control_btn3_3.setGeometry(QtCore.QRect(1340, 512, 81, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.control_btn3_3.setFont(font)
        self.control_btn3_3.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.control_btn3_3.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 15pt \"Inter\";\n"
"color: rgb(28, 115, 255);")
        self.control_btn3_3.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.control_btn3_3.setObjectName("control_btn3_3")
        self.control_btn3_1 = ClickableLabel(self.centralwidget)
        self.control_btn3_1.setGeometry(QtCore.QRect(1310, 470, 121, 71))
        self.control_btn3_1.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.control_btn3_1.setStyleSheet("font:  15pt \"Inter\";\n"
"background-color: rgba(255, 255, 255, 0);")
        self.control_btn3_1.setText("")
        self.control_btn3_1.setPixmap(QtGui.QPixmap(str(blue_btn_img)))
        self.control_btn3_1.setScaledContents(True)
        self.control_btn3_1.setAlignment(QtCore.Qt.AlignCenter)
        self.control_btn3_1.setObjectName("control_btn3_1")
        self.setting_btn = QtWidgets.QLabel(self.centralwidget)
        self.setting_btn.setGeometry(QtCore.QRect(1050, 420, 41, 40))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.setting_btn.setFont(font)
        self.setting_btn.setStyleSheet("border-radius :11px;\n"
"background-color: rgb(31, 31, 31);")
        self.setting_btn.setText("")
        self.setting_btn.setAlignment(QtCore.Qt.AlignCenter)
        self.setting_btn.setObjectName("setting_btn")
        self.terminal_main = QtWidgets.QTextBrowser(self.centralwidget)
        self.terminal_main.setGeometry(QtCore.QRect(1050, 70, 381, 320))
        self.terminal_main.setStyleSheet("border-radius :11px;\n"
"background-color: rgb(5, 5, 5);")
        self.terminal_main.setObjectName("terminal_main")
        self.parameter1_box_18 = QtWidgets.QLabel(self.centralwidget)
        self.parameter1_box_18.setGeometry(QtCore.QRect(700, 70, 101, 81))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter1_box_18.setFont(font)
        self.parameter1_box_18.setStyleSheet("border-radius :11px;\n"
"background-color: rgb(31, 31, 31);")
        self.parameter1_box_18.setText("")
        self.parameter1_box_18.setAlignment(QtCore.Qt.AlignCenter)
        self.parameter1_box_18.setObjectName("parameter1_box_18")
        self.parameter1_text2 = QtWidgets.QLabel(self.centralwidget)
        self.parameter1_text2.setGeometry(QtCore.QRect(710, 80, 91, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter1_text2.setFont(font)
        self.parameter1_text2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font: 12pt \"Inter\";")
        self.parameter1_text2.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.parameter1_text2.setObjectName("parameter1_text2")
        self.parameter1_main2 = QtWidgets.QLabel(self.centralwidget)
        self.parameter1_main2.setGeometry(QtCore.QRect(700, 100, 91, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(17)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter1_main2.setFont(font)
        self.parameter1_main2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font: 17pt \"Inter\";")
        self.parameter1_main2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.parameter1_main2.setObjectName("parameter1_main2")
        self.parameter1_unit2 = QtWidgets.QLabel(self.centralwidget)
        self.parameter1_unit2.setGeometry(QtCore.QRect(739, 120, 51, 20))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(11)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter1_unit2.setFont(font)
        self.parameter1_unit2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font: 11pt \"Inter\";")
        self.parameter1_unit2.setAlignment(QtCore.Qt.AlignBottom|QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing)
        self.parameter1_unit2.setObjectName("parameter1_unit2")
        self.Latitude_main = QtWidgets.QLabel(self.centralwidget)
        self.Latitude_main.setGeometry(QtCore.QRect(710, 190, 131, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Latitude_main.setFont(font)
        self.Latitude_main.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font:  12pt \"Inter\";\n"
"")
        self.Latitude_main.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.Latitude_main.setObjectName("Latitude_main")
        self.Latitude_box = QtWidgets.QLabel(self.centralwidget)
        self.Latitude_box.setGeometry(QtCore.QRect(700, 160, 155, 71))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Latitude_box.setFont(font)
        self.Latitude_box.setStyleSheet("font:  15pt \"Inter\";\n"
"border-radius :11px;\n"
"background-color: rgb(31, 31, 31);")
        self.Latitude_box.setText("")
        self.Latitude_box.setAlignment(QtCore.Qt.AlignCenter)
        self.Latitude_box.setObjectName("Latitude_box")
        self.Latitude_text = QtWidgets.QLabel(self.centralwidget)
        self.Latitude_text.setGeometry(QtCore.QRect(710, 170, 91, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Latitude_text.setFont(font)
        self.Latitude_text.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font:  12pt \"Inter\";\n"
"")
        self.Latitude_text.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.Latitude_text.setObjectName("Latitude_text")
        self.parameter2_main2 = QtWidgets.QLabel(self.centralwidget)
        self.parameter2_main2.setGeometry(QtCore.QRect(810, 100, 91, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(17)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter2_main2.setFont(font)
        self.parameter2_main2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font: 17pt \"Inter\";")
        self.parameter2_main2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.parameter2_main2.setObjectName("parameter2_main2")
        self.parameter2_unit2 = QtWidgets.QLabel(self.centralwidget)
        self.parameter2_unit2.setGeometry(QtCore.QRect(849, 120, 51, 20))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(11)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter2_unit2.setFont(font)
        self.parameter2_unit2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font: 11pt \"Inter\";")
        self.parameter2_unit2.setAlignment(QtCore.Qt.AlignBottom|QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing)
        self.parameter2_unit2.setObjectName("parameter2_unit2")
        self.parameter1_box_19 = QtWidgets.QLabel(self.centralwidget)
        self.parameter1_box_19.setGeometry(QtCore.QRect(810, 70, 101, 81))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter1_box_19.setFont(font)
        self.parameter1_box_19.setStyleSheet("border-radius :11px;\n"
"background-color: rgb(31, 31, 31);")
        self.parameter1_box_19.setText("")
        self.parameter1_box_19.setAlignment(QtCore.Qt.AlignCenter)
        self.parameter1_box_19.setObjectName("parameter1_box_19")
        self.parameter2_text2 = QtWidgets.QLabel(self.centralwidget)
        self.parameter2_text2.setGeometry(QtCore.QRect(820, 80, 91, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter2_text2.setFont(font)
        self.parameter2_text2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font: 12pt \"Inter\";")
        self.parameter2_text2.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.parameter2_text2.setObjectName("parameter2_text2")
        self.parameter3_unit = QtWidgets.QLabel(self.centralwidget)
        self.parameter3_unit.setGeometry(QtCore.QRect(959, 120, 51, 20))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(11)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter3_unit.setFont(font)
        self.parameter3_unit.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font: 11pt \"Inter\";")
        self.parameter3_unit.setAlignment(QtCore.Qt.AlignBottom|QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing)
        self.parameter3_unit.setObjectName("parameter3_unit")
        self.parameter3_text = QtWidgets.QLabel(self.centralwidget)
        self.parameter3_text.setGeometry(QtCore.QRect(930, 80, 91, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter3_text.setFont(font)
        self.parameter3_text.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font: 12pt \"Inter\";")
        self.parameter3_text.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.parameter3_text.setObjectName("parameter3_text")
        self.parameter3_main = QtWidgets.QLabel(self.centralwidget)
        self.parameter3_main.setGeometry(QtCore.QRect(920, 100, 91, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(17)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter3_main.setFont(font)
        self.parameter3_main.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font: 17pt \"Inter\";")
        self.parameter3_main.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.parameter3_main.setObjectName("parameter3_main")
        self.parameter1_box_23 = QtWidgets.QLabel(self.centralwidget)
        self.parameter1_box_23.setGeometry(QtCore.QRect(920, 70, 101, 81))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter1_box_23.setFont(font)
        self.parameter1_box_23.setStyleSheet("border-radius :11px;\n"
"background-color: rgb(31, 31, 31);")
        self.parameter1_box_23.setText("")
        self.parameter1_box_23.setAlignment(QtCore.Qt.AlignCenter)
        self.parameter1_box_23.setObjectName("parameter1_box_23")
        self.Longitude_box = QtWidgets.QLabel(self.centralwidget)
        self.Longitude_box.setGeometry(QtCore.QRect(865, 160, 155, 71))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Longitude_box.setFont(font)
        self.Longitude_box.setStyleSheet("font:  15pt \"Inter\";\n"
"border-radius :11px;\n"
"background-color: rgb(31, 31, 31);")
        self.Longitude_box.setText("")
        self.Longitude_box.setAlignment(QtCore.Qt.AlignCenter)
        self.Longitude_box.setObjectName("Longitude_box")
        self.Longitude_text = QtWidgets.QLabel(self.centralwidget)
        self.Longitude_text.setGeometry(QtCore.QRect(875, 170, 91, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Longitude_text.setFont(font)
        self.Longitude_text.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font:  12pt \"Inter\";\n"
"")
        self.Longitude_text.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.Longitude_text.setObjectName("Longitude_text")
        self.Longitude_main = QtWidgets.QLabel(self.centralwidget)
        self.Longitude_main.setGeometry(QtCore.QRect(875, 190, 131, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Longitude_main.setFont(font)
        self.Longitude_main.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font:  12pt \"Inter\";\n"
"")
        self.Longitude_main.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.Longitude_main.setObjectName("Longitude_main")
        self.parameter1_chart_text = QtWidgets.QLabel(self.centralwidget)
        self.parameter1_chart_text.setGeometry(QtCore.QRect(40, 350, 91, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter1_chart_text.setFont(font)
        self.parameter1_chart_text.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font: 12pt \"Inter\";")
        self.parameter1_chart_text.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.parameter1_chart_text.setObjectName("parameter1_chart_text")
        self.parameter2_chart_text = QtWidgets.QLabel(self.centralwidget)
        self.parameter2_chart_text.setGeometry(QtCore.QRect(550, 350, 91, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter2_chart_text.setFont(font)
        self.parameter2_chart_text.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font:  15pt \"Inter\";\n"
"")
        self.parameter2_chart_text.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.parameter2_chart_text.setObjectName("parameter2_chart_text")
        self.Gyro_yaw_main = QtWidgets.QLabel(self.centralwidget)
        self.Gyro_yaw_main.setGeometry(QtCore.QRect(921, 270, 91, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(17)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Gyro_yaw_main.setFont(font)
        self.Gyro_yaw_main.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font:  17pt \"Inter\";\n"
"")
        self.Gyro_yaw_main.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.Gyro_yaw_main.setObjectName("Gyro_yaw_main")
        self.Gyro_yaw_text = QtWidgets.QLabel(self.centralwidget)
        self.Gyro_yaw_text.setGeometry(QtCore.QRect(931, 250, 91, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Gyro_yaw_text.setFont(font)
        self.Gyro_yaw_text.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font:  12pt \"Inter\";\n"
"")
        self.Gyro_yaw_text.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.Gyro_yaw_text.setObjectName("Gyro_yaw_text")
        self.Gyro_yaw_box = QtWidgets.QLabel(self.centralwidget)
        self.Gyro_yaw_box.setGeometry(QtCore.QRect(920, 240, 101, 81))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Gyro_yaw_box.setFont(font)
        self.Gyro_yaw_box.setStyleSheet("border-radius :11px;\n"
"background-color: rgb(31, 31, 31);\n"
"font:  15pt \"Inter\";\n"
"")
        self.Gyro_yaw_box.setText("")
        self.Gyro_yaw_box.setAlignment(QtCore.Qt.AlignCenter)
        self.Gyro_yaw_box.setObjectName("Gyro_yaw_box")
        self.Gyro_roll_main = QtWidgets.QLabel(self.centralwidget)
        self.Gyro_roll_main.setGeometry(QtCore.QRect(701, 270, 91, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(17)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Gyro_roll_main.setFont(font)
        self.Gyro_roll_main.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font:  17pt \"Inter\";\n"
"")
        self.Gyro_roll_main.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.Gyro_roll_main.setObjectName("Gyro_roll_main")
        self.Gyro_roll_unit = QtWidgets.QLabel(self.centralwidget)
        self.Gyro_roll_unit.setGeometry(QtCore.QRect(740, 290, 51, 20))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(11)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Gyro_roll_unit.setFont(font)
        self.Gyro_roll_unit.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font:  11pt \"Inter\";\n"
"")
        self.Gyro_roll_unit.setAlignment(QtCore.Qt.AlignBottom|QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing)
        self.Gyro_roll_unit.setObjectName("Gyro_roll_unit")
        self.Gyro_pitch_unit = QtWidgets.QLabel(self.centralwidget)
        self.Gyro_pitch_unit.setGeometry(QtCore.QRect(850, 290, 51, 20))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(11)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Gyro_pitch_unit.setFont(font)
        self.Gyro_pitch_unit.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font:  11pt \"Inter\";\n"
"")
        self.Gyro_pitch_unit.setAlignment(QtCore.Qt.AlignBottom|QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing)
        self.Gyro_pitch_unit.setObjectName("Gyro_pitch_unit")
        self.Gyro_pitch_text = QtWidgets.QLabel(self.centralwidget)
        self.Gyro_pitch_text.setGeometry(QtCore.QRect(821, 250, 91, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Gyro_pitch_text.setFont(font)
        self.Gyro_pitch_text.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font:  12pt \"Inter\";\n"
"")
        self.Gyro_pitch_text.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.Gyro_pitch_text.setObjectName("Gyro_pitch_text")
        self.Gyro_roll_box = QtWidgets.QLabel(self.centralwidget)
        self.Gyro_roll_box.setGeometry(QtCore.QRect(700, 240, 101, 81))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Gyro_roll_box.setFont(font)
        self.Gyro_roll_box.setStyleSheet("border-radius :11px;\n"
"background-color: rgb(31, 31, 31);\n"
"font:  15pt \"Inter\";\n"
"")
        self.Gyro_roll_box.setText("")
        self.Gyro_roll_box.setAlignment(QtCore.Qt.AlignCenter)
        self.Gyro_roll_box.setObjectName("Gyro_roll_box")
        self.Gyro_pitch_main = QtWidgets.QLabel(self.centralwidget)
        self.Gyro_pitch_main.setGeometry(QtCore.QRect(811, 270, 91, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(17)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Gyro_pitch_main.setFont(font)
        self.Gyro_pitch_main.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font:  17pt \"Inter\";\n"
"")
        self.Gyro_pitch_main.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.Gyro_pitch_main.setObjectName("Gyro_pitch_main")
        self.Gyro_pitch_box = QtWidgets.QLabel(self.centralwidget)
        self.Gyro_pitch_box.setGeometry(QtCore.QRect(810, 240, 101, 81))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Gyro_pitch_box.setFont(font)
        self.Gyro_pitch_box.setStyleSheet("border-radius :11px;\n"
"background-color: rgb(31, 31, 31);\n"
"font:  15pt \"Inter\";\n"
"")
        self.Gyro_pitch_box.setText("")
        self.Gyro_pitch_box.setAlignment(QtCore.Qt.AlignCenter)
        self.Gyro_pitch_box.setObjectName("Gyro_pitch_box")
        self.Gyro_roll_text = QtWidgets.QLabel(self.centralwidget)
        self.Gyro_roll_text.setGeometry(QtCore.QRect(711, 250, 91, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Gyro_roll_text.setFont(font)
        self.Gyro_roll_text.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font:  12pt \"Inter\";\n"
"")
        self.Gyro_roll_text.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.Gyro_roll_text.setObjectName("Gyro_roll_text")
        self.Gyro_yaw_unit = QtWidgets.QLabel(self.centralwidget)
        self.Gyro_yaw_unit.setGeometry(QtCore.QRect(960, 290, 51, 20))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(11)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Gyro_yaw_unit.setFont(font)
        self.Gyro_yaw_unit.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(167, 167, 167);\n"
"font:  11pt \"Inter\";\n"
"")
        self.Gyro_yaw_unit.setAlignment(QtCore.Qt.AlignBottom|QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing)
        self.Gyro_yaw_unit.setObjectName("Gyro_yaw_unit")
        self.parameter2_unit1 = QtWidgets.QLabel(self.centralwidget)
        self.parameter2_unit1.setGeometry(QtCore.QRect(1300, 750, 131, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(20)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter2_unit1.setFont(font)
        self.parameter2_unit1.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"font: 20pt \"Inter\";\n"
"color: rgb(255, 255, 255);")
        self.parameter2_unit1.setAlignment(QtCore.Qt.AlignCenter)
        self.parameter2_unit1.setObjectName("parameter2_unit1")
        self.parameter1_text1 = QtWidgets.QLabel(self.centralwidget)
        self.parameter1_text1.setGeometry(QtCore.QRect(1120, 680, 131, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(7)
        self.parameter1_text1.setFont(font)
        self.parameter1_text1.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"color: rgb(255, 255, 255);\n"
"font: 63 15pt \"Inter\";")
        self.parameter1_text1.setAlignment(QtCore.Qt.AlignCenter)
        self.parameter1_text1.setObjectName("parameter1_text1")
        self.parameter1_unit1 = QtWidgets.QLabel(self.centralwidget)
        self.parameter1_unit1.setGeometry(QtCore.QRect(1120, 750, 131, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(20)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter1_unit1.setFont(font)
        self.parameter1_unit1.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"font: 20pt \"Inter\";\n"
"color: rgb(255, 255, 255);")
        self.parameter1_unit1.setAlignment(QtCore.Qt.AlignCenter)
        self.parameter1_unit1.setObjectName("parameter1_unit1")
        self.parameter2_main1 = QtWidgets.QLabel(self.centralwidget)
        self.parameter2_main1.setGeometry(QtCore.QRect(1300, 710, 131, 41))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(30)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter2_main1.setFont(font)
        self.parameter2_main1.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"color: rgb(255, 255, 255);\n"
"font: 30pt \"Inter\";")
        self.parameter2_main1.setAlignment(QtCore.Qt.AlignCenter)
        self.parameter2_main1.setObjectName("parameter2_main1")
        self.parameter2_text1 = QtWidgets.QLabel(self.centralwidget)
        self.parameter2_text1.setGeometry(QtCore.QRect(1300, 680, 131, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(7)
        self.parameter2_text1.setFont(font)
        self.parameter2_text1.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"color: rgb(255, 255, 255);\n"
"font: 63 15pt \"Inter\";")
        self.parameter2_text1.setAlignment(QtCore.Qt.AlignCenter)
        self.parameter2_text1.setObjectName("parameter2_text1")
        self.parameter1_main1 = QtWidgets.QLabel(self.centralwidget)
        self.parameter1_main1.setGeometry(QtCore.QRect(1120, 710, 131, 41))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(30)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.parameter1_main1.setFont(font)
        self.parameter1_main1.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"color: rgb(255, 255, 255);\n"
"font: 30pt \"Inter\";")
        self.parameter1_main1.setAlignment(QtCore.Qt.AlignCenter)
        self.parameter1_main1.setObjectName("parameter1_main1")
        self.Abort_btn = QtWidgets.QPushButton(self.centralwidget)
        self.Abort_btn.setGeometry(QtCore.QRect(675, 780, 121, 41))
        self.Abort_btn.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.Abort_btn.setStyleSheet("border-radius :9px;\n"
"background-color: rgb(255, 54, 54,60);\n"
"font:  15pt \"Inter\";\n"
"color: rgb(211, 35, 0);")
        self.Abort_btn.setObjectName("Abort_btn")
        self.Map_plus_btn = QtWidgets.QPushButton(self.centralwidget)
        self.Map_plus_btn.setGeometry(QtCore.QRect(560, 280, 101, 31))
        self.Map_plus_btn.setStyleSheet("border-radius :9px;\n"
"background-color: rgb(46, 46, 46);\n"
"font: 305 10pt \"Inter\";\n"
"color: rgb(205, 227, 255);")
        self.Map_plus_btn.setObjectName("Map_plus_btn")
        self.control_btn5_1 = ClickableLabel(self.centralwidget)
        self.control_btn5_1.setGeometry(QtCore.QRect(1180, 550, 121, 31))
        self.control_btn5_1.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.control_btn5_1.setStyleSheet("font:  15pt \"Inter\";\n"
"background-color: rgba(255, 255, 255, 0);")
        self.control_btn5_1.setText("")
        self.control_btn5_1.setPixmap(QtGui.QPixmap(str(blue_btn_mini_img)))
        self.control_btn5_1.setScaledContents(True)
        self.control_btn5_1.setAlignment(QtCore.Qt.AlignCenter)
        self.control_btn5_1.setObjectName("control_btn5_1")
        self.control_btn7_1 = ClickableLabel(self.centralwidget)
        self.control_btn7_1.setGeometry(QtCore.QRect(1180, 590, 121, 31))
        self.control_btn7_1.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.control_btn7_1.setStyleSheet("font:  15pt \"Inter\";\n"
"background-color: rgba(255, 255, 255, 0);")
        self.control_btn7_1.setText("")
        self.control_btn7_1.setPixmap(QtGui.QPixmap(str(blue_btn_mini_img)))
        self.control_btn7_1.setScaledContents(True)
        self.control_btn7_1.setAlignment(QtCore.Qt.AlignCenter)
        self.control_btn7_1.setObjectName("control_btn7_1")
        self.control_btn6_1 = ClickableLabel(self.centralwidget)
        self.control_btn6_1.setGeometry(QtCore.QRect(1310, 550, 121, 31))
        self.control_btn6_1.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.control_btn6_1.setStyleSheet("font:  15pt \"Inter\";\n"
"background-color: rgba(255, 255, 255, 0);")
        self.control_btn6_1.setText("")
        self.control_btn6_1.setPixmap(QtGui.QPixmap(str(blue_btn_mini_img)))
        self.control_btn6_1.setScaledContents(True)
        self.control_btn6_1.setAlignment(QtCore.Qt.AlignCenter)
        self.control_btn6_1.setObjectName("control_btn6_1")
        self.intro_box = QtWidgets.QLabel(self.centralwidget)
        self.intro_box.setGeometry(QtCore.QRect(0, 0, 1471, 841))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.intro_box.setFont(font)
        self.intro_box.setStyleSheet("background-color: rgb(31, 31, 31);")
        self.intro_box.setText("")
        self.intro_box.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.intro_box.setObjectName("intro_box")
        self.intro_logo = QtWidgets.QLabel(self.centralwidget)
        self.intro_logo.setGeometry(QtCore.QRect(690, 370, 91, 91))
        self.intro_logo.setStyleSheet("background-color: rgba(255, 255, 255, 0);")
        self.intro_logo.setText("")
        self.intro_logo.setPixmap(QtGui.QPixmap(str(intro_logo_img)))
        self.intro_logo.setScaledContents(True)
        self.intro_logo.setAlignment(QtCore.Qt.AlignJustify|QtCore.Qt.AlignVCenter)
        self.intro_logo.setObjectName("intro_logo")
        self.intro_text1 = QtWidgets.QLabel(self.centralwidget)
        self.intro_text1.setGeometry(QtCore.QRect(10, 810, 211, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(10)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.intro_text1.setFont(font)
        self.intro_text1.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"font: 400 10pt \"Inter\";\n"
"color: rgb(77, 77, 77);")
        self.intro_text1.setFrameShadow(QtWidgets.QFrame.Plain)
        self.intro_text1.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.intro_text1.setObjectName("intro_text1")
        self.intro_text2 = QtWidgets.QLabel(self.centralwidget)
        self.intro_text2.setGeometry(QtCore.QRect(1300, 810, 161, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(10)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(12)
        self.intro_text2.setFont(font)
        self.intro_text2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"font: 100 10pt \"Inter\";\n"
"color: rgb(77, 77, 77);")
        self.intro_text2.setFrameShadow(QtWidgets.QFrame.Plain)
        self.intro_text2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.intro_text2.setObjectName("intro_text2")
        self.control_next_btn = QtWidgets.QPushButton(self.centralwidget)
        self.control_next_btn.setGeometry(QtCore.QRect(1310, 590, 121, 31))
        self.control_next_btn.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.control_next_btn.setStyleSheet("border-radius :9px;\n"
"background-color: rgb(33, 33, 33);\n"
"font: 11pt \"Inter\";\n"
"color: rgb(151, 151, 151);")
        self.control_next_btn.setObjectName("control_next_btn")
        self.safty_btn1 = QtWidgets.QLabel(self.centralwidget)
        self.safty_btn1.setGeometry(QtCore.QRect(71, 288, 121, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.safty_btn1.setFont(font)
        self.safty_btn1.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.safty_btn1.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(172, 172, 172);\n"
"font: 400 12pt \"Inter\";")
        self.safty_btn1.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.safty_btn1.setObjectName("safty_btn1")
        self.set_a_spin1 = QtWidgets.QSpinBox(self.centralwidget)
        self.set_a_spin1.setGeometry(QtCore.QRect(917, 180, 41, 21))
        self.set_a_spin1.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"color: rgb(255, 255, 255);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 63 10pt \"Inter\";")
        self.set_a_spin1.setObjectName("set_a_spin1")
        self.settings_text3_1 = QtWidgets.QLabel(self.centralwidget)
        self.settings_text3_1.setGeometry(QtCore.QRect(240, 520, 381, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.settings_text3_1.setFont(font)
        self.settings_text3_1.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 400 15pt \"Inter\";\n"
"color: rgb(208, 208, 208);")
        self.settings_text3_1.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.settings_text3_1.setObjectName("settings_text3_1")
        self.settings2_img_box = QtWidgets.QLabel(self.centralwidget)
        self.settings2_img_box.setGeometry(QtCore.QRect(240, 352, 391, 131))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.settings2_img_box.setFont(font)
        self.settings2_img_box.setStyleSheet("border-radius :10px;\n"
"background-color: rgb(31, 31, 31);")
        self.settings2_img_box.setText("")
        self.settings2_img_box.setAlignment(QtCore.Qt.AlignCenter)
        self.settings2_img_box.setObjectName("settings2_img_box")
        self.Set_title = QtWidgets.QLabel(self.centralwidget)
        self.Set_title.setGeometry(QtCore.QRect(240, 90, 401, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(25)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.Set_title.setFont(font)
        self.Set_title.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"color: rgb(255, 255, 255);\n"
"border-color: rgb(0, 0, 0);\n"
"font:  25pt \"Inter\";")
        self.Set_title.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.Set_title.setObjectName("Set_title")
        self.settings_exit_btn = QtWidgets.QLabel(self.centralwidget)
        self.settings_exit_btn.setGeometry(QtCore.QRect(41, 68, 111, 41))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(14)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(25)
        self.settings_exit_btn.setFont(font)
        self.settings_exit_btn.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.settings_exit_btn.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"color: rgb(255, 255, 255);\n"
"border-color: rgb(0, 0, 0);\n"
"font:200 14pt \"Inter\";")
        self.settings_exit_btn.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.settings_exit_btn.setObjectName("settings_exit_btn")
        self.settings_btn2_2 = QtWidgets.QLabel(self.centralwidget)
        self.settings_btn2_2.setGeometry(QtCore.QRect(240, 320, 31, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(8)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.settings_btn2_2.setFont(font)
        self.settings_btn2_2.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.settings_btn2_2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"color: rgb(255, 255, 255);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 400 8pt \"Inter\";")
        self.settings_btn2_2.setAlignment(QtCore.Qt.AlignCenter)
        self.settings_btn2_2.setObjectName("settings_btn2_2")
        self.sequence_btn1 = QtWidgets.QLabel(self.centralwidget)
        self.sequence_btn1.setGeometry(QtCore.QRect(71, 248, 121, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.sequence_btn1.setFont(font)
        self.sequence_btn1.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.sequence_btn1.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(172, 172, 172);\n"
"font: 400 12pt \"Inter\";")
        self.sequence_btn1.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.sequence_btn1.setObjectName("sequence_btn1")
        self.programinfo_btn1 = QtWidgets.QLabel(self.centralwidget)
        self.programinfo_btn1.setGeometry(QtCore.QRect(71, 328, 121, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.programinfo_btn1.setFont(font)
        self.programinfo_btn1.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.programinfo_btn1.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(172, 172, 172);\n"
"font: 400 12pt \"Inter\";")
        self.programinfo_btn1.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.programinfo_btn1.setObjectName("programinfo_btn1")
        self.interface_btn2 = QtWidgets.QLabel(self.centralwidget)
        self.interface_btn2.setGeometry(QtCore.QRect(41, 128, 20, 20))
        self.interface_btn2.setStyleSheet("background-color: rgba(255, 255, 255, 0);")
        self.interface_btn2.setText("")
        self.interface_btn2.setPixmap(QtGui.QPixmap("../Flash_V4/img/interface_icon.png"))
        self.interface_btn2.setScaledContents(True)
        self.interface_btn2.setAlignment(QtCore.Qt.AlignCenter)
        self.interface_btn2.setObjectName("interface_btn2")
        self.settings_text2_3 = QtWidgets.QLabel(self.centralwidget)
        self.settings_text2_3.setGeometry(QtCore.QRect(240, 482, 391, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(8)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(7)
        self.settings_text2_3.setFont(font)
        self.settings_text2_3.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"color: rgb(255, 255, 255);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 63 8pt \"Inter\";")
        self.settings_text2_3.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.settings_text2_3.setObjectName("settings_text2_3")
        self.settings_btn1_1 = QtWidgets.QLabel(self.centralwidget)
        self.settings_btn1_1.setGeometry(QtCore.QRect(240, 230, 51, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.settings_btn1_1.setFont(font)
        self.settings_btn1_1.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.settings_btn1_1.setStyleSheet("border-radius :10px;\n"
"background-color: rgb(145, 145, 145);")
        self.settings_btn1_1.setText("")
        self.settings_btn1_1.setAlignment(QtCore.Qt.AlignCenter)
        self.settings_btn1_1.setObjectName("settings_btn1_1")
        self.settings_btn3_b = QtWidgets.QCheckBox(self.centralwidget)
        self.settings_btn3_b.setGeometry(QtCore.QRect(320, 570, 51, 20))
        self.settings_btn3_b.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.settings_btn3_b.setStyleSheet("background-color: rgba(255, 255, 255, 0);\n"
"color: rgb(255, 255, 255);\n"
"font: 300 10pt \"Inter\";")
        self.settings_btn3_b.setChecked(False)
        self.settings_btn3_b.setAutoRepeat(False)
        self.settings_btn3_b.setTristate(False)
        self.settings_btn3_b.setObjectName("settings_btn3_b")
        self.settings_box1 = QtWidgets.QLabel(self.centralwidget)
        self.settings_box1.setGeometry(QtCore.QRect(20, 60, 1011, 571))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.settings_box1.setFont(font)
        self.settings_box1.setCursor(QtGui.QCursor(QtCore.Qt.ArrowCursor))
        self.settings_box1.setStyleSheet("border-radius :15px;\n"
"background-color: rgb(20,20,20);")
        self.settings_box1.setText("")
        self.settings_box1.setAlignment(QtCore.Qt.AlignCenter)
        self.settings_box1.setObjectName("settings_box1")
        self.settings_btn3_1 = QtWidgets.QLabel(self.centralwidget)
        self.settings_btn3_1.setGeometry(QtCore.QRect(460, 570, 51, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.settings_btn3_1.setFont(font)
        self.settings_btn3_1.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.settings_btn3_1.setStyleSheet("border-radius :10px;\n"
"background-color: rgb(50,205,50);")
        self.settings_btn3_1.setText("")
        self.settings_btn3_1.setAlignment(QtCore.Qt.AlignCenter)
        self.settings_btn3_1.setObjectName("settings_btn3_1")
        self.settings_box2 = QtWidgets.QLabel(self.centralwidget)
        self.settings_box2.setGeometry(QtCore.QRect(20, 60, 181, 571))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.settings_box2.setFont(font)
        self.settings_box2.setStyleSheet("border-radius :15px;\n"
"background-color: rgb(11, 11, 11);\n"
"")
        self.settings_box2.setText("")
        self.settings_box2.setAlignment(QtCore.Qt.AlignCenter)
        self.settings_box2.setObjectName("settings_box2")
        self.settings_text2_2 = QtWidgets.QLabel(self.centralwidget)
        self.settings_text2_2.setGeometry(QtCore.QRect(240, 290, 401, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(10)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(7)
        self.settings_text2_2.setFont(font)
        self.settings_text2_2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"color: rgb(255, 255, 255);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 63 10pt \"Inter\";")
        self.settings_text2_2.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.settings_text2_2.setObjectName("settings_text2_2")
        self.set_desc = QtWidgets.QLabel(self.centralwidget)
        self.set_desc.setGeometry(QtCore.QRect(240, 120, 401, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(7)
        self.set_desc.setFont(font)
        self.set_desc.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"color: rgb(255, 255, 255);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 63 12pt \"Inter\";")
        self.set_desc.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.set_desc.setObjectName("set_desc")
        self.settings_guideline = QtWidgets.QLabel(self.centralwidget)
        self.settings_guideline.setGeometry(QtCore.QRect(240, 160, 631, 1))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.settings_guideline.setFont(font)
        self.settings_guideline.setStyleSheet("background-color: rgb(89, 89, 89);")
        self.settings_guideline.setText("")
        self.settings_guideline.setAlignment(QtCore.Qt.AlignCenter)
        self.settings_guideline.setObjectName("settings_guideline")
        self.settings_btn1_3 = QtWidgets.QLabel(self.centralwidget)
        self.settings_btn1_3.setGeometry(QtCore.QRect(260, 230, 31, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(8)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.settings_btn1_3.setFont(font)
        self.settings_btn1_3.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.settings_btn1_3.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"color: rgb(255, 255, 255);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 400 8pt \"Inter\";")
        self.settings_btn1_3.setAlignment(QtCore.Qt.AlignCenter)
        self.settings_btn1_3.setObjectName("settings_btn1_3")
        self.advanced_btn1 = QtWidgets.QLabel(self.centralwidget)
        self.advanced_btn1.setGeometry(QtCore.QRect(71, 168, 121, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.advanced_btn1.setFont(font)
        self.advanced_btn1.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.advanced_btn1.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(172, 172, 172);\n"
"font: 400 12pt \"Inter\";")
        self.advanced_btn1.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.advanced_btn1.setObjectName("advanced_btn1")
        self.safty_btn2 = QtWidgets.QLabel(self.centralwidget)
        self.safty_btn2.setGeometry(QtCore.QRect(41, 288, 20, 20))
        self.safty_btn2.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.safty_btn2.setStyleSheet("background-color: rgba(255, 255, 255, 0);")
        self.safty_btn2.setText("")
        self.safty_btn2.setPixmap(QtGui.QPixmap("../Flash_V4/img/safety2.png"))
        self.safty_btn2.setScaledContents(True)
        self.safty_btn2.setAlignment(QtCore.Qt.AlignCenter)
        self.safty_btn2.setObjectName("safty_btn2")
        self.settings_btn2_3 = QtWidgets.QLabel(self.centralwidget)
        self.settings_btn2_3.setGeometry(QtCore.QRect(269, 322, 20, 17))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.settings_btn2_3.setFont(font)
        self.settings_btn2_3.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.settings_btn2_3.setStyleSheet("border-radius :8px;\n"
"background-color: rgb(255, 255, 255);")
        self.settings_btn2_3.setText("")
        self.settings_btn2_3.setAlignment(QtCore.Qt.AlignCenter)
        self.settings_btn2_3.setObjectName("settings_btn2_3")
        self.sequence_btn2 = QtWidgets.QLabel(self.centralwidget)
        self.sequence_btn2.setGeometry(QtCore.QRect(41, 248, 20, 20))
        self.sequence_btn2.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.sequence_btn2.setStyleSheet("background-color: rgba(255, 255, 255, 0);")
        self.sequence_btn2.setText("")
        self.sequence_btn2.setPixmap(QtGui.QPixmap("../Flash_V4/img/sequence_icon.png"))
        self.sequence_btn2.setScaledContents(True)
        self.sequence_btn2.setAlignment(QtCore.Qt.AlignCenter)
        self.sequence_btn2.setObjectName("sequence_btn2")
        self.advanced_btn2 = QtWidgets.QLabel(self.centralwidget)
        self.advanced_btn2.setGeometry(QtCore.QRect(41, 168, 20, 20))
        self.advanced_btn2.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.advanced_btn2.setStyleSheet("background-color: rgba(255, 255, 255, 0);")
        self.advanced_btn2.setText("")
        self.advanced_btn2.setPixmap(QtGui.QPixmap("../Flash_V4/img/Hardware_icon.png"))
        self.advanced_btn2.setScaledContents(True)
        self.advanced_btn2.setAlignment(QtCore.Qt.AlignCenter)
        self.advanced_btn2.setObjectName("advanced_btn2")
        self.settings_btn3_c = QtWidgets.QCheckBox(self.centralwidget)
        self.settings_btn3_c.setGeometry(QtCore.QRect(390, 570, 51, 20))
        self.settings_btn3_c.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.settings_btn3_c.setStyleSheet("background-color: rgba(255, 255, 255, 0);\n"
"color: rgb(255, 255, 255);\n"
"font: 300 10pt \"Inter\";")
        self.settings_btn3_c.setChecked(False)
        self.settings_btn3_c.setAutoRepeat(False)
        self.settings_btn3_c.setTristate(False)
        self.settings_btn3_c.setObjectName("settings_btn3_c")
        self.settings_text3_2 = QtWidgets.QLabel(self.centralwidget)
        self.settings_text3_2.setGeometry(QtCore.QRect(240, 540, 401, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(10)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(7)
        self.settings_text3_2.setFont(font)
        self.settings_text3_2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"color: rgb(255, 255, 255);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 63 10pt \"Inter\";")
        self.settings_text3_2.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.settings_text3_2.setObjectName("settings_text3_2")
        self.programinfo_btn2 = QtWidgets.QLabel(self.centralwidget)
        self.programinfo_btn2.setGeometry(QtCore.QRect(41, 328, 20, 20))
        self.programinfo_btn2.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.programinfo_btn2.setStyleSheet("background-color: rgba(255, 255, 255, 0);")
        self.programinfo_btn2.setText("")
        self.programinfo_btn2.setPixmap(QtGui.QPixmap("../Flash_V4/img/information_icon.png"))
        self.programinfo_btn2.setScaledContents(True)
        self.programinfo_btn2.setAlignment(QtCore.Qt.AlignCenter)
        self.programinfo_btn2.setObjectName("programinfo_btn2")
        self.interface_btn1 = QtWidgets.QLabel(self.centralwidget)
        self.interface_btn1.setGeometry(QtCore.QRect(71, 128, 121, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.interface_btn1.setFont(font)
        self.interface_btn1.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.interface_btn1.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(172, 172, 172);\n"
"font: 400 12pt \"Inter\";")
        self.interface_btn1.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.interface_btn1.setObjectName("interface_btn1")
        self.settings_text1_1 = QtWidgets.QLabel(self.centralwidget)
        self.settings_text1_1.setGeometry(QtCore.QRect(240, 180, 381, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.settings_text1_1.setFont(font)
        self.settings_text1_1.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 400 15pt \"Inter\";\n"
"color: rgb(208, 208, 208);")
        self.settings_text1_1.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.settings_text1_1.setObjectName("settings_text1_1")
        self.data_btn1 = QtWidgets.QLabel(self.centralwidget)
        self.data_btn1.setGeometry(QtCore.QRect(71, 208, 121, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.data_btn1.setFont(font)
        self.data_btn1.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.data_btn1.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(172, 172, 172);\n"
"font: 400 12pt \"Inter\";")
        self.data_btn1.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.data_btn1.setObjectName("data_btn1")
        self.set_a_spinbox1 = QtWidgets.QLabel(self.centralwidget)
        self.set_a_spinbox1.setGeometry(QtCore.QRect(910, 175, 51, 31))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.set_a_spinbox1.setFont(font)
        self.set_a_spinbox1.setStyleSheet("border-radius :10px;\n"
"background-color: rgb(31, 31, 31);")
        self.set_a_spinbox1.setText("")
        self.set_a_spinbox1.setAlignment(QtCore.Qt.AlignCenter)
        self.set_a_spinbox1.setObjectName("set_a_spinbox1")
        self.settings_btn3_a = QtWidgets.QCheckBox(self.centralwidget)
        self.settings_btn3_a.setGeometry(QtCore.QRect(240, 570, 71, 20))
        self.settings_btn3_a.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.settings_btn3_a.setStyleSheet("background-color: rgba(255, 255, 255, 0);\n"
"color: rgb(255, 255, 255);\n"
"selection-color: rgb(100, 255, 80);\n"
"font: 300 10pt \"Inter\";")
        self.settings_btn3_a.setChecked(False)
        self.settings_btn3_a.setAutoRepeat(False)
        self.settings_btn3_a.setTristate(False)
        self.settings_btn3_a.setObjectName("settings_btn3_a")
        self.settings_btn3_2 = QtWidgets.QLabel(self.centralwidget)
        self.settings_btn3_2.setGeometry(QtCore.QRect(470, 570, 31, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(10)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.settings_btn3_2.setFont(font)
        self.settings_btn3_2.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.settings_btn3_2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"color: rgb(255, 255, 255);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 400 10pt \"Inter\";")
        self.settings_btn3_2.setAlignment(QtCore.Qt.AlignCenter)
        self.settings_btn3_2.setObjectName("settings_btn3_2")
        self.data_btn2 = QtWidgets.QLabel(self.centralwidget)
        self.data_btn2.setGeometry(QtCore.QRect(41, 208, 20, 20))
        self.data_btn2.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.data_btn2.setStyleSheet("background-color: rgba(255, 255, 255, 0);")
        self.data_btn2.setText("")
        self.data_btn2.setPixmap(QtGui.QPixmap("../Flash_V4/img/data_icon.png"))
        self.data_btn2.setScaledContents(True)
        self.data_btn2.setAlignment(QtCore.Qt.AlignCenter)
        self.data_btn2.setObjectName("data_btn2")
        self.settings_menubar = QtWidgets.QLabel(self.centralwidget)
        self.settings_menubar.setGeometry(QtCore.QRect(20, 120, 181, 41))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.settings_menubar.setFont(font)
        self.settings_menubar.setStyleSheet("font:  15pt \"Inter\";\n"
"background-color: rgb(25, 25, 25);")
        self.settings_menubar.setText("")
        self.settings_menubar.setAlignment(QtCore.Qt.AlignCenter)
        self.settings_menubar.setObjectName("settings_menubar")
        self.settings_btn1_2 = QtWidgets.QLabel(self.centralwidget)
        self.settings_btn1_2.setGeometry(QtCore.QRect(242, 232, 20, 17))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.settings_btn1_2.setFont(font)
        self.settings_btn1_2.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.settings_btn1_2.setStyleSheet("border-radius :8px;\n"
"background-color: rgb(255, 255, 255);")
        self.settings_btn1_2.setText("")
        self.settings_btn1_2.setAlignment(QtCore.Qt.AlignCenter)
        self.settings_btn1_2.setObjectName("settings_btn1_2")
        self.settings_text1_2 = QtWidgets.QLabel(self.centralwidget)
        self.settings_text1_2.setGeometry(QtCore.QRect(240, 200, 401, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(10)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(7)
        self.settings_text1_2.setFont(font)
        self.settings_text1_2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"color: rgb(255, 255, 255);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 63 10pt \"Inter\";")
        self.settings_text1_2.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.settings_text1_2.setObjectName("settings_text1_2")
        self.settings_btn2_1 = QtWidgets.QLabel(self.centralwidget)
        self.settings_btn2_1.setGeometry(QtCore.QRect(240, 320, 51, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.settings_btn2_1.setFont(font)
        self.settings_btn2_1.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.settings_btn2_1.setStyleSheet("border-radius :10px;\n"
"background-color: rgb(50,205,50);")
        self.settings_btn2_1.setText("")
        self.settings_btn2_1.setAlignment(QtCore.Qt.AlignCenter)
        self.settings_btn2_1.setObjectName("settings_btn2_1")
        self.settings_text2_1 = QtWidgets.QLabel(self.centralwidget)
        self.settings_text2_1.setGeometry(QtCore.QRect(240, 270, 381, 21))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.settings_text2_1.setFont(font)
        self.settings_text2_1.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 400 15pt \"Inter\";\n"
"color: rgb(208, 208, 208);")
        self.settings_text2_1.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.settings_text2_1.setObjectName("settings_text2_1")
        self.settings_box2_2 = QtWidgets.QLabel(self.centralwidget)
        self.settings_box2_2.setGeometry(QtCore.QRect(180, 60, 21, 571))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.settings_box2_2.setFont(font)
        self.settings_box2_2.setStyleSheet("background-color: rgb(11, 11, 11);\n"
"")
        self.settings_box2_2.setText("")
        self.settings_box2_2.setAlignment(QtCore.Qt.AlignCenter)
        self.settings_box2_2.setObjectName("settings_box2_2")
        self.export_btn1 = ClickableLabel(self.centralwidget)
        self.export_btn1.setGeometry(QtCore.QRect(1110, 432, 17, 18))
        self.export_btn1.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.export_btn1.setStyleSheet("background-color: rgba(255, 255, 255, 0);")
        self.export_btn1.setText("")
        self.export_btn1.setPixmap(QtGui.QPixmap(str(export_btn_img)))
        self.export_btn1.setScaledContents(True)
        self.export_btn1.setAlignment(QtCore.Qt.AlignCenter)
        self.export_btn1.setObjectName("export_btn1")
        self.setting_btn1 = ClickableLabel(self.centralwidget)
        self.setting_btn1.setGeometry(QtCore.QRect(1060, 430, 21, 21))
        self.setting_btn1.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.setting_btn1.setStyleSheet("background-color: rgba(255, 255, 255, 0);")
        self.setting_btn1.setText("")
        self.setting_btn1.setPixmap(QtGui.QPixmap(str(setting_btn_img)))
        self.setting_btn1.setScaledContents(True)
        self.setting_btn1.setAlignment(QtCore.Qt.AlignCenter)
        self.setting_btn1.setObjectName("setting_btn1")
        self.export_btn2 = ClickableLabel(self.centralwidget)
        self.export_btn2.setGeometry(QtCore.QRect(1098, 420, 41, 40))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.export_btn2.setFont(font)
        self.export_btn2.setStyleSheet("border-radius :11px;\n"
"background-color: rgb(31, 31, 31);")
        self.export_btn2.setText("")
        self.export_btn2.setAlignment(QtCore.Qt.AlignCenter)
        self.export_btn2.setObjectName("export_btn2")
        self.control_side_box = QtWidgets.QLabel(self.centralwidget)
        self.control_side_box.setGeometry(QtCore.QRect(1145, 420, 285, 40))
        font = QtGui.QFont()
        font.setFamily("Inter")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.control_side_box.setFont(font)
        self.control_side_box.setStyleSheet("border-radius :11px;\n"
"background-color: rgb(31, 31, 31);\n"
"font:  15pt \"Inter\";\n"
"")
        self.control_side_box.setText("")
        self.control_side_box.setAlignment(QtCore.Qt.AlignCenter)
        self.control_side_box.setObjectName("control_side_box")
        self.back_grad_up.raise_()
        self.parameter1_chart_main.raise_()
        self.parameter2_chart_main.raise_()
        self.control_box.raise_()
        self.setting_btn.raise_()
        self.control_btn7_1.raise_()
        self.control_btn6_1.raise_()
        self.control_btn5_1.raise_()
        self.parameter_interface_box.raise_()
        self.Gyro_roll_box.raise_()
        self.Gyro_yaw_box.raise_()
        self.Gyro_pitch_box.raise_()
        self.parameter1_box_23.raise_()
        self.parameter1_box_19.raise_()
        self.Latitude_box.raise_()
        self.control_btn3_1.raise_()
        self.control_btn4_1.raise_()
        self.control_btn1_1.raise_()
        self.terminal_box.raise_()
        self.parameter2_chart_box.raise_()
        self.control_btn4_3.raise_()
        self.control_btn4_2.raise_()
        self.control_btn1_2.raise_()
        self.control_btn1_3.raise_()
        self.control_btn5_2.raise_()
        self.parameter1_chart_box.raise_()
        self.control_btn7_2.raise_()
        self.control_btn6_2.raise_()
        self.control_btn2_1.raise_()
        self.control_btn2_2.raise_()
        self.control_btn2_3.raise_()
        self.control_btn3_2.raise_()
        self.control_btn3_3.raise_()
        self.Flight_interface_unSafty_btn.raise_()
        self.terminal_main.raise_()
        self.parameter1_box_18.raise_()
        self.parameter1_text2.raise_()
        self.parameter1_main2.raise_()
        self.parameter1_unit2.raise_()
        self.Latitude_main.raise_()
        self.Latitude_text.raise_()
        self.parameter2_main2.raise_()
        self.parameter2_unit2.raise_()
        self.parameter2_text2.raise_()
        self.parameter3_unit.raise_()
        self.parameter3_text.raise_()
        self.parameter3_main.raise_()
        self.Longitude_box.raise_()
        self.Longitude_text.raise_()
        self.Longitude_main.raise_()
        self.parameter1_chart_text.raise_()
        self.parameter2_chart_text.raise_()
        self.Gyro_yaw_main.raise_()
        self.Gyro_yaw_text.raise_()
        self.Gyro_roll_main.raise_()
        self.Gyro_roll_unit.raise_()
        self.Gyro_pitch_unit.raise_()
        self.Gyro_pitch_text.raise_()
        self.Gyro_pitch_main.raise_()
        self.Gyro_roll_text.raise_()
        self.Gyro_yaw_unit.raise_()
        self.control_next_btn.raise_()
        self.settings_box1.raise_()
        self.settings_btn3_1.raise_()
        self.settings_box2.raise_()
        self.settings_exit_btn.raise_()
        self.settings_text2_1.raise_()
        self.settings_text2_2.raise_()
        self.settings_btn3_b.raise_()
        self.settings_text1_1.raise_()
        self.settings_btn3_2.raise_()
        self.settings_text3_2.raise_()
        self.settings_btn2_1.raise_()
        self.settings_text3_1.raise_()
        self.settings_btn3_c.raise_()
        self.Set_title.raise_()
        self.set_desc.raise_()
        self.settings_btn3_a.raise_()
        self.settings_text2_3.raise_()
        self.settings_text1_2.raise_()
        self.settings_btn2_3.raise_()
        self.settings_btn2_2.raise_()
        self.settings2_img_box.raise_()
        self.set_a_spinbox1.raise_()
        self.set_a_spin1.raise_()
        self.settings_guideline.raise_()
        self.settings_btn1_1.raise_()
        self.settings_btn1_2.raise_()
        self.settings_btn1_3.raise_()
        self.settings_box2_2.raise_()
        self.settings_menubar.raise_()
        self.advanced_btn1.raise_()
        self.advanced_btn2.raise_()
        self.safty_btn1.raise_()
        self.interface_btn2.raise_()
        self.safty_btn2.raise_()
        self.interface_btn1.raise_()
        self.programinfo_btn1.raise_()
        self.sequence_btn2.raise_()
        self.data_btn2.raise_()
        self.data_btn1.raise_()
        self.sequence_btn1.raise_()
        self.programinfo_btn2.raise_()
        self.setting_btn1.raise_()
        self.export_btn2.raise_()
        self.export_btn1.raise_()
        self.control_side_box.raise_()

        self.map_main.raise_()
        self.alarm_box.raise_()
        self.alarm_title.raise_()
        self.alarm_text.raise_()
        self.alarm_icon.raise_()
        self.alarm_time.raise_()
        self.back_grad_down.raise_()
        self.Sequence_time_text.raise_()
        self.Data_info_text.raise_()
        self.flight_data_text.raise_()
        self.parameter1_box1.raise_()
        self.parameter2_box1.raise_()
        self.parameter1_gauge.raise_()
        self.parameter2_gauge.raise_()
        self.parameter2_unit1.raise_()
        self.parameter1_text1.raise_()
        self.parameter1_unit1.raise_()
        self.parameter2_main1.raise_()
        self.parameter2_text1.raise_()
        self.parameter1_main1.raise_()
        self.Map_plus_btn.raise_()
        self.Abort_text.raise_()
        self.Abort_Box.raise_()
        self.Abort_btn.raise_()
        self.Flight_interface_connect.raise_()
        self.Flight_interface_Time.raise_()

        self.confirm_box.raise_()
        self.confirm_logo.raise_()
        self.confirm_text1.raise_()
        self.confirm_text2.raise_()
        self.confirm_btn1.raise_()
        self.confirm_exit_btn.raise_()
        
        self.intro_box.raise_()
        self.intro_text2.raise_()
        self.intro_text1.raise_()
        self.intro_logo.raise_()

        self.settings_box1.hide()
        self.settings_exit_btn.hide()
        self.Set_title.hide()
        self.set_desc.hide()
        self.settings_box2.hide()
        self.settings_box2_2.hide()
        self.safty_btn1.hide()
        self.sequence_btn1.hide()
        self.data_btn1.hide()
        self.interface_btn1.hide()
        self.safty_btn2.hide()
        self.sequence_btn2.hide()
        self.data_btn2.hide()
        self.interface_btn2.hide()
        self.settings2_img_box.hide()
        self.programinfo_btn1.hide()
        self.programinfo_btn2.hide()
        self.set_a_spinbox1.hide()
        self.set_a_spin1.hide()
        self.settings_guideline.hide()
        self.advanced_btn1.hide()
        self.settings_menubar.hide()

        self.settings_btn1_1.hide()
        self.settings_btn1_2.hide()
        self.settings_btn1_3.hide()
        self.settings_text1_1.hide()
        self.settings_text1_2.hide()

        self.settings_btn2_1.hide()
        self.settings_btn2_2.hide()
        self.settings_btn2_3.hide()
        self.settings_text2_1.hide()
        self.settings_text2_2.hide()
        self.settings_text2_3.hide()

        self.settings_btn3_1.hide()
        self.settings_btn3_2.hide()
        self.settings_btn3_a.hide()
        self.settings_btn3_b.hide()
        self.settings_btn3_c.hide()
        self.settings_text3_1.hide()
        self.settings_text3_2.hide()

        self.Abort_text.hide()
        self.Abort_btn.hide()
        self.Abort_Box.hide()
        
        self.alarm_title.hide()
        self.alarm_text.hide()
        self.alarm_time.hide()
        self.alarm_icon.hide()
        self.alarm_box.hide()


        self.confirm_box.hide()
        self.confirm_btn1.hide()
        self.confirm_btn2.hide()
        self.confirm_logo.hide()
        self.confirm_text2.hide()
        self.confirm_exit_btn.hide()
        self.confirm_text1.hide()

        self.parameter1_chart_main.hide()
        self.parameter2_chart_main.hide()
        

        self.TimeUpdateThread = TimeUpdateThread()
        self.TimeUpdateThread.time_signal.connect(self.time)
        self.TimeUpdateThread.start()

        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        self.export_btn1.clicked.connect(self.export)
        self.export_btn2.clicked.connect(self.export)

        self.control_btn1_1.clicked.connect(self.sequence)
        self.control_btn1_2.clicked.connect(self.sequence)
        self.control_btn1_3.clicked.connect(self.sequence)

        self.control_btn4_1.clicked.connect(self.safty)
        self.control_btn4_2.clicked.connect(self.safty)
        self.control_btn4_3.clicked.connect(self.safty)

        self.control_btn2_1.clicked.connect(self.Manual_Ignition)
        self.control_btn2_2.clicked.connect(self.Manual_Ignition)
        self.control_btn2_3.clicked.connect(self.Manual_Ignition)

        self.confirm_btn1.clicked.connect(self.Confirm)
        self.confirm_btn2.clicked.connect(self.Confirm)

        self.control_btn5_1.clicked.connect(self.data_reset)
        self.control_btn5_2.clicked.connect(self.data_reset)

        self.setting_btn1.clicked.connect(self.settings)

        self.Map_plus_btn.clicked.connect(self.map_plus)

        self.confirm_exit_btn.clicked.connect(self.confirm_exit)

        self.Abort_btn.clicked.connect(self.abort)
 
    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.Sequence_time_text.setText(_translate("MainWindow", "T-00"))
        self.Abort_text.setText(_translate("MainWindow", "시퀸스를 취소하시려면 ABORT를 누르세요.\n"
"↓"))
        self.Data_info_text.setText(_translate("MainWindow", "Searching for device..."))
        self.flight_data_text.setText(_translate("MainWindow", "Data: N/A"))
        self.Flight_interface_Time.setText(_translate("MainWindow", "KST 12:41:00"))
        self.alarm_title.setText(_translate("MainWindow", "시퀀스 시작"))
        self.alarm_text.setText(_translate("MainWindow", "시퀀스 시스템이 정상적으로 작동 중입니다.\n"
"KST 기준 19:00에 점화 시그널이 작동됩니다."))
        self.alarm_time.setText(_translate("MainWindow", "18:09"))
        self.control_btn4_3.setText(_translate("MainWindow", "OFF"))
        self.control_btn4_2.setText(_translate("MainWindow", "안전\n"
"모드"))
        self.control_btn1_2.setText(_translate("MainWindow", "시퀀스\n"
"시작"))
        self.control_btn1_3.setText(_translate("MainWindow", "start"))
        self.control_btn5_2.setText(_translate("MainWindow", "Data Reset"))
        self.control_btn7_2.setText(_translate("MainWindow", "N"))
        self.control_btn6_2.setText(_translate("MainWindow", "N"))
        self.control_btn2_2.setText(_translate("MainWindow", "강제\n"
"점화"))
        self.control_btn2_3.setText(_translate("MainWindow", "ignition"))
        self.control_btn3_2.setText(_translate("MainWindow", "모드\n"
"변경"))
        self.control_btn3_3.setText(_translate("MainWindow", "N/A"))
        self.terminal_main.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Inter\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'Menlo,Monaco,Courier New,monospace\'; font-size:13pt; color:#d8f6ff;\">HANWOOL</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'Menlo,Monaco,Courier New,monospace\'; font-size:13pt; color:#d8f6ff;\">FLASH5 - ICS</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'Menlo,Monaco,Courier New,monospace\'; font-size:13pt; color:#d8f6ff;\">© HANWOOL with ALTIS</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'Menlo,Monaco,Courier New,monospace\'; font-size:13pt; color:#d8f6ff;\">2025 YBB(ybb1833@naver.com)</span></p></body></html>"))
        self.parameter1_text2.setText(_translate("MainWindow", "Parameter1"))
        self.parameter1_main2.setText(_translate("MainWindow", "N/A"))
        self.parameter1_unit2.setText(_translate("MainWindow", "N"))
        self.Latitude_main.setText(_translate("MainWindow", "N/A"))
        self.Latitude_text.setText(_translate("MainWindow", "Latitude"))
        self.parameter2_main2.setText(_translate("MainWindow", "N/A"))
        self.parameter2_unit2.setText(_translate("MainWindow", "Mpa"))
        self.parameter2_text2.setText(_translate("MainWindow", "Parameter2"))
        self.parameter3_unit.setText(_translate("MainWindow", "?"))
        self.parameter3_text.setText(_translate("MainWindow", "Parameter3"))
        self.parameter3_main.setText(_translate("MainWindow", "N/A"))
        self.Longitude_text.setText(_translate("MainWindow", "Longitude"))
        self.Longitude_main.setText(_translate("MainWindow", "N/A"))
        self.parameter1_chart_text.setText(_translate("MainWindow", "Parameter1"))
        self.parameter2_chart_text.setText(_translate("MainWindow", "Parameter2"))
        self.Gyro_yaw_main.setText(_translate("MainWindow", "N/A"))
        self.Gyro_yaw_text.setText(_translate("MainWindow", "Gyro - yaw"))
        self.Gyro_roll_main.setText(_translate("MainWindow", "N/A"))
        self.Gyro_roll_unit.setText(_translate("MainWindow", "degree"))
        self.Gyro_pitch_unit.setText(_translate("MainWindow", "degree"))
        self.Gyro_pitch_text.setText(_translate("MainWindow", "Gyro - pitch"))
        self.Gyro_pitch_main.setText(_translate("MainWindow", "N/A"))
        self.Gyro_roll_text.setText(_translate("MainWindow", "Gyro - roll"))
        self.Gyro_yaw_unit.setText(_translate("MainWindow", "degree"))
        self.parameter2_unit1.setText(_translate("MainWindow", "Mpa"))
        self.parameter1_text1.setText(_translate("MainWindow", "Parameter1"))
        self.parameter1_unit1.setText(_translate("MainWindow", "N"))
        self.parameter2_main1.setText(_translate("MainWindow", "N/A"))
        self.parameter2_text1.setText(_translate("MainWindow", "Parameter2"))
        self.parameter1_main1.setText(_translate("MainWindow", "N/A"))
        self.Abort_btn.setText(_translate("MainWindow", "ABORT"))
        self.Map_plus_btn.setText(_translate("MainWindow", "View larger map"))
        self.intro_text1.setText(_translate("MainWindow", "© HANWOOL with ALTIS All Rights Reserved"))
        self.intro_text2.setText(_translate("MainWindow", "ICS FLASH5 block1"))
        self.control_next_btn.setText(_translate("MainWindow", "<          1/1          >"))
        self.safty_btn1.setText(_translate("MainWindow", "Safty"))
        self.settings_text3_1.setText(_translate("MainWindow", "게이지 인터페이스"))
        self.Set_title.setText(_translate("MainWindow", "Interface"))
        self.settings_exit_btn.setText(_translate("MainWindow", "< settings"))
        self.settings_btn2_2.setText(_translate("MainWindow", "ON"))
        self.sequence_btn1.setText(_translate("MainWindow", "Sequence"))
        self.programinfo_btn1.setText(_translate("MainWindow", "Program info"))
        self.settings_text2_3.setText(_translate("MainWindow", "※ IDA 활성화 시, 자세 시각화를 위한 공간이 확보되어 그래프 출력 영역이 다소 줄어듭니다."))
        self.settings_btn3_b.setText(_translate("MainWindow", "Nomal"))
        self.settings_text2_2.setText(_translate("MainWindow", "센서 데이터를 기반으로 실시간 회전 자세를 모니터 에서 확인할수 있습니다."))
        self.set_desc.setText(_translate("MainWindow", "이곳에서 인터페이스 설정을 수정할 수 있습니다."))
        self.settings_btn1_3.setText(_translate("MainWindow", "OFF"))
        self.advanced_btn1.setText(_translate("MainWindow", "Advanced"))
        self.settings_btn3_c.setText(_translate("MainWindow", "None"))
        self.settings_text3_2.setText(_translate("MainWindow", "게이지의 인터페이스를 수정할수 있습니다."))
        self.interface_btn1.setText(_translate("MainWindow", "Interface"))
        self.settings_text1_1.setText(_translate("MainWindow", "VFS (Voice Feedback System) 활성화"))
        self.data_btn1.setText(_translate("MainWindow", "Data"))
        self.settings_btn3_a.setText(_translate("MainWindow", "Gauge bar"))
        self.settings_btn3_2.setText(_translate("MainWindow", "변경"))
        self.settings_text1_2.setText(_translate("MainWindow", "시스템 이벤트를 음성으로 안내하는 기능입니다."))
        self.settings_text2_1.setText(_translate("MainWindow", "ADI (Attitude Direction Indicator) 활성화"))
        
        self.confirm_text1.setText(_translate("MainWindow", "시퀀스 활성화"))
        self.confirm_text2.setText(_translate("MainWindow", "시퀀스를 활성화 하시겠습니까?"))
        self.confirm_btn1.setText(_translate("MainWindow", "Confirm"))
        self.confirm_exit_btn.setText(_translate("MainWindow", "← 돌아가기"))


    #알람 하이드
    def hide_alarm(self):
        for widget in [self.alarm_title, self.alarm_text, self.alarm_box, self.alarm_icon, self.alarm_time]:
            widget.hide()

    #인포 알람
    def show_info_alarm(self, title: str, text: str):
        _translate = QtCore.QCoreApplication.translate
        self.alarm_icon.setPixmap(QtGui.QPixmap(str(info_alarm_icon_img))) # 인포 아이콘으로 변경
        self.alarm_box.setStyleSheet("font:  15pt \"Inter\";\n"
"border-radius :7px;\n"
"border: 1px solid rgb(67, 160, 71);\n"
"background-color: rgb(232,245,233);") # 알람 박스 색 변경
        self.alarm_title.setText(_translate("MainWindow", title))
        self.alarm_text.setText(_translate("MainWindow", text))
        self.alarm_time.setText(_translate("MainWindow", datetime.now().strftime("%H:%M")))

        for widget in [self.alarm_title, self.alarm_icon ,self.alarm_text, self.alarm_box, self.alarm_time]:
            widget.show()

        if not hasattr(self, "feedback_timer"):
            self.feedback_timer = QTimer()
            self.feedback_timer.setSingleShot(True)
            self.feedback_timer.timeout.connect(self.hide_alarm)
        else:
            self.feedback_timer.stop()

        self.feedback_timer.start(3000)

    #경고 알람
    def show_warning_alarm(self, title: str, text: str):
        _translate = QtCore.QCoreApplication.translate
        self.alarm_icon.setPixmap(QtGui.QPixmap(str(warning_alarm_icon_img))) # 경고 아이콘으로 변경
        self.alarm_box.setStyleSheet("font:  15pt \"Inter\";\n"
"border-radius :7px;\n"
"border: 1px solid rgb(255, 152, 0);\n"
"background-color: rgb(255,243,224);") # 알람 박스 색 변경
        self.alarm_title.setText(_translate("MainWindow", title))
        self.alarm_text.setText(_translate("MainWindow", text))
        self.alarm_time.setText(_translate("MainWindow", datetime.now().strftime("%H:%M")))

        for widget in [self.alarm_title, self.alarm_icon ,self.alarm_text, self.alarm_box, self.alarm_time]:
            widget.show()

        if not hasattr(self, "feedback_timer"):
            self.feedback_timer = QTimer()
            self.feedback_timer.setSingleShot(True)
            self.feedback_timer.timeout.connect(self.hide_alarm)
        else:
            self.feedback_timer.stop()

        self.feedback_timer.start(3000)
    
    #비상 알람
    def show_emergency_alarm(self, title: str, text: str):
        _translate = QtCore.QCoreApplication.translate
        self.alarm_icon.setPixmap(QtGui.QPixmap(str(emergency_alarm_icon_img))) # 경고 아이콘으로 변경
        self.alarm_box.setStyleSheet("font:  15pt \"Inter\";\n"
"border-radius :7px;\n"
"border: 1px solid rgb(244, 67, 54);\n"
"background-color: rgb(255,235,238);") # 알람 박스 색 변경
        self.alarm_title.setText(_translate("MainWindow", title))
        self.alarm_text.setText(_translate("MainWindow", text))
        self.alarm_time.setText(_translate("MainWindow", datetime.now().strftime("%H:%M")))

        for widget in [self.alarm_title, self.alarm_icon ,self.alarm_text, self.alarm_box, self.alarm_time]:
            widget.show()

        if not hasattr(self, "feedback_timer"):
            self.feedback_timer = QTimer()
            self.feedback_timer.setSingleShot(True)
            self.feedback_timer.timeout.connect(self.hide_alarm)
        else:
            self.feedback_timer.stop()

        self.feedback_timer.start(3000)

    def _ensure_opacity(self, w):
        eff = w.graphicsEffect()
        if not isinstance(eff, QGraphicsOpacityEffect):
            eff = QGraphicsOpacityEffect(w)
            eff.setOpacity(1.0)
            w.setGraphicsEffect(eff)
        return eff
        

    def fade_out_with_logo_zoom(self, delay_ms=3000, fade_duration=1000, zoom_factor=1.2):
        _translate = QtCore.QCoreApplication.translate
        """3초 대기 후 텍스트/박스 페이드아웃 + 로고 확대 & 페이드아웃"""
        def _run():
            group = QParallelAnimationGroup()

            # 텍스트/박스 페이드아웃
            for w in [self.intro_box, self.intro_text1, self.intro_text2]:
                eff = self._ensure_opacity(w)
                anim = QPropertyAnimation(eff, b"opacity", w)
                anim.setDuration(fade_duration)
                anim.setStartValue(1.0)
                anim.setEndValue(0.0)
                anim.setEasingCurve(QEasingCurve.InOutQuad)
                group.addAnimation(anim)

            # 로고 페이드아웃
            eff_logo = self._ensure_opacity(self.intro_logo)
            fade_logo = QPropertyAnimation(eff_logo, b"opacity", self.intro_logo)
            fade_logo.setDuration(fade_duration + 500)  # 살짝 더 늦게 사라지게
            fade_logo.setStartValue(1.0)
            fade_logo.setEndValue(0.0)
            fade_logo.setEasingCurve(QEasingCurve.InOutQuad)
            group.addAnimation(fade_logo)

            # 로고 확대 애니메이션 (geometry 이용)
            rect = self.intro_logo.geometry()
            w, h = rect.width(), rect.height()
            new_w, new_h = int(w * zoom_factor), int(h * zoom_factor)
            new_x, new_y = rect.x() - (new_w - w)//2, rect.y() - (new_h - h)//2

            zoom_logo = QPropertyAnimation(self.intro_logo, b"geometry")
            zoom_logo.setDuration(fade_duration + 500)
            zoom_logo.setStartValue(rect)
            zoom_logo.setEndValue(QtCore.QRect(new_x, new_y, new_w, new_h))
            zoom_logo.setEasingCurve(QEasingCurve.OutCubic)
            group.addAnimation(zoom_logo)

            def _after():
                for w in [self.intro_box, self.intro_text1, self.intro_text2, self.intro_logo]:
                    w.hide()
                self.auto_device_connecting()
                self.start_port_watcher(interval_ms=1000, auto_when_connected=False)

            group.finished.connect(_after)
            group.start()
            self._anim_group = group  # GC 방지

        QTimer.singleShot(delay_ms, _run)
        
        


    def time(self, current_time):
        global ignition_signal
        global sim_ig
        global t
        global intro_exit
        _translate = QtCore.QCoreApplication.translate
        self.Flight_interface_Time.setText(_translate("MainWindow", f"KST {current_time}"))

        if sequence == 1:
            if abort == 0:
                if t <= 0:
                    if t == 0:
                        self.Data_info_text.setText(_translate("MainWindow", "점화 신호 전송"))
                        if ignition_signal == 0:
                            ignition_signal = 1
                            self.ser.write("ignition".encode())
                    self.Abort_btn.hide()
                    self.Abort_Box.hide()
                    self.Abort_text.hide()
                    self.Sequence_time_text.setText(_translate("Dialog", f"T+{-1*t}"))
                    t=t-1
                else:
                    self.Sequence_time_text.setText(_translate("Dialog", f"T-{t}"))
                    t=t-1
                    if VFS_count == 1:
                        if t == 10:
                            pygame.mixer.music.load(Path(__file__).parent / "mp3" / "t minus.mp3")
                            pygame.mixer.music.play()
                            print("t_minus")
                            self.terminal_main.append("t_minus")
                        elif t == 9:
                            pygame.mixer.music.load(Path(__file__).parent / "mp3" / "10.mp3")
                            pygame.mixer.music.play()
                            print("10")
                            self.terminal_main.append("10")
                        if t == 8:
                            pygame.mixer.music.load(Path(__file__).parent / "mp3" / "9.mp3")
                            pygame.mixer.music.play()
                            print("9")
                            self.terminal_main.append("9")
                        elif t == 7:
                            pygame.mixer.music.load(Path(__file__).parent / "mp3" / "8.mp3")
                            pygame.mixer.music.play()
                            print("8")
                            self.terminal_main.append("8")
                        elif t == 6:
                            pygame.mixer.music.load(Path(__file__).parent / "mp3" / "7.mp3")
                            pygame.mixer.music.play()
                            print("7")
                            self.terminal_main.append("7")
                        elif t == 5:
                            pygame.mixer.music.load(Path(__file__).parent / "mp3" / "6.mp3")
                            pygame.mixer.music.play()
                            print("6")
                            self.terminal_main.append("6")
                        elif t == 4:
                            pygame.mixer.music.load(Path(__file__).parent / "mp3" / "5.mp3")
                            pygame.mixer.music.play()
                            print("5")
                            self.terminal_main.append("5")
                        elif t == 3:
                            pygame.mixer.music.load(Path(__file__).parent / "mp3" / "4.mp3")
                            pygame.mixer.music.play()
                            print("4")
                            self.terminal_main.append("4")
                        elif t == 2:
                            pygame.mixer.music.load(Path(__file__).parent / "mp3" / "3.mp3")
                            pygame.mixer.music.play()
                            print("3")
                            self.terminal_main.append("3")
                        elif t == 1:
                            pygame.mixer.music.load(Path(__file__).parent / "mp3" / "2.mp3")
                            pygame.mixer.music.play()
                            print("2")
                            self.terminal_main.append("2")
                        elif t == 0:
                            pygame.mixer.music.load(Path(__file__).parent / "mp3" / "1.mp3")
                            pygame.mixer.music.play()
                            print("1")
                            self.terminal_main.append("1")

    def init_signal_graph(self):
        # 1) 공통 펜/브러시 캐시
        self._pen_p1 = pg.mkPen(color=(0, 200, 255), width=2)          # 추력(하늘색)
        self._pen_p2 = pg.mkPen(color=(255, 24, 116), width=2)         # 압력(핑크)
        # fill (기존 그라디언트 느낌 유지)
        grad1 = QtGui.QLinearGradient(0, 0, 0, 1000)
        grad1.setCoordinateMode(QtGui.QGradient.LogicalMode)
        grad1.setColorAt(1.0, QtGui.QColor(0, 200, 255, 250))
        grad1.setColorAt(0.0, QtGui.QColor(0, 200, 255,   0))
        self._brush_p1 = QtGui.QBrush(grad1)

        grad2 = QtGui.QLinearGradient(0, 0, 0, 10)
        grad2.setCoordinateMode(QtGui.QGradient.LogicalMode)
        grad2.setColorAt(1.0, QtGui.QColor(216, 0, 68, 250))
        grad2.setColorAt(0.0, QtGui.QColor(216, 0, 68,   0))
        self._brush_p2 = QtGui.QBrush(grad2)

        self._pen_grid = pg.mkPen((150,150,150), width=0.3)
        self._pen_max  = pg.mkPen(color='red', width=1, style=pg.QtCore.Qt.DashLine)
        self._pen_avg  = pg.mkPen(QtGui.QColor(255,179,0), width=1, style=pg.QtCore.Qt.DashLine)

        # 2) 데이터 컨테이너
        self.x_data  = []
        self.y_data  = []  # 추력(N)
        self.y2_data = []  # 압력(MPa)
        self.start_time = time.time()
        self.frame_count = 0

        # 3) PlotDataItem 생성(재사용)
        self._curve_p1 = self.parameter1_chart_main.plot([], [], pen=self._pen_p1, fillLevel=0, brush=self._brush_p1, clipToView=True)
        self._curve_p2 = self.parameter2_chart_main.plot([], [], pen=self._pen_p2, fillLevel=0, brush=self._brush_p2, clipToView=True)
        # 다운샘플링(피크)로 그리기 비용 감소
        self._curve_p1.setDownsampling(auto=True)
        self._curve_p2.setDownsampling(auto=True)

        # 4) 그리드라인(1회 생성)
        if not hasattr(self, "_p1_grid_lines"):
            self._p1_grid_lines = []
            for y in [50,100,150,200,250,300,350,400,450,500,550,600]:
                line = pg.InfiniteLine(pos=y, angle=0, pen=self._pen_grid)
                self.parameter1_chart_main.addItem(line)
                self._p1_grid_lines.append(line)

        if not hasattr(self, "_p2_grid_lines"):
            self._p2_grid_lines = []
            for y in [0.5,1,1.5,2,2.5,3,3.5,4,4.5,5,5.5,6,6.5]:
                line = pg.InfiniteLine(pos=y, angle=0, pen=self._pen_grid)
                self.parameter2_chart_main.addItem(line)
                self._p2_grid_lines.append(line)

        # 5) 최대/평균 라인 & 텍스트(재사용)
        # p1
        self._p1_max_line  = pg.InfiniteLine(angle=0, pen=self._pen_max)
        self._p1_avg_line  = pg.InfiniteLine(angle=0, pen=self._pen_avg)
        self.parameter1_chart_main.addItem(self._p1_max_line)
        self.parameter1_chart_main.addItem(self._p1_avg_line)

        self._p1_max_text  = pg.TextItem(color=QtGui.QColor(216,0,68), anchor=(0,0.5))
        self._p1_avg_text  = pg.TextItem(color=QtGui.QColor(255,179,0), anchor=(0,0.5))
        self.parameter1_chart_main.addItem(self._p1_max_text)
        self.parameter1_chart_main.addItem(self._p1_avg_text)

        # p2
        self._p2_max_line  = pg.InfiniteLine(angle=0, pen=self._pen_max)
        self._p2_avg_line  = pg.InfiniteLine(angle=0, pen=self._pen_avg)
        self.parameter2_chart_main.addItem(self._p2_max_line)
        self.parameter2_chart_main.addItem(self._p2_avg_line)

        self._p2_max_text  = pg.TextItem(color=QtGui.QColor(216,0,68), anchor=(0,0.5))
        self._p2_avg_text  = pg.TextItem(color=QtGui.QColor(255,179,0), anchor=(0,0.5))
        self.parameter2_chart_main.addItem(self._p2_max_text)
        self.parameter2_chart_main.addItem(self._p2_avg_text)

        # 6) X축 그리드(원하면)
        # self.parameter2_chart_main.showGrid(x=True, y=False, alpha=0.3)
        # self.parameter1_chart_main.showGrid(x=True, y=False, alpha=0.3)
    
    def _map_to_index(self, value, vmin, vmax, steps):
        """value를 vmin~vmax 범위를 steps(1..steps)로 균등 매핑"""
        if value <= vmin:
            return 1
        if value >= vmax:
            return steps
        # 구간 너비
        step = (vmax - vmin) / steps
        # 1..steps 범위의 인덱스(0-based -> +1)
        return int((value - vmin) // step) + 1

    def _get_gauge_pixmap(self, color, idx):
        """게이지 이미지 캐시 로더. color: 'R' or 'B'"""
        # 캐시 dict 준비
        if not hasattr(self, "_gauge_cache"):
            self._gauge_cache = {}
        key = f"{color}_{idx:02d}"
        pm = self._gauge_cache.get(key)
        if pm is None:
            filename = f"Gauge_{color}_{idx:02d}.png"
            p = Path(__file__).parent / "img" / "gauge" / filename
            pm = QtGui.QPixmap(str(p))
            self._gauge_cache[key] = pm
        return pm
    
    def gauge(self, data):
        # data: "g값,압력MPa"
        data_list = data.split(',')
        parameter1_data_g = float(data_list[0]) * 9.8  # 추력 (N)로 환산
        parameter2_data_g = float(data_list[1])        # 압력 (MPa)

        # 범위 정의 (기존 로직과 일관: 추력 0~500N, 압력 0~6MPa)
        R_MIN, R_MAX = 0.0, 500.0
        B_MIN, B_MAX = 0.0,   6.0
        STEPS = 45

        # 음수 방지(이전 코드와 동일 의미)
        if parameter1_data_g < 0:
            parameter1_data_g = 0.0
        if parameter2_data_g < 0:
            parameter2_data_g = 0.0

        # 1..45로 매핑
        index_R = self._map_to_index(parameter1_data_g, R_MIN, R_MAX, STEPS)
        index_B = self._map_to_index(parameter2_data_g, B_MIN, B_MAX, STEPS)

        # 픽스맵 캐시 사용하여 세팅
        self.parameter1_gauge.setPixmap(self._get_gauge_pixmap('R', index_R))
        self.parameter2_gauge.setPixmap(self._get_gauge_pixmap('B', index_B))

    def signal_graph(self, data):
        global avg_parameter1_2, avg_parameter2_2, max_parameter1, max_parameter2

        # 프레임 스킵(기존 유지)
        if not hasattr(self, "frame_count"):
            self.frame_count = 0
        self.frame_count += 1
        if self.frame_count % 5 != 0:
            return

        try:
            # 파싱 (기존 동일)
            data_list = data.split(',')
            parameter1_g_data = float(data_list[0]) * 9.8   # 추력(N)
            parameter2_g_data = float(data_list[1])         # 압력(MPa)

            # 초기 컨테이너 존재 확인 (안 되어 있으면 초기화)
            if not hasattr(self, "_curve_p1"):
                self.init_signal_graph()

            # 센서 이상치 처리(기존 동일)
            if parameter1_g_data > 500 or parameter1_g_data < -100:
                print(f" 이상 센서 값 감지: {parameter1_g_data:.1f}N → 0N 처리됨")
                parameter1_g_data = 0
            elif parameter1_g_data < 0:
                parameter1_g_data = 0

            # 시간축/데이터 갱신 (기존 동일)
            time_passed = time.time() - self.start_time
            self.x_data.append(time_passed)
            self.y_data.append(parameter1_g_data)
            self.y2_data.append(parameter2_g_data)

            # --- 여기부터가 핵심 최적화: clear() 금지, setData 재사용 ---

            # 1) 곡선/필 채우기 업데이트
            self._curve_p1.setData(self.x_data, self.y_data)   # 펜/브러시는 이미 설정됨
            self._curve_p2.setData(self.x_data, self.y2_data)

            # 2) 최대값 라인/텍스트 업데이트
            max_parameter1 = max(self.y_data)
            self._p1_max_line.setValue(max_parameter1)

            max_parameter2 = max(self.y2_data)
            self._p2_max_line.setValue(max_parameter2)

            # 3) 평균값 (유효 구간 내 평균) + 라인/텍스트 업데이트
            parameter1_array = np.array(self.y_data, dtype=float)
            parameter2_array = np.array(self.y2_data, dtype=float)

            Effective_value1 = 10
            Effective_value2 = 1

            # p1 평균
            valid_indices1 = np.where(parameter1_array > Effective_value1)[0]
            if len(valid_indices1) > 0:
                start_idx1, end_idx1 = valid_indices1[0], valid_indices1[-1]
                avg_parameter1_2 = float(np.mean(parameter1_array[start_idx1:end_idx1+1]))
                self._p1_avg_line.setValue(avg_parameter1_2)

                # 텍스트 위치 x(=chart1_pos) 계산(기존 로직 유지)
                x_valid_start = self.x_data[start_idx1]
                x_valid_end   = self.x_data[end_idx1]
                chart1_pos    = x_valid_end - (x_valid_end - x_valid_start) * 0.1

                self._p1_avg_text.setText(f"평균값 (실제 값과 다를수 있음!)\n{avg_parameter1_2:.3f}")
                self._p1_avg_text.setPos(chart1_pos, avg_parameter1_2)
            else:
                # 유효 구간 없을 때는 그냥 왼쪽 위 근처로 치워놓거나 숨길 수도 있음
                self._p1_avg_line.setValue(0)
                self._p1_avg_text.setText("")

            # p2 평균
            valid_indices2 = np.where(parameter2_array > Effective_value2)[0]
            if len(valid_indices2) > 0:
                start_idx2, end_idx2 = valid_indices2[0], valid_indices2[-1]
                avg_parameter2_2 = float(np.mean(parameter2_array[start_idx2:end_idx2+1]))
                self._p2_avg_line.setValue(avg_parameter2_2)

                x_valid_start2 = self.x_data[start_idx2]
                x_valid_end2   = self.x_data[end_idx2]
                chart2_pos     = x_valid_end2 - (x_valid_end2 - x_valid_start2) * 0.0  # 기존 유지

                self._p2_avg_text.setText(f"평균값 (실제 값과 다를수 있음!)\n{avg_parameter2_2:.3f}")
                self._p2_avg_text.setPos(chart2_pos, avg_parameter2_2)
            else:
                self._p2_avg_line.setValue(0)
                self._p2_avg_text.setText("")

            # 4) 최대값 텍스트(기존 문구/위치 규칙 유지)
            #    chart*_pos 는 위 평균 계산 뒤에 값이 정해지므로 그걸 사용
            #    (p1)
            if len(self.y_data) > 0:
                if len(valid_indices1) > 0:
                    chart1_pos = x_valid_end - (x_valid_end - x_valid_start) * 0.1
                else:
                    chart1_pos = self.x_data[-1] if self.x_data else 0.0
                self._p1_max_text.setText(f"최대값 (실제 값과 다를수 있음!)\n{max_parameter1:.3f}")
                self._p1_max_text.setPos(chart1_pos, max_parameter1)

            #    (p2)
            if len(self.y2_data) > 0:
                if len(valid_indices2) > 0:
                    chart2_pos = x_valid_end2 - (x_valid_end2 - x_valid_start2) * 0.0
                else:
                    chart2_pos = self.x_data[-1] if self.x_data else 0.0
                self._p2_max_text.setText(f"최대값 (실제 값과 다를수 있음!)\n{max_parameter2:.3f}")
                self._p2_max_text.setPos(chart2_pos, max_parameter2)

        except Exception as e:
            print(f"signal 처리 중 오류: {e}")
    
    def start_signal_simulator(self, hz=50):
        """가짜 텔레메트리 생성 → signal_graph(data) 호출"""
        # 이미 돌고 있으면 무시
        if hasattr(self, "_sim_timer") and self._sim_timer.isActive():
            return

        # ✅ QObject 기반 위젯을 부모로 지정
        parent_obj = getattr(self, 'centralwidget', None)
        if parent_obj is None:
            parent_obj = getattr(self, 'parameter2_chart_main', None)  # 다른 위젯도 OK
        self._sim_timer = QTimer(parent_obj)  # 또는 QTimer()로 부모 없이 생성해도 됨

        interval_ms = max(5, int(1000 / hz))
        self._sim_timer.setInterval(interval_ms)

        # 상태값
        self._sim_t0 = time.time()
        self._sim_phase = "idle"  # idle -> ramp -> plateau -> tail -> idle
        self._sim_last_switch = self._sim_t0

        # 시나리오 파라미터
        self._sim_idle_dur     = 1.5
        self._sim_ramp_dur     = 1.0
        self._sim_plateau_dur  = 2.5
        self._sim_tail_dur     = 1.2

        self._sim_thrust_peak  = 420.0  # N
        self._sim_pressure_pk  = 3.2    # MPa
        self._sim_noise_thrust = 4.0    # N
        self._sim_noise_press  = 0.03   # MPa

        def _update():
            now = time.time()
            elapsed = now - self._sim_last_switch

            # 페이즈 전환
            if self._sim_phase == "idle" and elapsed >= self._sim_idle_dur:
                self._sim_phase, self._sim_last_switch, elapsed = "ramp", now, 0
            elif self._sim_phase == "ramp" and elapsed >= self._sim_ramp_dur:
                self._sim_phase, self._sim_last_switch, elapsed = "plateau", now, 0
            elif self._sim_phase == "plateau" and elapsed >= self._sim_plateau_dur:
                self._sim_phase, self._sim_last_switch, elapsed = "tail", now, 0
            elif self._sim_phase == "tail" and elapsed >= self._sim_tail_dur:
                self._sim_phase, self._sim_last_switch, elapsed = "idle", now, 0

            # 파형 생성
            if self._sim_phase == "idle":
                thrust = 0.0
                press  = 0.98 + 0.02*math.sin(now*2.0)
            elif self._sim_phase == "ramp":
                r = min(1.0, elapsed / self._sim_ramp_dur)
                thrust = r * self._sim_thrust_peak
                press  = 0.9 + r * (self._sim_pressure_pk - 0.9)
            elif self._sim_phase == "plateau":
                thrust = self._sim_thrust_peak * (1.0 + 0.02*math.sin(now*10.0))
                press  = self._sim_pressure_pk   * (1.0 + 0.01*math.sin(now*7.0))
            else:  # tail
                r = 1.0 - min(1.0, elapsed / self._sim_tail_dur)
                thrust = r * self._sim_thrust_peak
                press  = 0.9 + r * (self._sim_pressure_pk - 0.9)

            # 노이즈
            thrust += random.uniform(-self._sim_noise_thrust, self._sim_noise_thrust)
            press  += random.uniform(-self._sim_noise_press,  self._sim_noise_press)
            thrust = max(thrust, 0.0)

            # signal_graph는 "g, MPa"로 받으니 N→g 환산
            thrust_g = thrust / 9.8
            simulated = f"{thrust_g:.5f},{press:.5f}"
            self.signal_graph(simulated)
            self.gauge(simulated)
            self.signal(simulated)

        self._sim_timer.timeout.connect(_update)
        self._sim_timer.start()


    def stop_signal_simulator(self):
        if hasattr(self, "_sim_timer"):
            self._sim_timer.stop()

    def signal(self, data):
        _translate = QtCore.QCoreApplication.translate
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

        # 로그 원본 데이터는 매번 축적
        self.log_entry += f"[{current_time}] Received data: {data}\n"

        try:
            data_list = data.split(',')
            parameter1_data = float(data_list[0]) * 9.8
            parameter2_data = float(data_list[1])

            if parameter1_data > 500 or parameter1_data < -100:
                parameter1_data = 0
            elif parameter1_data < 0:
                parameter1_data = 0

            now = time.time()
            if not hasattr(self, "_last_terminal_update"):
                self._last_terminal_update = 0.0

            if now - self._last_terminal_update >= 0.1:  # 100ms마다 한 번
                self._last_terminal_update = now
                self.flight_data_text.setText(f"[{current_time}]:{data}")
                self.terminal_main.append(f"[{current_time}]:{data}")

            # ---- 수치 라벨은 즉시 갱신 ----
            if not hasattr(self, "_p1_history"):
                self._p1_history, self._p2_history = [], []

            # 최근 10개 값만 유지
            self._p1_history.append(parameter1_data)
            self._p2_history.append(parameter2_data)
            self._p1_history = self._p1_history[-10:]
            self._p2_history = self._p2_history[-10:]

            avg_p1 = sum(self._p1_history) / len(self._p1_history)
            avg_p2 = sum(self._p2_history) / len(self._p2_history)

            self.parameter1_main1.setText(_translate("MainWindow", f"{avg_p1:.0f}"))
            self.parameter2_main1.setText(_translate("MainWindow", f"{avg_p2:.1f}"))
            self.parameter1_main2.setText(_translate("MainWindow", f"{avg_p1:.0f}"))
            self.parameter2_main2.setText(_translate("MainWindow", f"{avg_p2:.1f}"))

        except Exception as e:
            print(f"[{current_time}] signal 처리 중 오류: {e}")
    
    def export(self):
        _translate = QtCore.QCoreApplication.translate
        print("export")
        self.log_entry += "--------------------------------------------------------\n"
        self.log_entry += "- End of Data - "

        # 로그 파일 경로 설정
        log_dir = Path(__file__).parent / "LOG"
        
        # 디렉토리가 없는 경우 생성
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        # 파일 이름을 현재 시간으로 설정
        current_time_for_filename = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file_name = f"serial_log_{current_time_for_filename}.txt"
        log_file_path = os.path.join(log_dir, log_file_name)
        
        # log_entry에 저장된 로그 메시지를 파일에 추가 기록
        with open(log_file_path, "a") as log_file:
            log_file.write(self.log_entry)

        # 로그 저장 후 콘솔 출력
        print(f"Log saved in: {log_file_path}")
        print(self.log_entry.strip())
        # 피드백 출력
        self.show_info_alarm("Export", "로그가 .txt 파일로 성공적으로 저장되었습니다.\nFLASH5/LOG 에서 확인하세요.")
    
    def Confirm(self):
        _translate = QtCore.QCoreApplication.translate
        global I_S
        global t
        global t_set
        global sequence

        global IFP_count
        global IFP_confirm_popup_count
        global avg_parameter1_2
        global avg_parameter2_2
        global max_parameter1
        global max_parameter2

        # UI 숨기기 (공통)
        self.confirm_btn1.hide()
        self.confirm_btn2.hide()
        self.confirm_logo.hide()
        self.confirm_text2.hide()
        self.confirm_exit_btn.hide()
        self.confirm_text1.hide()
        self.confirm_box.hide()

        if I_S == 0:  # 수동 점화
            pygame.mixer.music.load(Path(__file__).parent / "mp3" / "ignition.mp3")
            pygame.mixer.music.play()
            print("ignition")
            self.terminal_main.append("ignition")
            self.show_info_alarm("수동 점화", "수동 점화가 시작되었습니다!\n안전에 주의하세요.")
            self.confirm_box.hide()
            self.ser.write("ignition".encode())

        if I_S == 1: # 시퀀스 시작
            self.parameter1_chart_main.show()
            self.parameter2_chart_main.show()
            t = t_set
            sequence = 1
            self.Abort_btn.show()
            self.Abort_text.show()
            self.Abort_Box.show()
            self.Sequence_time_text.setText(_translate("Dialog", f"T-{t}"))

            self.show_info_alarm("시퀀스 시작", "시퀀스가 정상적으로 시작되었습니다.\n안전에 유의하세요!")
            self.Data_info_text.setText(_translate("MainWindow", "Sequence Start"))

            self.back_grad_down.setStyleSheet("color: rgb(255, 255, 255);\n"
"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(255, 255, 255, 0), stop:1 rgba(30,144,255,70));")
            self.back_grad_up.setStyleSheet("color: rgb(255, 255, 255);\n"
"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(30,144,255,70), stop:1 rgba(255, 255, 255, 0));")


        if I_S == 2:  # 데이터 초기화
            print("data_reset")
            self.terminal_main.append("data_reset")
            self.log_entry = ""
            self.x_data = []
            self.y_data = []
            self.y2_data = []
            avg_parameter1_2 = 0
            avg_parameter2_2 = 0
            max_parameter1 = 00
            max_parameter2 = 0

            self.confirm_box.hide()
            self.parameter1_chart_main.show()
            self.parameter2_chart_main.show()

            print(f"log_entry: '{self.log_entry}'")
            print(f"x_data: {self.x_data}")
            print(f"y_data: {self.y_data}")
            print(f"y2_data: {self.y2_data}")
            print(f"avg_parameter1_2: {avg_parameter1_2}")
            print(f"avg_parameter2_2: {avg_parameter2_2}")
            print(f"max_parameter1: {max_parameter1}")
            print(f"max_parameter2: {max_parameter2}")

            self.terminal_main.append(f"avg_parameter1_2 = {avg_parameter1_2}")
            self.terminal_main.append(f"avg_parameter2_2 = {avg_parameter2_2}")
            self.terminal_main.append(f"max_parameter1 = {max_parameter1}")
            self.terminal_main.append(f"max_parameter2 = {max_parameter2}")
            self.terminal_main.append("※ 로그 및 그래프 데이터가 초기화되었습니다.")

            self.start_time = time.time()
            self.parameter1_chart_main.clear()
            self.parameter2_chart_main.clear()
            self.init_signal_graph()

            self.show_info_alarm("데이터 초기화 완료! ", "초기화 후에도 로그는 터미널에 보존됩니다.")

        if I_S == 3: # 디바이스 리커넥팅
            self.re_auto_device_connecting()
            self.parameter1_chart_main.show()
            self.parameter2_chart_main.show()

        if I_S == 4: #IFP 모드
            IFP_count = 0
            IFP_confirm_popup_count = 0
            self.confirm_box.hide()
            self.ser.write("IFP_OFF".encode())
    
    def confirm_exit(self):
        global IFP_confirm_popup_count
        self.confirm_btn1.hide()
        self.confirm_btn2.hide()
        self.confirm_box.hide()
        self.confirm_logo.hide()
        self.confirm_text2.hide()
        self.confirm_exit_btn.hide()
        self.confirm_text1.hide()
        IFP_confirm_popup_count = 0
        self.parameter1_chart_main.show()
        self.parameter2_chart_main.show()
    
    def Manual_Ignition(self):
        global I_S
        global chart_count
        I_S = 0
        _translate = QtCore.QCoreApplication.translate

        if sequence == 1:
            self.show_warning_alarm("주의", "시퀀스가 이미 진행중입니다!")
        else:
            if safty_count == 1:
                self.show_warning_alarm("안전 모드", "안전 모드가 활성화 중입니다!\n안전모드를 해제후 다시 실행하세요.")
            else:
                chart_count = 0
                print("Manual_Ignition")
                self.confirm_text1.setText(_translate("MainWindow", "수동 점화"))
                self.confirm_text2.setText(_translate("MainWindow", "수동 점화를 활성화 하시겠습니까?"))
                manual_ignition_img_path = Path(__file__).parent / "img" / "cauntion.png"
                self.confirm_logo.setPixmap(QtGui.QPixmap(str(manual_ignition_img_path)))
                self.confirm_btn1.show()
                self.confirm_btn2.show()
                self.confirm_box.show()
                self.confirm_logo.show()
                self.confirm_text2.show()
                self.confirm_exit_btn.show()
                self.confirm_text1.show()
                self.parameter1_chart_main.hide()
                self.parameter2_chart_main.hide()
    

    def device_find(self):
        global port
        # 1. 자동으로 아두이노 포트 찾기
        arduino_ports = [
            p.device for p in serial.tools.list_ports.comports()
            if DEVICE_KEYWORD in p.description
        ]

        if arduino_ports:
            port = arduino_ports[0]
        global I_S, map_plus_count
        _translate = QtCore.QCoreApplication.translate
        I_S = 3
        map_plus_count = 0
        self.Map_plus_btn.setText(_translate("MainWindow", "View larger map"))
        self.map_main.setGeometry(QtCore.QRect(20, 60, 660, 270))
        self.Map_plus_btn.setGeometry(QtCore.QRect(560, 280, 101, 31))
        self.map_main.setStyleSheet("border-radius :15px;\n"
"background-color: rgb(230, 230, 230);\n")
        self.parameter1_chart_main.show()
        self.parameter2_chart_main.show()
        self.confirm_text1.setText(_translate("MainWindow", "새로운 디바이스"))
        self.confirm_text2.setText(_translate("MainWindow", f"새로운 디바이스 '{port}'가 감지되었습니다.\n새로 연결하시겠습니까?"))
        self.confirm_logo.setPixmap(QtGui.QPixmap(str(re_connected_icon_img)))
        self.parameter1_chart_main.hide()
        self.parameter2_chart_main.hide()
        self.confirm_btn1.show()
        self.confirm_btn2.show()
        self.confirm_box.show()
        self.confirm_logo.show()
        self.confirm_text2.show()
        self.confirm_exit_btn.show()
        self.confirm_text1.show()


    def sequence(self):
        global I_S
        global chart_count
        _translate = QtCore.QCoreApplication.translate

        if sequence == 1:
            self.show_warning_alarm("주의", "시퀀스가 이미 진행중입니다!")
        else:
            if safty_count == 1:
                self.show_warning_alarm("안전 모드", "안전 모드가 활성화 중입니다!\n안전모드를 해제후 다시 실행하세요.")
            else:
                I_S = 1
                chart_count = 0
                self.confirm_text1.setText(_translate("MainWindow", "시퀀스 활성화"))
                self.confirm_text2.setText(_translate("MainWindow", "시퀀스를 활성화 하시겠습니까?"))
                sequence_ignition_img_path = Path(__file__).parent / "img" / "Sequence.png"
                self.confirm_logo.setPixmap(QtGui.QPixmap(str(sequence_ignition_img_path)))
                self.parameter1_chart_main.hide()
                self.parameter2_chart_main.hide()
                self.confirm_btn1.show()
                self.confirm_btn2.show()
                self.confirm_box.show()
                self.confirm_logo.show()
                self.confirm_text2.show()
                self.confirm_exit_btn.show()
                self.confirm_text1.show()
                
    def abort(self):
        global sequence
        global abort
        global t_set
        global safty_count

        _translate = QtCore.QCoreApplication.translate

        sequence = 0
        abort = 0
        safty_count = 1

        if VFS_count == 1:
            # 음성 출력
            pygame.mixer.music.stop()
            pygame.mixer.music.load(Path(__file__).parent / "mp3" / "abort.mp3")
            pygame.mixer.music.play()
            print("play")

        # 시퀀스 시간 초기화
        self.Sequence_time_text.setText(_translate("Dialog", f"T-{t_set}"))

        # ABORT UI 숨기기
        self.Abort_text.hide()
        self.Abort_btn.hide()
        self.Abort_Box.hide()
        
        # ABORT 피드백 출력
        self.show_emergency_alarm("ABORT - 시퀀스 중단!", "사용자에 의해 시퀀스가 중단되었습니다!\n안전모드로 자동 변환 됩니다.")
        self.control_btn4_1.setPixmap(QtGui.QPixmap(str(blue_btn_img)))
        self.control_btn4_3.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"font:  15pt \"Inter\";\n"
"color: rgb(28, 115, 255);")
        self.control_btn4_3.setText(_translate("MainWindow", "ON"))
        self.control_btn4_2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(145, 201, 255);\n"
"font: 25 10pt \"Inter\";")
        self.Data_info_text.setText(_translate("MainWindow", "ABORT - 시퀀스가 중단되었습니다!"))

        self.back_grad_down.setStyleSheet("color: rgb(255, 255, 255);\n"
"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(255, 255, 255, 0), stop:1 rgba(220,20,60,70));")
        self.back_grad_up.setStyleSheet("color: rgb(255, 255, 255);\n"
"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(220,20,60,70), stop:1 rgba(255, 255, 255, 0));")
    
    
    def re_auto_device_connecting(self):
        _translate = QtCore.QCoreApplication.translate
        print(f"기기 찾는중...")
        
        # 1. 자동으로 아두이노 포트 찾기
        arduino_ports = [
            p.device for p in serial.tools.list_ports.comports()
            if DEVICE_KEYWORD in p.description
        ]

        if arduino_ports:
            port = arduino_ports[0]
            # 자동 연결 성공
            self.ser = serial.Serial(port, 19200)
            print(f"기기 확인됨 - {port} 연결중...")
            self.Flight_interface_connect.setPixmap(QtGui.QPixmap(str(connected_img)))
            self.show_info_alarm("연결 성공", "디바이스와의 연결이 완료되었습니다.\n실시간 데이터가 정상적으로 수신되는지 확인해 주세요.")
            self.Data_info_text.setText(_translate("MainWindow", "Connect Success"))
            print(f"기기 연결 완료! - {port}")
            self.serial_reader_thread = SerialReaderThread(self.ser)
            self.serial_reader_thread.new_data_signal.connect(self.signal)
            self.serial_reader_thread.new_data_signal.connect(self.signal_graph)
            self.serial_reader_thread.new_data_signal.connect(self.gauge)
            # 스레드 시작할 때 연결
            self.serial_reader_thread = SerialReaderThread(self.ser, self.centralwidget)
            self.serial_reader_thread.new_data_signal.connect(self.on_serial_data, QtCore.Qt.QueuedConnection)
            self.serial_reader_thread.disconnected.connect(self.on_serial_disconnected, QtCore.Qt.QueuedConnection)
            self.serial_reader_thread.start()

        else:
            # 자동 연결 실패
            print("자동 연결 실패")
            self.Flight_interface_connect.setPixmap(QtGui.QPixmap(str(NoSignal_img)))
            self.show_warning_alarm("연결 실패", "디바이스와의 연결을 확인할 수 없습니다!\n디바이스를 다시 한번 확인해보세요.")


    def auto_device_connecting(self):
        _translate = QtCore.QCoreApplication.translate
        print(f"기기 찾는중...")
        
        # 1. 자동으로 아두이노 포트 찾기
        arduino_ports = [
            p.device for p in serial.tools.list_ports.comports()
            if DEVICE_KEYWORD in p.description
        ]

        if arduino_ports:
            port = arduino_ports[0]
            # 자동 연결 성공
            self.ser = serial.Serial(port, 19200)
            print(f"기기 확인됨 - {port} 연결중...")
            self.Flight_interface_connect.setPixmap(QtGui.QPixmap(str(connected_img)))
            self.show_info_alarm("연결 성공", "디바이스와의 연결이 완료되었습니다.\n실시간 데이터가 정상적으로 수신되는지 확인해 주세요.")
            self.Data_info_text.setText(_translate("MainWindow", "Connect Success"))
            print(f"기기 연결 완료! - {port}")
            self.init_signal_graph()
            self.serial_reader_thread = SerialReaderThread(self.ser)
            self.serial_reader_thread.new_data_signal.connect(self.signal)
            self.serial_reader_thread.new_data_signal.connect(self.signal_graph)
            self.serial_reader_thread.new_data_signal.connect(self.gauge)
            # 스레드 시작할 때 연결
            self.serial_reader_thread = SerialReaderThread(self.ser, self.centralwidget)
            self.serial_reader_thread.new_data_signal.connect(self.on_serial_data, QtCore.Qt.QueuedConnection)
            self.serial_reader_thread.disconnected.connect(self.on_serial_disconnected, QtCore.Qt.QueuedConnection)
            self.serial_reader_thread.start()

        else:
            # 자동 연결 실패
            print("자동 연결 실패")
            self.Flight_interface_connect.setPixmap(QtGui.QPixmap(str(NoSignal_img)))
            self.show_warning_alarm("연결 실패", "디바이스와의 연결을 확인할 수 없습니다!\n시뮬레이션 모드로 전환합니다.")
            self.Data_info_text.setText(_translate("MainWindow", "Simulation Mode"))
            self.init_signal_graph()
            self.start_signal_simulator(hz=100)  # 시뮬레이션 가상 랜덤 데이터를 50Hz 로 출력
            self.control_btn3_3.setText(_translate("MainWindow", "Simulation"))

        self.parameter1_chart_main.show()
        self.parameter2_chart_main.show()
            
        t = t_set
        self.Sequence_time_text.setText(_translate("Dialog", f"T-{t}"))
        current_time = datetime.now().strftime("%Y-%m-%d %H")[:-3]
        self.log_entry = f"{current_time} 의 데이터 - Logging initiated\n"
        self.log_entry += "↓ data ↓\n"
        self.log_entry += "--------------------------------------------------------\n"

    def on_serial_data(self, line: str):
        # 한 곳에서만 처리(중복 파싱 방지)
        self.signal(line)
        self.signal_graph(line)
        self.gauge(line)

    def on_serial_disconnected(self, reason: str):
        global abort
        global safty_count
        global sequence
        _translate = QtCore.QCoreApplication.translate

        # 1) 알림 & 상태
        print("disconnected!")
        if sequence == 1:
            sequence = 0
            abort = 0
            safty_count = 1

            if VFS_count == 1:
                # 음성 출력
                pygame.mixer.music.stop()
                pygame.mixer.music.load(Path(__file__).parent / "mp3" / "abort.mp3")
                pygame.mixer.music.play()
                print("play")

            # 시퀀스 시간 초기화
            self.Sequence_time_text.setText(_translate("Dialog", f"T-{t_set}"))

            # ABORT UI 숨기기
            self.Abort_text.hide()
            self.Abort_btn.hide()
            self.Abort_Box.hide()
            
            # ABORT 피드백 출력
            self.control_btn4_1.setPixmap(QtGui.QPixmap(str(blue_btn_img)))
            self.control_btn4_3.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
    "border-color: rgb(0, 0, 0);\n"
    "font:  15pt \"Inter\";\n"
    "color: rgb(28, 115, 255);")
            self.control_btn4_3.setText(_translate("MainWindow", "ON"))
            self.control_btn4_2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
    "border-color: rgb(0, 0, 0);\n"
    "color: rgb(145, 201, 255);\n"
    "font: 25 10pt \"Inter\";")
            self.Data_info_text.setText(_translate("MainWindow", "ABORT - 시퀀스가 중단되었습니다!"))
            self.show_emergency_alarm("ABORT - 시퀀스 중단!", "디바이스 연결이 해제되어 자동으로 ABORT가 작동하였습니다!\n디바이스를 확인해보세요.")
            self.back_grad_down.setStyleSheet("color: rgb(255, 255, 255);\n"
"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(255, 255, 255, 0), stop:1 rgba(220,20,60,70));")
            self.back_grad_up.setStyleSheet("color: rgb(255, 255, 255);\n"
"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(220,20,60,70), stop:1 rgba(255, 255, 255, 0));")
            self.Flight_interface_connect.setPixmap(QtGui.QPixmap(str(NoSignal_img)))
            
        else:
            self.show_warning_alarm("연결 해제","디바이스 연결이 해제되었습니다!\n디바이스를 확인해보세요.")
            self.Data_info_text.setText(_translate("MainWindow", "Disconnected"))
            self.Flight_interface_connect.setPixmap(QtGui.QPixmap(str(NoSignal_img)))
        try:
            if hasattr(self, "serial_reader_thread") and self.serial_reader_thread:
                self.serial_reader_thread.stop()
                self.serial_reader_thread.quit()
                self.serial_reader_thread.wait(1500)
                self.serial_reader_thread = None
        except Exception:
            pass

        try:
            if hasattr(self, "ser") and self.ser:
                # 포트 닫기 (이미 닫혔어도 예외 없이 지나가게)
                self.ser.close()
        except Exception:
            pass

    def safty(self):
        global safty_count
        _translate = QtCore.QCoreApplication.translate

        if sequence == 1:
            self.show_info_alarm("안전모드", "시퀀스중에는 안전모드 전환이 불가능 합니다!\n시퀀스를 중단후 안전모드로 전환하세요.")
        else:
            if safty_count == 1:
                self.control_btn4_1.setPixmap(QtGui.QPixmap(str(red_btn_img)))
                self.control_btn4_3.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"font:  15pt \"Inter\";\n"
"color: rgb(211, 35, 0);")
                self.control_btn4_3.setText(_translate("MainWindow", "OFF"))
                self.control_btn4_2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"font: 25 10pt \"Inter\";\n"
"color: rgb(183, 142, 142);")
                
                safty_count = 0
                self.back_grad_down.setStyleSheet("color: rgb(255, 255, 255);\n"
"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(255, 255, 255, 0), stop:1 rgba(0,0,0,150));")
                self.back_grad_up.setStyleSheet("color: rgb(255, 255, 255);\n"
"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(0,0,0,150), stop:1 rgba(255, 255, 255, 0));")
                
                self.show_info_alarm("안전모드", "안전모드로 해제하였습니다!\n안전에 주의하세요.")
            else:
                self.control_btn4_1.setPixmap(QtGui.QPixmap(str(blue_btn_img)))
                self.control_btn4_3.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"font:  15pt \"Inter\";\n"
"color: rgb(28, 115, 255);")
                self.control_btn4_3.setText(_translate("MainWindow", "ON"))
                self.control_btn4_2.setStyleSheet("background-color: rgb(0, 0, 0, 0);\n"
"border-color: rgb(0, 0, 0);\n"
"color: rgb(145, 201, 255);\n"
"font: 25 10pt \"Inter\";")

                self.back_grad_down.setStyleSheet("color: rgb(255, 255, 255);\n"
"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(255, 255, 255, 0), stop:1 rgba(220,20,60,70));")
                self.back_grad_up.setStyleSheet("color: rgb(255, 255, 255);\n"
"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(220,20,60,70), stop:1 rgba(255, 255, 255, 0));")
                self.show_info_alarm("안전모드", "안전모드로 전환하였습니다!\n모든 동작이 제한 됩니다.")
                safty_count = 1
                

    def start_port_watcher(self, interval_ms=1000, auto_when_connected=False):
        """
        새 포트가 생기면 device_find() 실행.
        - interval_ms: 폴링 주기(ms)
        - auto_when_connected: 이미 연결 상태여도 새 포트 등장 시 device_find 실행할지 여부
        """
        # 이전 목록 스냅샷
        self._known_ports = set(p.device for p in list_ports.comports())

        # 기존 타이머가 있으면 재사용
        if hasattr(self, "_port_watch_timer") and self._port_watch_timer is not None:
            self._port_watch_timer.stop()
        parent_obj = getattr(self, "centralwidget", None)
        self._port_watch_timer = QTimer(parent_obj)
        self._port_watch_timer.setInterval(interval_ms)
        self._port_watch_timer.timeout.connect(
            lambda: self._check_ports_changed(auto_when_connected)
        )
        self._port_watch_timer.start()

    def stop_port_watcher(self):
        if hasattr(self, "_port_watch_timer") and self._port_watch_timer is not None:
            self._port_watch_timer.stop()

    def _check_ports_changed(self, auto_when_connected: bool):
        """내부용: 포트 목록 변화 감지"""
        try:
            current_ports = set(p.device for p in list_ports.comports())
        except Exception:
            return

        # 새로 추가된 포트
        added = current_ports - getattr(self, "_known_ports", set())
        # 제거된 포트(쓸 일 있으면 활용)
        removed = getattr(self, "_known_ports", set()) - current_ports

        if added:
            # 디바운스(짧은 시간 중복 트리거 방지)
            if not hasattr(self, "_last_device_find_ms"):
                self._last_device_find_ms = 0
            now_ms = int(time.time() * 1000)
            if now_ms - self._last_device_find_ms > 800:  # 0.8s 디바운스
                self._last_device_find_ms = now_ms

                # 이미 연결 중이면 스킵할지 여부
                if not auto_when_connected and getattr(self, "device_connected", False):
                    # 연결 중이면 목록만 갱신하고 종료
                    self._known_ports = current_ports
                    return

                # ✅ 새 포트 감지 시 실행
                try:
                    self.device_find()
                except Exception as e:
                    # 필요하면 로깅
                    pass

        # 스냅샷 업데이트
        self._known_ports = current_ports

    def data_reset(self):

        global I_S
        global chart_count
        _translate = QtCore.QCoreApplication.translate
        if data_safe_count == 1:
            self.show_warning_alarm("초기화 차단", "데이터 초기화가 차단되었습니다!")
        else:
            if safty_count == 1:
                self.show_warning_alarm("Safty MODE", "안전 모드가 활성화 중입니다!")
            else:
                I_S = 2
                chart_count = 0
                self.confirm_text1.setText(_translate("MainWindow", "데이터 초기화"))
                self.confirm_text2.setText(_translate("MainWindow", "데이터를 정말 초기화하시겠습니까?"))
                data_reset_img_path = Path(__file__).parent / "img" / "cauntion.png"
                self.confirm_logo.setPixmap(QtGui.QPixmap(str(data_reset_img_path)))
                self.confirm_btn1.show()
                self.confirm_btn2.show()
                self.confirm_box.show()
                self.confirm_logo.show()
                self.confirm_text2.show()
                self.confirm_exit_btn.show()
                self.confirm_text1.show()
                self.parameter1_chart_main.hide()
                self.parameter2_chart_main.hide()

    def map_plus(self):
        global map_plus_count
        _translate = QtCore.QCoreApplication.translate
        if map_plus_count == 0:
            map_plus_count = 1
            self.Map_plus_btn.setText(_translate("MainWindow", "View smaller map"))
            self.map_main.setGeometry(QtCore.QRect(0, 0, 1471, 841))
            self.Map_plus_btn.setGeometry(QtCore.QRect(1320, 360, 121, 41))
            self.map_main.setStyleSheet("background-color: rgb(230, 230, 230);")
            self.parameter1_chart_main.hide()
            self.parameter2_chart_main.hide()
        else:
            map_plus_count = 0
            self.Map_plus_btn.setText(_translate("MainWindow", "View larger map"))
            self.map_main.setGeometry(QtCore.QRect(20, 60, 660, 270))
            self.Map_plus_btn.setGeometry(QtCore.QRect(560, 280, 101, 31))
            self.map_main.setStyleSheet("border-radius :15px;\n"
"background-color: rgb(230, 230, 230);\n")
            self.parameter1_chart_main.show()
            self.parameter2_chart_main.show()

    def settings(self):
        print("미지원 기능 - 추후 지원 예정")
        self.show_info_alarm("미지원 기능", "아직 지원하지 않는 기능입니다.\n- 추후 지원 예정입니다.")


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    ui.fade_out_with_logo_zoom()
    sys.exit(app.exec_())
