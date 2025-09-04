#!/usr/bin/env python3
import os
import time
import threading
import requests
import streamlit as st
import numpy as np
import cv2
from xbox import XboxBotDriver
import signal as _signal

# Configuration
API_BASE = os.getenv("BOT_API_BASE", "http://0.0.0.0:6969")
MOVE_ENDPOINT = f"{API_BASE}/robot/move"
STOP_ENDPOINT = f"{API_BASE}/robot/stop"
CAMERA_ENDPOINT = f"{API_BASE}/camera/frame"
STATUS_ENDPOINT = f"{API_BASE}/status"

SEND_PERIOD = 0.1
REQ_TIMEOUT = 0.25
DEVICE_HINT = os.getenv("CONTROLLER_DEVICE")

class RobotController:
    def __init__(self):
        self.driver = None
        self.send_thread = None
        self.running = False
        self.xbox_enabled = False
        
    def post_move(self, payload: dict):
        try:
            requests.post(MOVE_ENDPOINT, headers={"Content-Type":"application/json"}, 
                         json=payload, timeout=REQ_TIMEOUT)
        except requests.exceptions.RequestException:
            pass
    
    def post_stop(self):
        try:
            requests.post(STOP_ENDPOINT, timeout=REQ_TIMEOUT)
        except requests.RequestException:
            pass
    
    def send_loop(self):
        last = 0.0
        try:
            while self.running and self.xbox_enabled and self.driver and self.driver.running:
                now = time.monotonic()
                if now - last >= SEND_PERIOD:
                    last = now
                    try:
                        payload = self.driver.get_payload()
                        self.post_move(payload)
                    except Exception as e:
                        print(f"Controller error: {e}")
                        break
                time.sleep(0.01)
        except Exception as e:
            print(f"Xbox controller error: {e}")
        finally:
            self.post_stop()
    
    def start_xbox_control(self):
        if not self.xbox_enabled or self.running:
            return False
            
        try:
            _orig_signal = _signal.signal
            try:
                _signal.signal = lambda *a, **k: None
                self.driver = XboxBotDriver(device_hint=DEVICE_HINT)
            finally:
                _signal.signal = _orig_signal
            
            self.driver.start_event_and_control_loops()
            self.running = True
            self.send_thread = threading.Thread(target=self.send_loop, daemon=True)
            self.send_thread.start()
            return True
                    
        except Exception as e:
            print(f"Failed to start Xbox controller: {e}")
            self.xbox_enabled = False
            return False
    
    def stop_xbox_control(self):
        self.running = False
        if self.driver:
            self.driver.running = False
        if self.send_thread and self.send_thread.is_alive():
            self.send_thread.join(timeout=2.0)
        self.xbox_enabled = False
        self.driver = None
        self.send_thread = None
        self.post_stop()

def fetch_camera_frame():
    """Simple synchronous frame fetch"""
    try:
        response = requests.get(CAMERA_ENDPOINT, timeout=1.0, headers={"Accept": "image/jpeg"})
        response.raise_for_status()
        arr = np.frombuffer(response.content, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError("Failed to decode JPEG")
        # resize
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        return rgb
    except Exception:
        return None

def get_robot_status():
    try:
        response = requests.get(STATUS_ENDPOINT, timeout=1.0)
        if response.status_code == 200:
            return response.json()
        else:
            return {"ok": False, "ros_connected": False, "camera_active": False}
    except requests.exceptions.RequestException:
        return {"ok": False, "ros_connected": False, "camera_active": False}

def xbox_toggle_callback():
    controller = st.session_state.controller
    if st.session_state.xbox_toggle:
        controller.xbox_enabled = True
        controller.start_xbox_control()
    else:
        controller.stop_xbox_control()

def main():
    st.set_page_config(
        page_title="Noir",
        page_icon="🤖",
        layout="centered"
    )
    
    # Remove extra padding and make layout compact
    st.markdown("""
    <style>
        .block-container {
            padding-top: 1rem;
            padding-bottom: 0rem;
            padding-left: 1rem;
            padding-right: 1rem;
        }
        .main .block-container {
            max-width: 100%;
        }
        h1 {
            margin-top: 0rem;
            margin-bottom: 1rem;
        }
    </style>
    """, unsafe_allow_html=True)
    
    st.title("Noir")
    
    # Initialize session state
    if 'controller' not in st.session_state:
        st.session_state.controller = RobotController()
    
    if 'xbox_toggle' not in st.session_state:
        st.session_state.xbox_toggle = False
    
    controller = st.session_state.controller
    
    # Sidebar controls
    st.sidebar.header("🎮 Control Settings")
    
    # Xbox controller toggle switch
    st.sidebar.toggle(
        "Xbox Controller", 
        value=st.session_state.xbox_toggle,
        key="xbox_toggle",
        on_change=xbox_toggle_callback
    )
    
    # Robot status with smaller cards in sidebar
    st.sidebar.header("📊 System Status")
    status_info = get_robot_status()
    
    # Create smaller status cards directly in sidebar context
    def render_status_card_sidebar(title, status, icon_on="🟢", icon_off="🔴"):
        """Render a smaller status card in the sidebar"""
        status_color = "#28a745" if status else "#dc3545"
        bg_color = "#d4edda" if status else "#f8d7da"
        icon = icon_on if status else icon_off
        status_text = "Online" if status else "Offline"
        
        st.sidebar.markdown(f"""
        <div style="
            background: {bg_color};
            border: 1px solid {status_color};
            border-radius: 6px;
            padding: 6px;
            margin: 3px 0;
            text-align: center;
        ">
            <div style="font-size: 16px; margin-bottom: 2px;">{icon}</div>
            <div style="font-weight: bold; color: {status_color}; font-size: 11px;">{title}</div>
            <div style="font-size: 9px; color: {status_color}; opacity: 0.8;">{status_text}</div>
        </div>
        """, unsafe_allow_html=True)
    
    # Create three smaller status cards in sidebar
    render_status_card_sidebar("System", status_info.get("ok", False), "⚡", "❌")
    render_status_card_sidebar("ROS", status_info.get("ros_connected", False), "🔗", "🔌")
    render_status_card_sidebar("Camera", status_info.get("camera_active", False), "📸", "📹")
    
    # Add a subtle timestamp in sidebar
    st.sidebar.markdown(f"""
    <div style="
        text-align: center;
        font-size: 9px;
        color: #6c757d;
        margin-top: 8px;
        padding: 4px;
        border-top: 1px solid #dee2e6;
    ">
        Last updated: {time.strftime('%H:%M:%S')}
    </div>
    """, unsafe_allow_html=True)
        
    # Fetch and display frame with viewport-fitted height
    frame = fetch_camera_frame()
    if frame is not None:
        # Calculate image height to fit viewport (leaving room for header and margins)
        st.markdown("""
        <style>
            .stImage > img {
                max-height: 60vh;
                width: auto;
                object-fit: contain;
            }
        </style>
        """, unsafe_allow_html=True)
        st.image(frame, channels="RGB", width="stretch")  # FIXED: Changed from use_column_width=True
    else:
        st.error("📡 Unable to fetch camera frame")
    
    # Auto-refresh with fast interval (4 FPS)
    time.sleep(0.25)
    st.rerun()

if __name__ == "__main__":
    main()
