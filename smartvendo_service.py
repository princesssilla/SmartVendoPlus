import time
import os
import logging
import sys
import threading
import traceback

import numpy as np
from PIL import Image
import cv2
import serial
import tflite_runtime.interpreter as tflite

# === CONFIGURATION ===
SERIAL_PORT = "/dev/ttyUSB0"    # change if needed
BAUDRATE = 115200
MODEL_PATH = "smartvendo.tflite"    # path to your tflite model
IMG_SIZE = (224, 224)               # model input size (width, height)
VALID_THRESHOLD = 0.7
INVALID_THRESHOLD = 0.3

def find_usb_cameras(name="EMEET"):
    devices = []
    for i in range(10):  # check /dev/video0..video9
        cap = cv2.VideoCapture(i)
        if not cap.isOpened():
            continue
        # try reading a frame
        ret, _ = cap.read()
        cap.release()
        if ret:
            # optional: check the device name via v4l2-ctl
            try:
                with os.popen(f"v4l2-ctl -d /dev/video{i} --all | grep '{name}'") as f:
                    if name in f.read():
                        devices.append(i)
            except:
                devices.append(i)
    return devices

CAM_IDS = find_usb_cameras()
print(f"Detected cameras: {CAM_IDS}")

# Configure which camera is for plastic and which for paper
# Assuming first camera is plastic, second is paper
# You can swap these if needed
PLASTIC_CAMERA_INDEX = 0 if len(CAM_IDS) > 0 else None
PAPER_CAMERA_INDEX = 1 if len(CAM_IDS) > 1 else None

SERIAL_TIMEOUT = 1                  # seconds
CAM_OPEN_RETRY = 3                  # attempts to reopen camera
CAM_READ_TIMEOUT = 2.0              # seconds to wait trying to read a frame

# Logging
LOGFILE = "/home/pi/smartvendo/smartvendo.log"
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(LOGFILE),
        logging.StreamHandler(sys.stdout)
    ]
)

# === TFLITE / MODEL LOADING ===
def load_interpreter(model_path):
    logging.info("Loading TFLite model: %s", model_path)
    interpreter = tflite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    logging.info("Model loaded. Input details: %s | Output details: %s", input_details, output_details)
    return interpreter, input_details, output_details

interpreter, input_details, output_details = load_interpreter(MODEL_PATH)
input_idx = input_details[0]['index']
input_dtype = input_details[0]['dtype']
input_shape = input_details[0]['shape']
input_quant = input_details[0].get('quantization', (0.0, 0))
output_idx = output_details[0]['index']
output_dtype = output_details[0]['dtype']
output_quant = output_details[0].get('quantization', (0.0, 0))

logging.info("Input dtype=%s shape=%s quant=%s", input_dtype, input_shape, input_quant)
logging.info("Output dtype=%s quant=%s", output_dtype, output_quant)

# === CAMERA HANDLER ===
class CameraHandler:
    def __init__(self, cam_id):
        self.cam_id = cam_id
        self.cap = None
        self.lock = threading.Lock()
        self.open()

    def open(self):
        with self.lock:
            if self.cap is not None and self.cap.isOpened():
                return True
            logging.info("Opening camera %s", self.cam_id)
            self.cap = cv2.VideoCapture(self.cam_id, cv2.CAP_V4L2)
            # common settings:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_SIZE[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_SIZE[1])
            # Wait a short time to let camera warm up
            time.sleep(0.5)
            ok = self.cap.isOpened()
            if not ok:
                logging.warning("Failed to open camera %s", self.cam_id)
            else:
                logging.info("Camera %s opened", self.cam_id)
            return ok

    def read_frame(self, timeout=CAM_READ_TIMEOUT, flush_frames=5):
        """Return RGB numpy array or None on failure. Flushes a few frames to avoid stale-buffer reads."""
        start = time.time()
        while True:
            with self.lock:
                if self.cap is None:
                    if not self.open():
                        return None
                # Discard a few frames to flush the driver buffer
                frame = None
                for _ in range(flush_frames):
                    # use grab() when available (faster) then retrieve once at the end
                    self.cap.grab()       # final fresh frame
                ret, frame = self.cap.read() 
            if ret and frame is not None:
                try:
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                except Exception:
                    rgb = frame[..., ::-1]
                return rgb
            if time.time() - start > timeout:
                logging.warning("Timeout reading camera %s", self.cam_id)
                return None
            time.sleep(0.05)

    def close(self):
        with self.lock:
            if self.cap:
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap = None

# Instantiate cameras
cameras = [CameraHandler(cid) for cid in CAM_IDS]

# === PREPROCESS & INFERENCE ===
def preprocess_for_model(img_rgb):
    """Given RGB numpy array (H,W,3) produce input tensor matching model dtype and quantization.
       We assume MobileNet-like training preprocessing: scale to [-1,1]. If your model uses different
       preprocessing change here.
    """
    # Resize to target
    img = cv2.resize(img_rgb, IMG_SIZE, interpolation=cv2.INTER_AREA)
    # Convert to float32
    img_f = img.astype(np.float32)

    # MobileNet style: map 0..255 -> -1..1
    arr_norm = (img_f / 127.5) - 1.0  # shape (H,W,3)

    # final shape: (1,H,W,3)
    arr_norm = np.expand_dims(arr_norm, axis=0)

    # If model expects float32:
    if input_dtype == np.float32:
        return arr_norm.astype(np.float32)

    # If model expects quantized input (uint8 or int8):
    scale, zero_point = input_quant if input_quant is not None else (0.0, 0)
    if scale == 0.0:
        # fallback mapping: map arr_norm [-1,1] to approx int8 range
        if input_dtype == np.uint8:
            # map -1..1 to 0..255
            q = np.clip(((arr_norm + 1.0) * 127.5), 0, 255).astype(np.uint8)
        else:
            # int8: -1..1 -> -127..127
            q = np.clip(np.round(arr_norm * 127.0), -128, 127).astype(np.int8)
        return q
    else:
        # quantize using scale & zero_point
        q = np.round(arr_norm / scale + zero_point).astype(input_dtype)
        # clip based on dtype
        if input_dtype == np.uint8:
            q = np.clip(q, 0, 255)
        else:
            q = np.clip(q, -128, 127)
        return q

def decode_output(raw_out):
    """Convert raw TFLite output to float probability [0..1] using quantization params if necessary."""
    out_raw = np.array(raw_out).reshape(-1)
    if output_dtype == np.float32:
        return float(out_raw[0])
    # quantized
    scale, zero_point = output_quant if output_quant is not None else (0.0, 0)
    if scale == 0.0:
        # No scale info: assume symmetric int8 scaled to [-1,1] then sigmoid? fallback: map to -1..1
        if output_dtype == np.uint8:
            # 0..255 -> 0..1
            return float(out_raw[0] / 255.0)
        else:
            return float((out_raw[0] + 128) / 255.0)
    else:
        return float(scale * (out_raw[0] - zero_point))

def run_inference_on_rgb(img_rgb):
    """Return probability float (0..1) or None on error"""
    try:
        input_tensor = preprocess_for_model(img_rgb)
        interpreter.set_tensor(input_idx, input_tensor)
        interpreter.invoke()
        out = interpreter.get_tensor(output_idx)
        prob = decode_output(out)
        return prob
    except Exception as e:
        logging.error("Inference error: %s\n%s", e, traceback.format_exc())
        return None

# === SERIAL HANDLING ===
def open_serial():
    while True:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=SERIAL_TIMEOUT)
            logging.info("Opened serial port %s @%d", SERIAL_PORT, BAUDRATE)
            return ser
        except Exception as e:
            logging.error("Failed opening serial %s: %s. Retrying in 3s...", SERIAL_PORT, e)
            time.sleep(3)

ser = open_serial()

def decide_single_result(prob):
    """Decision logic for single camera input"""
    if prob is None:
        return "RP:UNCERTAIN"
    elif prob >= VALID_THRESHOLD:
        return "RP:VALID"
    elif prob <= INVALID_THRESHOLD:
        return "RP:INVALID"
    else:
        return "RP:UNCERTAIN"

# === MAIN LOOP ===
logging.info("Service ready. Waiting for Mega commands on serial...")
logging.info("Plastic camera index: %s, Paper camera index: %s", 
             PLASTIC_CAMERA_INDEX, PAPER_CAMERA_INDEX)

try:
    while True:
        try:
            line = ser.readline().decode(errors='ignore').strip()
        except Exception as e:
            logging.error("Serial read error: %s. Reopening serial port...", e)
            ser.close()
            ser = open_serial()
            line = ""

        if line:
            logging.info("Received from Mega: '%s'", line)

        # Handle plastic scanning
        if line == "PL":
            if PLASTIC_CAMERA_INDEX is None:
                logging.error("Plastic camera not configured")
                result = "RP:UNCERTAIN"
            else:
                camera_idx = PLASTIC_CAMERA_INDEX
                camera_id = CAM_IDS[camera_idx]
                logging.info("Plastic scan command received. Using camera %s", camera_id)
                
                frame = cameras[camera_idx].read_frame()
                if frame is None:
                    logging.warning("Plastic camera returned no frame")
                    result = "RP:UNCERTAIN"
                else:
                    prob = run_inference_on_rgb(frame)
                    logging.info("Plastic camera prob=%.4f", prob if prob is not None else float('nan'))
                    result = decide_single_result(prob)
            
            # Send reply to Mega
            try:
                ser.write((result + "\n").encode())
                logging.info("Sent to Mega: %s", result)
            except Exception as e:
                logging.error("Failed to write to serial: %s. Reopening serial port...", e)
                ser.close()
                ser = open_serial()

        # Handle paper scanning
        elif line == "PA":
            if PAPER_CAMERA_INDEX is None:
                logging.error("Paper camera not configured")
                result = "RP:UNCERTAIN"
            else:
                camera_idx = PAPER_CAMERA_INDEX
                camera_id = CAM_IDS[camera_idx]
                logging.info("Paper scan command received. Using camera %s", camera_id)
                
                frame = cameras[camera_idx].read_frame()
                if frame is None:
                    logging.warning("Paper camera returned no frame")
                    result = "RP:UNCERTAIN"
                else:
                    prob = run_inference_on_rgb(frame)
                    logging.info("Paper camera prob=%.4f", prob if prob is not None else float('nan'))
                    result = decide_single_result(prob)
            
            # Send reply to Mega
            try:
                ser.write((result + "\n").encode())
                logging.info("Sent to Mega: %s", result)
            except Exception as e:
                logging.error("Failed to write to serial: %s. Reopening serial port...", e)
                ser.close()
                ser = open_serial()

        # Keep backward compatibility with MEGA:SCAN if needed
        elif line == "MEGA:SCAN":
            logging.warning("Using deprecated MEGA:SCAN command. Defaulting to plastic camera.")
            # Default to plastic camera for backward compatibility
            if PLASTIC_CAMERA_INDEX is not None:
                camera_idx = PLASTIC_CAMERA_INDEX
                camera_id = CAM_IDS[camera_idx]
                logging.info("Using plastic camera %s for backward compatibility", camera_id)
                
                frame = cameras[camera_idx].read_frame()
                if frame is None:
                    logging.warning("Camera returned no frame")
                    result = "RP:UNCERTAIN"
                else:
                    prob = run_inference_on_rgb(frame)
                    logging.info("Camera prob=%.4f", prob if prob is not None else float('nan'))
                    result = decide_single_result(prob)
            else:
                result = "RP:UNCERTAIN"
            
            # Send reply to Mega
            try:
                ser.write((result + "\n").encode())
                logging.info("Sent to Mega: %s", result)
            except Exception as e:
                logging.error("Failed to write to serial: %s. Reopening serial port...", e)
                ser.close()
                ser = open_serial()

        # tiny sleep to avoid busy loop
        time.sleep(0.01)

except KeyboardInterrupt:
    logging.info("Interrupted by user, closing...")
finally:
    try:
        ser.close()
    except Exception:
        pass
    for cam in cameras:
        cam.close()
    logging.info("Service stopped.")