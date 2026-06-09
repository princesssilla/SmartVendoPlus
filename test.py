import numpy as np
import tflite_runtime.interpreter as tflite
import os
from PIL import Image

TFLITE_MODEL = "smartvendo.tflite"
IMG_PATH = "validd.jpg"
IMG_SIZE = (224, 224)
if not os.path.exists(IMG_PATH):
    raise FileNotFoundError(f"Image not found: {IMG_PATH}")

# Load interpreter
interpreter = tflite.Interpreter(model_path=TFLITE_MODEL)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
print("Input details:", input_details)
print("Output details:", output_details)

# Preprocess sample image
img = Image.open(IMG_PATH).convert("RGB").resize(IMG_SIZE)
input_data = np.array(img)

# If model expects uint8 input (quantized), cast accordingly:
if input_details[0]['dtype'] == np.uint8:
    input_data = input_data.astype(np.uint8)
else:
    # normalize to [0,1] for float model
    input_data = (input_data.astype(np.float32) / 255.0).astype(np.float32)

# Add batch dim
input_data = np.expand_dims(input_data, axis=0)

interpreter.set_tensor(input_details[0]['index'], input_data)
interpreter.invoke()
out = interpreter.get_tensor(output_details[0]['index'])
prob = float(out[0][0])

if prob >= 0.7:
    label = "valid"
elif prob <= 0.3:
    label = "invalid"
else:
    label = "uncertain"

print(label, prob)
