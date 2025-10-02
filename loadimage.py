import base64
import io
import re
from PIL import Image
import numpy as np

def load_image_from_data_url(data_url):
    match = re.search(r'base64,(.*)', data_url)
    if not match:
        raise ValueError("Invalid data URL: no Base64 data found.")
    
    base64_data = match.group(1)
    decoded_bytes = base64.b64decode(base64_data)
    image_bytes = io.BytesIO(decoded_bytes)
    image = Image.open(image_bytes)
    return np.array(image)
