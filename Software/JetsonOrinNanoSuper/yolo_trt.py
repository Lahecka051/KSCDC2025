import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

CONF_THRESHOLD = 0.4

class YOLOTRT:
    def __init__(self, engine_path="/home/user/models/yolov8n-fire.engine", input_shape=(640, 640)):
        self.engine_path = engine_path
        self.input_shape = input_shape
        self._load_engine()

    def _load_engine(self):
        TRT_LOGGER = trt.Logger(trt.Logger.INFO)
        with open(self.engine_path, "rb") as f:
            runtime = trt.Runtime(TRT_LOGGER)
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()

        self.input_binding_idx = self.engine.get_binding_index("images")
        self.output_binding_idx = self.engine.get_binding_index("output0")

        self.input_size = trt.volume(self.engine.get_binding_shape(self.input_binding_idx)) * np.dtype(np.float32).itemsize
        self.output_size = trt.volume(self.engine.get_binding_shape(self.output_binding_idx)) * np.dtype(np.float32).itemsize

        self.d_input = cuda.mem_alloc(self.input_size)
        self.d_output = cuda.mem_alloc(self.output_size)

        self.h_input = np.empty(trt.volume(self.engine.get_binding_shape(self.input_binding_idx)), dtype=np.float32)
        self.h_output = np.empty(trt.volume(self.engine.get_binding_shape(self.output_binding_idx)), dtype=np.float32)

        self.bindings = [int(self.d_input), int(self.d_output)]
        print("[YOLOTRT] Engine loaded successfully")

    def preprocess(self, img):
        resized = cv2.resize(img, self.input_shape)
        img_rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        img_rgb = img_rgb.astype(np.float32) / 255.0
        img_rgb = np.transpose(img_rgb, (2, 0, 1))
        return np.expand_dims(img_rgb, axis=0)

    def detect(self, img):
        input_data = self.preprocess(img)
        np.copyto(self.h_input, input_data.ravel())
        cuda.memcpy_htod(self.d_input, self.h_input)
        self.context.execute_v2(self.bindings)
        cuda.memcpy_dtoh(self.h_output, self.d_output)
        return self._parse_outputs(self.h_output)

    def _parse_outputs(self, outputs):
        detections = []
        stride = 7  # x, y, w, h, conf, cls, ...
        for i in range(0, len(outputs), stride):
            conf = outputs[i + 4]
            cls = int(outputs[i + 5])
            if conf > CONF_THRESHOLD:
                detections.append((cls, conf))
        return detections
