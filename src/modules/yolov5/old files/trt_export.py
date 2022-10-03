import torch
import torch.nn as nn
from models.experimental import attempt_load
import torch_tensorrt as trt

weights = "/media/ffil/XavierSSD/home/ffil/gaia-feedback-control/src/GAIA-drone-control/src/modules/yolov5/yolov5s.pt"

model = attempt_load(weights)

print(model)

model.eval()

print('model in eval mode')
trt_model = trt.compile(model, 
    inputs= [trt.Input((1, 3, 448, 448))],
    enabled_precisions= { trt.dtype.half} # Run with FP16
)

# class Ensemble(nn.ModuleList):
#     # Ensemble of models
#     def __init__(self):
#         super().__init__()

#     def forward(self, x, augment=False, profile=False, visualize=False):
#         y = []
#         for module in self:
#             y.append(module(x, augment, profile, visualize)[0])
#         # y = torch.stack(y).max(0)[0]  # max ensemble
#         # y = torch.stack(y).mean(0)  # mean ensemble
#         y = torch.cat(y, 1)  # nms ensemble
#         return y, None  # inference, train output

# model = Ensemble()
# chkpt = torch.load(weights)

# print('done')