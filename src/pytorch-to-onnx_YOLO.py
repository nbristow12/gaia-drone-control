import torch
import onnx
from modules.yolov5.models.experimental import attempt_load
from modules.yolov5.models.common import DetectMultiBackend
from modules.yolov5.utils.torch_utils import select_device
# import torchvision

"""https://pytorch.org/docs/stable/onnx.html
https://learnopencv.com/how-to-convert-a-model-from-pytorch-to-tensorrt-and-speed-up-inference/
"""

def main():

    # modelname = 'raft_small'
    # pt_path = '%s.pth' % modelname
    modelname = 'smoke'
    pt_path = '%s.pt' % modelname
    onnx_path = '%s.onnx' % modelname
    # onnx_path='dummy.onnx'
  
    device='cuda:0'

    device = select_device(device)
    model = DetectMultiBackend(pt_path,device=device,fp16=True)
    print('loading model')
    # model = attempt_load(pt_path,device=device,inplace=True,fuse=True)
    # dummy_input = torch.randn(3, 3, 480, 640).to(device)
    dummy_input = torch.randn(1, 3, 352, 448).to(device)
    # dummy_input = torch.randn(1, 3, 480, 640).to(device)

    # if modelname == 'raft_small':
    #     dummy_input = [dummy_input,dummy_input]
    
    # model.eval()
    model.to(device)
    model.eval()

    for _ in range(2):
        y = model(dummy_input) # dry runs needed

    print('exporting model')
    torch.onnx.export(model,dummy_input,onnx_path,input_names=["images"],output_names=["output"],opset_version=11,export_params=True)

    print('checking onnx model')
    onnx_model = onnx.load(onnx_path)
    onnx.checker.check_model(onnx_model)

    print('Model successfully converted to ONNX, saved to %s' % onnx_path)

if __name__=="__main__":
    main()
