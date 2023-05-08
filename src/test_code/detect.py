import argparse
import shutil
import time
import threading

from pathlib import Path
from sys import platform

from models import *
from utils.datasets import *
from utils.utils import *




obstacle_detected = False

def motor_control(result):
    # code to control the motor based on the result
    global obstacle_detected
    if result == 1:
        #obstacle_detected = True
        #return 1
        print("Far")
        # send command to rotate car
    elif result == 2:
        print("Close")
    else:
        #obstacle_detected = False
        #return 0
        print("straight")


def detectobstacle(
        cfg,
        weights,
        images,
        output='output',  # output folder
        img_size=416,
        conf_thres=0.3,
        nms_thres=0.45,
        save_txt=False,
        save_images=True,
        webcam=True,
        callback=None
):
    device = torch_utils.select_device()
    if os.path.exists(output):
        shutil.rmtree(output)  # delete output folder
    os.makedirs(output)  # make new output folder

    # Initialize model
    model = Darknet(cfg, img_size)

    # Load weights
    if weights.endswith('.pt'):  # pytorch format
        if weights.endswith('yolov3.pt') and not os.path.exists(weights):
            if (platform == 'darwin') or (platform == 'linux'):
                os.system('wget https://storage.googleapis.com/ultralytics/yolov3.pt -O ' + weights)
        model.load_state_dict(torch.load(weights, map_location='cpu')['model'])
    else:  # darknet format
        load_darknet_weights(model, weights)

    model.to(device).eval()

    # Set Dataloader
    if webcam:
        save_images = False
        dataloader = LoadWebcam(img_size=img_size)
        
    else:
        dataloader = LoadImages(images, img_size=img_size)

    # Get classes and colors
    classes = load_classes(parse_data_cfg('cfg/coco.data')['names'])

    for i, (path, img, im0) in enumerate(dataloader):
        t = time.time()
        #if webcam:
        #    print('webcam frame %g: ' % (i + 1), end='')
        #else:
        #    print('image %g/%g %s: ' % (i + 1, len(dataloader), path), end='')
        save_path = str(Path(output) / Path(path).name)

        # Get detections
        img = torch.from_numpy(img).unsqueeze(0).to(device)
        if ONNX_EXPORT:
            torch.onnx.export(model, img, 'weights/model.onnx', verbose=True)
            return
        pred = model(img)
        pred = pred[pred[:, :, 4] > conf_thres]  # remove boxes < threshold

        if len(pred) > 0:
            
            
            # Run NMS on predictions
            detections = non_max_suppression(pred.unsqueeze(0), conf_thres, nms_thres)[0]

            # Rescale boxes from 416 to true image size
            scale_coords(img_size, detections[:, :4], im0.shape).round()

            # Draw bounding boxes and labels of detections
            for x1, y1, x2, y2, conf, cls_conf, cls in detections:
                #if save_txt:  # Write to file
                #    with open(save_path + '.txt', 'a') as file:
                #        file.write('%g %g %g %g %g %g\n' %
                #                   (x1, y1, x2, y2, cls, cls_conf * conf))

                # Add bbox to the image
                #label = plot_one_box([x1, y1, x2, y2], im0)
                #print(label,end=', ')
                if (x2-x1)*(y2-y1) < 10000:
                    motor_control(1)
                    callback()
                else:
                    motor_control(2)
                    callback()
            #obstacle = 1
        else:
            motor_control(0)        

        dt = time.time() - t
        #print('Done. (%.3fs)' % dt)

        if save_images:  # Save generated image with detections
            cv2.imwrite(save_path, im0)

        if webcam:  # Show live webcam
            cv2.imshow(weights, im0)

    if save_images and (platform == 'darwin'):  # linux/macos
        os.system('open ' + output + ' ' + save_path)

# Define a global variable
#detected_object = 0

