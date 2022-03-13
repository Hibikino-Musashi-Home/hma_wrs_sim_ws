#!/usr/bin/env python3
import os
import sys
import roslib
import yaml

# sys.path.append(roslib.packages.get_pkg_dir("hma_yolact_pkg") + "/yolact_edge/data")
sys.path.append(roslib.packages.get_pkg_dir("hma_yolact_pkg") + "/yolact_edge")
from data.config import *

ycb_config = yaml.load(open(roslib.packages.get_pkg_dir("hma_yolact_pkg") + '/io/ycb_object_config.yaml'), Loader=yaml.FullLoader)

hma_base = dataset_base.copy({
    "name": "hma base",

    "train_images": "train/",
    "train_info":   "annotations.json",

    "valid_images": "valid/",
    "valid_info":   "annotations.json",

    "has_gt": True
})

task1 = hma_base.copy({
    'name': 'YCBdataset for task1 20210606',

    'class_names': ycb_config["task1"]["class_names"],
    'show_names': ycb_config["task1"]["show_names"]
})

task2b = hma_base.copy({
    'name': 'YCBdataset for task2b 20210227',

    'class_names': ycb_config["task2b"]["class_names"],
    'show_names': ycb_config["task2b"]["show_names"]
})

task1_config = yolact_resnet50_config.copy({
    "name": "task1",

    "dataset": task1,
    "num_classes": len(task1.class_names) + 1,

    "max_iter": 800000,

    "torch2trt_backbone": False,
    "torch2trt_backbone_int8": False,
    "torch2trt_protonet": False,
    "torch2trt_protonet_int8": False,
    "torch2trt_fpn": False,
    "torch2trt_fpn_int8": False,
    "torch2trt_prediction_module": False,
    "torch2trt_prediction_module_int8": False,
})

task2b_config = yolact_resnet50_config.copy({
    "name": "task2b",

    "dataset": task2b,
    "num_classes": len(task2b.class_names) + 1,

    "max_iter": 800000,

    "torch2trt_backbone": False,
    "torch2trt_backbone_int8": False,
    "torch2trt_protonet": False,
    "torch2trt_protonet_int8": False,
    "torch2trt_fpn": False,
    "torch2trt_fpn_int8": False,
    "torch2trt_prediction_module": False,
    "torch2trt_prediction_module_int8": False,
})

def set_hma_cfg(config_name:str):
    global cfg

    cfg.replace(eval(config_name))
    return cfg
