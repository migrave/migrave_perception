#!/usr/bin/env python3

import os
import torch
import pickle
import yaml
import numpy as np

from FACIL.networks.network import LLL_Net
from FACIL.datasets.exemplars_dataset_ntu import ExemplarsDataset
from FACIL.datasets.ntu_dataset import NTUDataset
from FACIL.datasets import tools
from FACIL.approach.bic import Appr, BiasLayer
from FACIL.loggers.exp_logger import MultiLogger

from CTRGCN.model.ctrgcn import Model

from mas_tools.ros_utils import get_package_path
from mas_tools.file_utils import load_yaml_file


class ActionModel:
    def __init__(self, model_cfg, action_list):
        self.model_cfg = load_yaml_file(model_cfg)
        self.model_path = get_package_path("migrave_action_recognition", "models")
        
        with open(action_list, 'r') as f:
            self.actions = f.read().splitlines()

        self.device = self.init_device()
        self.net, bias_layers = self.load_network()
        exemplars = self.load_exemplars()
        self.model = self.load_model(bias_layers, exemplars)

    def init_device(self):
        if torch.cuda.is_available():
            torch.cuda.set_device(self.model_cfg["gpu"])
            device = "cuda"
        else:
            print("WARNING: [CUDA unavailable] Using CPU instead!")
            device = "cpu"

        return device

    def load_network(self):
        init_model = Model(**self.model_cfg["network_args"])
        init_model.head_var = "fc"

        net = LLL_Net(init_model, remove_existing_head=True)

        pretrained = torch.load(os.path.join(self.model_path, self.model_cfg["model_name"]), map_location=torch.device(self.device))

        # Create Model Heads
        for head in self.model_cfg["heads"]:
            net.add_head(head)

        net.set_state_dict(pretrained["model"])

        if pretrained["bias_layers"] is not None:
            bias_layers = []
            for layer in pretrained["bias_layers"]:
                bias_layer = BiasLayer(self.device)
                bias_layer.load_state_dict(layer)
                bias_layers.append(bias_layer)
        else:
            bias_layers = None

        return net, bias_layers

    def load_exemplars(self):
        with open(os.path.join(self.model_path, self.model_cfg["exemplars_name"]), "rb") as inp:
            exemplars_ds = pickle.load(inp)

        return exemplars_ds

    def load_model(self, bias_layers, exemplars):
        self.model_cfg['model_args']['exemplars_dataset'] = exemplars['exm_ds']
        model = Appr(self.net, self.device, **self.model_cfg['model_args'])
        
        model.model_old = self.net
        model.bias_layers = bias_layers
        model.x_valid_exemplars = exemplars['val_exm_x']
        model.y_valid_exemplars = exemplars['val_exm_y']

        return model

    def classify(self, ske_seq):
        N, T, J = ske_seq.shape
        J = int(J/6)
        x = ske_seq.reshape((N, T, 2, J, 3)).transpose(0, 4, 1, 3, 2)

        valid_frame_num = np.sum(x.sum(0).sum(-1).sum(-1) != 0)
        # reshape Tx(MVC) to CTVM
        x = tools.valid_crop_resize(x[0], valid_frame_num, [0.95], 64)

        with torch.no_grad():
            self.net.eval()
            out = self.net(torch.from_numpy(x[np.newaxis]))
            out = self.model.bias_forward(out)

            pred = torch.cat(out, dim=1).argmax(1)

        return self.actions[pred.data], pred.data

    def train(self, action, trn_data, val_data):
        trn_loader = torch.utils.data.DataLoader(NTUDataset(trn_data, [], **self.model_cfg['train_data_args']),
                                                 batch_size=self.model_cfg['batch_size'], shuffle=True, num_workers=2, pin_memory=False)
        val_loader = torch.utils.data.DataLoader(NTUDataset(val_data, [], **self.model_cfg['test_data_args']),
                                                 batch_size=self.model_cfg['test_batch_size'], shuffle=True, num_workers=2, pin_memory=False)
        
        if action.task_id == -1:
            rospy.loginfo("Training Model...")
            self.net.add_head(action.num_actions)
            self.model.train(self.model_cfg['num_heads'], trn_loader, val_loader, True)
        else:
            rospy.loginfo("Modifying Head#{} of Model...".format(action.task_id))
            self.net.modify_head(action.task_id, self.model_cfg['heads'][action.task_id]+1)
            self.model.train(self.model_cfg['num_heads'], trn_loader, val_loader, False)
          

    def save_model(self, action):
        self.new_model_cfg = load_yaml_file(get_package_path("migrave_action_recognition", "config", "action_model_config.yaml"))
        self.new_model_cfg['heads'].append(action.num_actions)    
        self.new_model_cfg['num_heads'] += 1
        self.new_model_cfg['model_name'] = "model{}.ckpt".format(self.new_model_cfg['num_heads'])
        self.new_model_cfg['exemplars_name'] = "exemplars_model{}.pkl".format(self.new_model_cfg['num_heads'])
        self.actions.extend(action.action_names)
        
        np.savetxt(get_package_path("migrave_action_recognition", "config", "action_list2.txt"), self.actions, fmt="%s")
        
        with open(get_package_path("migrave_action_recognition", "config", "action_model_config2.yaml"), 'w') as cfg_file:
            yaml.dump(self.new_model_cfg, cfg_file, default_flow_style=False)
        
        bias_layers = []
        for layer in self.model.bias_layers:
            bias_layers.append(layer.state_dict())
        
        exm = {'exm_ds': self.model.exemplars_dataset, 'val_exm_x': self.model.x_valid_exemplars, 'val_exm_y': self.model.y_valid_exemplars}
        
        torch.save({'model': self.net.state_dict(), 'bias_layers': bias_layers},
                   os.path.join(self.model_path, self.new_model_cfg['model_name']))
                   
        with open(os.path.join(self.model_path, self.new_model_cfg['exemplars_name']), 'wb') as outp:
            pickle.dump(exm, outp, pickle.HIGHEST_PROTOCOL)
        
        
        
