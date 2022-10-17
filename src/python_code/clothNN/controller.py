import math
from collections import deque

import numpy as np

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.autograd as autograd

from torch import Tensor


__all__ = [
    'Controller',
    'ClosedController',
    'IndClosedController']



class LinearBlock(nn.Module):
    def __init__(self, in_features, out_features):
        super().__init__()
        self.linear = nn.Linear(in_features, out_features, bias=True)
        self.nonlinearity = nn.Tanh()

    def forward(self, x):
        out = self.linear(x)
        out = self.nonlinearity(out)
        return out


class Controller(nn.Module):
    def __init__(self, cppSim, optimizeHelper):
        super().__init__()
        self.cppSim = cppSim
        self.helper = optimizeHelper





class ClosedController(Controller):
    def __init__(self, cppSim, optimizeHelper,  widths, dropout=0.0):
        super().__init__(cppSim, optimizeHelper)
        self.layers = nn.ModuleList()
        for i in range(len(widths) - 1):
            in_feature, out_features = widths[i], widths[i + 1]
            if i < len(widths) - 2:
                self.layers.append(LinearBlock(
                    in_feature, out_features))
            else:
                if dropout > 0.0:
                    self.layers.append(nn.Dropout(p=dropout))
                self.layers.append(nn.Linear(widths[i], widths[i + 1], bias=True))

    def reset_parameters(self, gain=1.0, last_w=1.0):
        modules = list(self.modules())
        for i, m in enumerate(modules):
            if isinstance(m, nn.Linear):
                if i == len(modules) - 1:
                    nn.init.orthogonal_(m.weight, gain * last_w)
                else:
                    nn.init.orthogonal_(m.weight, gain)
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)

    def forward(self, x, prev_a):
        raise NotImplementedError



class IndClosedController(ClosedController):
    def __init__(self, cppSim, optimizeHelper, widths, dropout=0.0):
        super().__init__(cppSim, optimizeHelper, widths, dropout)
        ndof_u = cppSim.ndof_u
        self.layers[-1] = nn.Linear(self.layers[-1].in_features, self.layers[-1].out_features, bias=True)

    def forward(self, x) -> torch.Tensor:
        for layer in self.layers:
            x = layer(x)
        a = x
        return a
