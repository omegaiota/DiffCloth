from typing import Any, Optional, Mapping, Tuple

import torch
import torch.autograd as autograd
from torch import Tensor


import diffcloth_py as diffcloth
import numpy as np
from numpy import linalg
import time



from diffcloth_py import Simulation, ForwardInformation


class SimFunction(autograd.Function):

    @staticmethod
    def forward(
            ctx: Any,
            x: Tensor,
            v: Tensor,
            a: Tensor,
            cppSim: diffcloth.Simulation,
            helper: diffcloth.OptimizeHelper
    ) -> Tuple[Tensor, Tensor]:
        # print("Forward: {} {} {}".format(x.shape, v.shape, a.shape))
        ctx.helper = helper
        ctx.simulation = cppSim
        ctx.pastRecord = cppSim.getStateInfo()

        argX = np.float64(x.contiguous().detach().cpu().numpy())
        argV = np.float64(v.contiguous().detach().cpu().numpy())
        argA = np.float64(a.contiguous().detach().cpu().numpy())
        cppSim.stepNN(ctx.pastRecord.stepIdx + 1, argX, argV, argA)

        newRecord = cppSim.getStateInfo()
        ctx.newRecord = newRecord


        x_next = torch.as_tensor(newRecord.x).float()
        v_next = torch.as_tensor(newRecord.v).float()

        ctx.save_for_backward(x, v, a, x_next, v_next)

        return x_next, v_next

    @staticmethod
    def backward(
            ctx: Any,
            dL_dx_next: Tensor,
            dL_dv_next: Tensor
    ) -> Tuple[ None, Tensor, Tensor, Tensor]:
        x, v, a, x_next, v_next = ctx.saved_tensors

        cppSim = ctx.simulation
        dL_dxnew_np = dL_dx_next.contiguous().detach().cpu().numpy()
        dL_dvnew_np = dL_dv_next.contiguous().detach().cpu().numpy()
        isLast = ctx.newRecord.stepIdx == cppSim.sceneConfig.stepNum
        # print("isLast: {} stepping backward for {}: dL/dx:{} dL/dv:{}".format(isLast,
        # ctx.newRecord.stepIdx, linalg.norm(dL_dxnew_np), linalg.norm(dL_dvnew_np)
        # ))

        if isLast:
            backRecord = cppSim.stepBackwardNN(
                ctx.helper.taskInfo,
                np.zeros_like(dL_dxnew_np),
                np.zeros_like(dL_dvnew_np),
                ctx.newRecord,
                ctx.newRecord.stepIdx == 1, # TODO: check if this should be 0 or 1
                dL_dxnew_np,
                dL_dvnew_np)
        else:
            backRecord = cppSim.stepBackwardNN(
                ctx.helper.taskInfo,
                dL_dxnew_np,
                dL_dvnew_np,
                ctx.newRecord,
                ctx.newRecord.stepIdx == 1, # TODO: check if this should be 0 or 1
                np.zeros_like(dL_dxnew_np),
                np.zeros_like(dL_dvnew_np)
            )

        dL_dx = torch.as_tensor(backRecord.dL_dx)
        dL_dv = torch.as_tensor(backRecord.dL_dv)

        dL_da_norm = np.linalg.norm(backRecord.dL_dxfixed)
        if dL_da_norm > 1e-7:
            maxNorm = 4.0
            # print("norm dL_da from {} to {}".format(dL_da_norm, backRecord.dL_dxfixed.shape[0]  * maxNorm))
            normalized = backRecord.dL_dxfixed * (max(min(backRecord.dL_dxfixed.shape[0] * maxNorm, dL_da_norm), 0.05) / dL_da_norm )
            dL_da = normalized
        else:
            dL_da = backRecord.dL_dxfixed
        # print(cppSim.perStepGradient)
        # print("new dL/dx:{} dL/dv:{} dL/da:{}".format(linalg.norm(backRecord.dL_dx), linalg.norm(backRecord.dL_dv), linalg.norm(dL_da)))
        dL_da = torch.as_tensor(dL_da)
        # print("Backward: {} {} {}".format(dL_dx.shape, dL_dv.shape, dL_da.shape))

        return dL_dx, dL_dv, dL_da, None, None
