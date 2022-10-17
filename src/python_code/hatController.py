import diffcloth_py as diffcloth
import numpy as np
import time, math, random, scipy, utils, common, argparse, torch, os
from pySim.pySim import pySim
from pathlib import Path
import scipy.optimize
import numpy as np
import matplotlib.pyplot as plt
import torch.optim as optim
import torch.nn as nn
import torch.nn.functional
from torch.optim import Adam, LBFGS
from datetime import datetime
from clothNN import Controller, IndClosedController



def getX0A0PairsFromSphericalCoord(xzDegree, yDegree):
    diff = HEAD_CENTER_POS-CLOTH_INIT_POS_CENTER
    dist = np.linalg.norm(np.array([diff[0],  diff[2]]), 2) + 3
    HEAD_CENTER_POS[1] = CLOTH_INIT_POS_CENTER[1]
    xzRad = xzDegree * math.pi  / 180
    xmean_newPoint = utils.getPointOnSphere(dist, xzRad, math.radians(yDegree)) + HEAD_CENTER_POS
    translation = (xmean_newPoint - CLOTH_INIT_POS_CENTER).reshape(1, 3)
    translationMat = np.tile(translation, (vert_num, 1) )
    x0_shifted = common.toTorchTensor(x0_mat + translationMat, False, False)
    a0_shifted =  common.toTorchTensor(CLIP_INIT_POS + np.tile(translation, (1, ndof_u // 3)) , True, False)
    return (x0_shifted, a0_shifted)

# Uniformly sample num initial positions from the range yrangeDegree and xzRangeDegree
def getX0A0PairsFromRange(num = 10, yRangeDegree = (0, 90), xzRangeDegree = (0, 360)):
    (yMin, yMax) = yRangeDegree
    (xzMin, xzMax) = xzRangeDegree
    pairs = []
    for i in range(num):
        xzDegree, yDegree = random.randrange(xzMin, xzMax), random.randrange(yMin, yMax)
        print("Generating {}-th position at (xz, y)=({},{}) deg".format(i, xzDegree, yDegree))
        (x0_shifted, a0_shifted) = getX0A0PairsFromSphericalCoord(xzDegree,  yDegree)
        pairs.append((x0_shifted, a0_shifted, (yDegree, xzDegree)))
    return pairs


def getX0A0DegTuplesUniformlyFromHeight(num = 10, yDegrees = [90]):
    pairs = []
    for yDegree in yDegrees:
        for i in range(0, num):
            xzDegree = i / num * 360.0
            (x0_shifted, a0_shifted) = getX0A0PairsFromSphericalCoord(xzDegree,  yDegree)
            pairs.append((x0_shifted, a0_shifted, (yDegree, xzDegree)))

    return pairs


def lossFunction(xvPairs):
    stretchPenalty = 0
    for (i,(x_i,v_i)) in enumerate(xvPairs):
        x_fixed1 = x_i[attachmentIdx[0]*3:attachmentIdx[0]*3+3]
        x_fixed2 = x_i[attachmentIdx[1]*3:attachmentIdx[1]*3+3]
        clip_dist = torch.linalg.norm(x_fixed2-x_fixed1)
        stretchPenalty += torch.clamp(torch.abs(clip_dist - CLIP_REST_DIST) - 1.0, min=0.0, max=None) * 0.2

    directionPenalty, targetLoss, targetLast = 0, 0, 0
    for (x_last,v_i) in  xvPairs:
        for (i, (idx1, idx2)) in enumerate(CLIP_DIR_VERTEX_PAIR):
            dir = x_last[idx1*3:idx1*3+3]-x_last[idx2*3:idx2*3+3]
            dirGoal = targetShape[idx1*3:idx1*3+3]- targetShape[idx2*3:idx2*3+3]
            dirGoaltorch =  common.toTorchTensor(dirGoal, False, False)
            cosine = torch.clamp(torch.dot(torch.nn.functional.normalize(dir, dim=0), torch.nn.functional.normalize(dirGoaltorch, dim=0)),  max=0.5, min=None)
            directionPenalty += (0.5 - cosine) * 3.0

        targetLoss += torch.nn.functional.smooth_l1_loss(x_last,  targetshape_torch)
    targetLast = torch.nn.functional.smooth_l1_loss(xvPairs[-1][0],  targetshape_torch)
    succeed = targetLast < 1.0

    loss = {'succeed': succeed, 'target' : targetLoss,'stretch' : stretchPenalty, 'direction' : directionPenalty, 'total' : stretchPenalty + targetLoss + directionPenalty}
    return loss

def simulateAndGetLoss(x0a0pairs, render=True):
    simulations = []
    lossTotal = 0.0
    losses = []
    print("[simulationAndGetLoss] totalSeq: {} running simulation...".format(len(x0a0pairs)))
    sim.forwardConvergenceThreshold = 1e-8
    for (i, (x0_shift_torch, a0_shift_torch, _)) in enumerate(x0a0pairs):
        sim.resetSystem()
        xvPairs = common.forwardSimulation(sim, x0_shift_torch.clone(), v0_torch.clone(), a0_shift_torch.clone(), getState, controller, pySim)
        simulations.append(xvPairs)
        if render:
            diffcloth.render(sim, renderPosPairs=True, autoExit=True)
        loss = lossFunction(xvPairs)
        losses.append(loss)
        simulations.append(xvPairs)
        lossTotal += loss['total']
    return lossTotal / len(x0a0pairs), losses, simulations
    
def trainStep(sim, optimizer, trainSeqSampleNum = 20, render=False):
    loss = 0
    X0A0pairs_train = getX0A0PairsFromRange(trainSeqSampleNum)
    loss, _, _ = simulateAndGetLoss(X0A0pairs_train, render=render)
    # Step backward
    optimizer.zero_grad()
    loss.backward()
    nn.utils.clip_grad_norm_(controller.parameters(), 1.0)
    optimizer.step()
    return float(loss)
 
def getValidationLosses(epoch, expName, render = False, saveImage = True, saveSimulation = True):
    sim.forwardConvergenceThreshold = 1e-6
    totalLoss = 0
    success_count = 0
    headPrim = sim.primitives[0]
    headVec = headPrim.getPointVec()
    lossAvg, losses, simulations = simulateAndGetLoss(X0A0pairs_eval, render=render)
    
    for (i, (x0a0Deg, loss, xvPairs)) in enumerate(zip(X0A0pairs_eval, losses, simulations)):
        print('{}/{}: {}'.format(i+1, len(X0A0pairs_eval), 'Succeed' if  loss['succeed'].item() else 'Failed'))
        degrees = x0a0Deg[2]
        success_count += int(loss['succeed'])

        clothVec, clothVecInit = xvPairs[-1][0].detach().numpy(), xvPairs[0][0].detach().numpy()
        vecStack = np.concatenate([clothVec, clothVecInit], axis = 0)
        if saveImage:
            Path(root_path / 'evalImages' / expName).mkdir(parents=True, exist_ok=True)
            identifier = 'epoch-{:d}-deg{:.1f}-{:.1f}-loss-{:.2f}'.format(epoch, degrees[0], round(degrees[1]), loss['total'].item())
            savePath = root_path / 'evalImages' / expName / identifier 
            print("saving image to... {}".format(savePath))

            utils.plotPointCloudFromVecs([vecStack, headVec],  identifier, save=saveImage, path=savePath)
        if saveSimulation:
            savePath = expName + "_eval" + "_{}_{}".format(degrees[0], degrees[1])
            print("saving simulation to output/{}".format(savePath))
            sim.exportCurrentSimulation(savePath)

    sim.forwardConvergenceThreshold = 1e-8
    return lossAvg, success_count

def getState(x, v):
    HEAD_CENTER_POS = common.toTorchTensor(sim.primitives[0].center.copy(), False, False)
    headRadius = 2.1

    state = [x-targetshape_torch]
    v_mean, x_mean = v.reshape(-1, 3).mean(axis=0), x.reshape(-1, 3).mean(axis=0)
    elevationVector = headRadius * torch.nn.functional.normalize(x_mean - HEAD_CENTER_POS, dim=0)
    projectionOnHead = elevationVector + HEAD_CENTER_POS

    state.append(projectionOnHead)
    state.append(elevationVector)
    state.append(v_mean)

    for (i, (idx1, idx2)) in enumerate(CLIP_DIR_VERTEX_PAIR):
        dir = x[idx1*3:idx1*3+3]-x[idx2*3:idx2*3+3]
        state.append(dir)

    return torch.cat(state).float().unsqueeze(0)

def saveEpoch(isBestTrain, isBestVal):
    ckpt = {
            'epoch': epoch,
            'trainMinLoss': trainMinLoss,
            'testMinLoss' : testMinLoss,
            'trainLosses' : trainLosses,
            'testLosses' : testLosses,
            'successLog' : successLog,
            'trainBestEpoch' : trainBestEpoch,
            'testBestEpoch' : testBestEpoch,
            'optimizer_state_dict': optimizer.state_dict(),
            'controller_state_dict': controller.state_dict(),
            }
    if isBestTrain:
        torch.save(ckpt, exp_path / 'trainBestEpoch.pth')
    if isBestVal:
        torch.save(ckpt, exp_path / 'testBestEpoch.pth')
    else:
        torch.save(ckpt, exp_path / '{}.pth'.format(epoch))

def loadCheckpoint(path, epoch):
    global controller, optimizer
    checkpoint = torch.load(path  / epoch)
    controller.load_state_dict(checkpoint['controller_state_dict'])
    optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
    epochStart,testMinLoss, trainMinLoss, trainLosses, testLosses = checkpoint['epoch'], checkpoint['testMinLoss'], checkpoint['trainMinLoss'], checkpoint['trainLosses'], checkpoint['testLosses']
    successLog, trainBestEpoch, testBestEpoch= checkpoint['successLog'], checkpoint['trainBestEpoch'], checkpoint['testBestEpoch']
    print("Loaded checkpoint from epoch {}, trainMinLoss is {}".format(epochStart, trainMinLoss))

    return epochStart,testMinLoss, trainMinLoss, trainLosses, testLosses, successLog, trainBestEpoch, testBestEpoch

args = common.parseInputs()
train_resume, eval_mode = args.train_resume, args.eval

# Experiment
example =  "wear_hat"
if args.eval or args.train_resume:
    expName = args.load_expname
    loadEpoch = args.load_epoch
else: 
    dt_string = common.getTimeString()
    expName = '{}-{}-{}'.format(dt_string, 'Adam', args.randSeed)
    common.setRandomSeed(args.randSeed)

# DiffSimulation Settings
sim = diffcloth.makeSim("wear_hat")
sim.gradientClippingThreshold, sim.gradientClipping = 100.0, False
np.set_printoptions(precision=5)

root_path = Path(__file__).resolve().parent
parent_path = root_path / 'experiments' / example 
exp_path = parent_path/ expName
diffcloth.enableOpenMP(n_threads = 5)
helper = diffcloth.makeOptimizeHelper(example)
# forwardConvergence needs to be reset after helper is made
sim.forwardConvergenceThreshold =  1e-8


sim.resetSystem()
pySim = pySim(sim, helper, True)
state_info_init = sim.getStateInfo()
ndof_u = sim.ndof_u
x0, v0 = state_info_init.x, state_info_init.v
vert_num = x0.shape[0] // 3
x0_mat = x0.reshape((-1, 3))
frame, targetShape = helper.lossInfo.targetFrameShape[0]
CLIP_INIT_POS = np.array(sim.getStateInfo().x_fixedpoints)
CLIP_DIR_VERTEX_PAIR = [(394, 562), (32, 108)]
HEAD_CENTER_POS = sim.primitives[0].center.copy()
CLOTH_INIT_POS_CENTER = x0_mat.mean(axis=0)

x0_torch, v0_torch, a_torch, a0_torch, targetshape_torch, fxiedPointInitDist_torch, CLIP_REST_DIST = common.getTorchVectors(x0, v0, CLIP_INIT_POS, targetShape)
X0A0pairs_eval = getX0A0DegTuplesUniformlyFromHeight(3, [10,30,60])
attachmentIdx = sim.sceneConfig.customAttachmentVertexIdx[0][1]
trainMinLoss, testMinLoss, trainBestEpoch, testBestEpoch, epochStart  = 10000, 10000, 0, 0, 0
controller = IndClosedController(sim, helper, [getState(x0_torch, v0_torch).size(1), 64, 64, ndof_u], dropout=0.0)
controller.reset_parameters(nn.init.calculate_gain('tanh'), 0.001)
optimizer = optim.Adam(controller.parameters(), lr=1e-4 * 2, weight_decay=0)

trainLosses, testLosses, successLog = [], [], []

 
if not(eval_mode):
    # Train
    if not(train_resume):
        exp_path.mkdir(parents=True, exist_ok=True)
        os.system("cp hatController.py {}".format(exp_path / 'wearhat_nn_{}.py'.format(dt_string)))
        os.system("cp utils.py {}".format(exp_path / 'utils_{}.py'.format(dt_string)))
        os.system("cp closedLoop_common.py {}".format(exp_path / 'closedLoop_common_{}.py'.format(dt_string)))
        configFile = open(exp_path / "config.txt", "a")
        configFile.write("randSeed: {}\n".format(args.randSeed))
        configFile.write("optimizer: {}\n".format('Adam'))
        configFile.close()
    else:
        epochStart,testMinLoss, trainMinLoss, trainLosses, testLosses, successLog, trainBestEpoch, testBestEpoch = loadCheckpoint(exp_path, '{}.pth'.format(loadEpoch))
    logFile = open(exp_path / "log.txt", "a")
 
    for epoch in range(epochStart, args.epochNum):
        logFile.write("Epoch {}\n".format(epoch))
        logFile.close()
        logFile = open(exp_path / "log.txt", "a")
        loss = trainStep(sim, optimizer, trainSeqSampleNum=20, render= args.render and (epoch > 0) and (epoch % 10 == 0))
        trainLosses.append(loss)
        if (loss < trainMinLoss):
            trainMinLoss, trainBestEpoch = loss, epoch
            saveEpoch(isBestTrain=True, isBestVal=False)
        utils.log("Train: loss: {} minLoss: {} bestEpoch: {} norm:{} \n".format(loss, trainMinLoss, trainBestEpoch, nn.utils.clip_grad_norm_(controller.parameters(), 1.0)), not(eval_mode), logFile)
        evalLoss, evalSuccessNum = getValidationLosses(epoch, expName , render = (epoch % 10 == 0) and (args.render), saveImage=True, saveSimulation=False)
        testLosses.append(loss)
        successLog.append(evalSuccessNum  * 1.0 / len(X0A0pairs_eval))
        if evalLoss < testMinLoss:
            testMinLoss = evalLoss
            testBestEpoch = epoch
            saveEpoch(isBestTrain=False, isBestVal=True)
        utils.log("Test: loss: {} minLoss: {} bestEpoch:{}, evalSuccessNum:{}/{} rate:{}\n".format(evalLoss, testMinLoss, testBestEpoch, evalSuccessNum,len(X0A0pairs_eval),  evalSuccessNum * 1.0 / len(X0A0pairs_eval)), not(eval_mode), logFile)
        utils.plotLosses(trainLosses, testLosses, exp_path)
        utils.plotCurve(successLog, 'successRate', exp_path)
        saveEpoch(isBestTrain = False, isBestVal=False)
else:
    # Eval
    epochStart,testMinLoss, trainMinLoss, trainLosses, testLosses, successLog, trainBestEpoch, testBestEpoch = loadCheckpoint(exp_path, '{}.pth'.format(loadEpoch))
    evalLoss, evalSuccessNum = getValidationLosses(epochStart, expName, render=args.render, saveImage=True, saveSimulation=True)
    

del sim
