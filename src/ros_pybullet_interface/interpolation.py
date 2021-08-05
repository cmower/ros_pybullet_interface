#!/usr/bin/env python3

import numpy as np

#  spline interpolation
import scipy.interpolate as spintpl
from scipy.spatial.transform import Slerp, RotationSpline
from scipy.spatial.transform import Rotation as R


def interpolateLinearlyQuaternions(timeSeq, QuatSeq, sampleFreq = 100):
    """
    Interpolate trajectory from the knot points of an initial trajectory
    Doing a linear inetrpolation of the quaternions to the given waypoints, while respecting value and timing
    Using scipy Slerp function
    """

    # Create the quaternion interpolator object
    slerp = Slerp(timeSeq, R.from_quat(QuatSeq.T))

    # compute the number of samples, given the duration of the trajectory and desired control frequency
    num_samples = int(np.floor(timeSeq[-1]*sampleFreq))
    # generating the time vector of samples
    intertimeSeq = np.linspace(0, timeSeq[-1], num_samples)
    # generating interpolated trajectory
    interRotSeq = slerp(intertimeSeq)
    interQuatSeq = interRotSeq.as_quat()

    return intertimeSeq, interQuatSeq

def interpolateCubiQuaternions(timeSeq, QuatSeq, sampleFreq = 100):
    """
    Interpolate trajectory from the knot points of an initial trajectory
    Doing a cubic inetrpolation of the quaternions to the given waypoints, while respecting value and timing
    Using scipy RotationSpline function
    """

    # Create the quaternion interpolator object
    quat_spline = RotationSpline(timeSeq, R.from_quat(QuatSeq.T))

    # compute the number of samples, given the duration of the trajectory and desired control frequency
    num_samples = int(np.floor(timeSeq[-1]*sampleFreq))
    # generating the time vector of samples
    intertimeSeq = np.linspace(0, timeSeq[-1], num_samples)
    # generating interpolated trajectory
    interRotSeq = quat_spline(intertimeSeq)
    interQuatSeq = interRotSeq.as_quat()

    return intertimeSeq, interQuatSeq


def interpolateCubicHermiteSpline(timeSeq, PosSeq, dPosSeq, sampleFreq = 100, plotFlag=False, plotTitle="None"):
    """
    Interpolate trajectory from the knot points of an initial trajectory
    Fitting a cubic spline of polynomials to the given waypoints, while respecting both value and derivative of the value
    Using numpy CubicHermiteSpline function
    """

    # ---scipy CubicSpline interpolation method, with node position and velocity
    cubicSplineFunc = spintpl.CubicHermiteSpline(timeSeq, PosSeq, dPosSeq, axis=0, extrapolate=None)

    # compute the number of samples, given the duration of the trajectory and desired control frequency
    num_samples = int(np.floor(timeSeq[-1]*sampleFreq))
    # generating the time vector of samples
    intertimeSeq = np.linspace(0, timeSeq[-1], num_samples)
    # generating interpolated trajectory
    interPosSeq = cubicSplineFunc(intertimeSeq)

    if plotFlag:
        plotInterpolResults(intertimeSeq, interPosSeq, timeSeq, PosSeq, plotTitle)

    return intertimeSeq, interPosSeq



def interpolatePolyfit(timeSeq, PosSeq, polyOrder = 5, sampleFreq = 100, plotFlag=False, plotTitle="None"):
    """
    Interpolate trajectory from the knot points of an initial trajectory
    Fitting an arbitary polynomial of order :polyOrder: to the given waypoints
    Using numpy polyfit function
    """

    # ---python interpolation method
    poly_coefs = np.polyfit(timeSeq, PosSeq, polyOrder)
    # generate the polyOrder polynomial
    polyFunc = np.poly1d(poly_coefs)

    # compute the number of samples, given the duration of the trajectory and desired control frequency
    num_samples = int(np.floor(timeSeq[-1]*sampleFreq))
    # generating the time vector of samples
    intertimeSeq = np.linspace(0, timeSeq[-1], num_samples)
    # generating interpolated trajectory
    interPosSeq = polyFunc(intertimeSeq)

    if plotFlag:
        plotInterpolResults(intertimeSeq, interPosSeq, timeSeq, PosSeq, plotTitle)

    return intertimeSeq, interPosSeq


def interpolateInterp1d(timeSeq, PosSeq, kind='linear', sampleFreq = 100, plotFlag=False, plotTitle="None"):
    """
    Interpolate trajectory from the knot points of an initial trajectory
    Fitting an arbitary polynomial of order:  Using scipy.interpolate.interp1d
    Specifies the kind of interpolation as a string (linear, nearest, zero, slinear, quadratic, cubic,
    previous, next, where zero, slinear, quadratic and cubic refer to a spline interpolation of
    zeroth, first, second or third order;
    """

    # ---python interpolation method
    polyFunc = spintpl.interp1d(timeSeq, PosSeq, kind=kind)


    # compute the number of samples, given the duration of the trajectory and desired control frequency
    num_samples = int(np.floor(timeSeq[-1]*sampleFreq))
    # generating the time vector of samples
    intertimeSeq = np.linspace(0, timeSeq[-1], num_samples)
    # generating interpolated trajectory
    interPosSeq = polyFunc(intertimeSeq)

    if plotFlag:
        plotInterpolResults(intertimeSeq, interPosSeq, timeSeq, PosSeq, plotTitle)

    return intertimeSeq, interPosSeq


def interpolateCubicSpline(timeSeq, PosSeq, sampleFreq = 100, plotFlag=False, plotTitle="None"):
    """
    Interpolate trajectory from the knot points of an initial trajectory
    Fitting an arbitary polynomial of order :polyOrder: to the given waypoints
    Using numpy polyfit function
    """

    # ---python interpolation method
    polyFunc = spintpl.CubicSpline(timeSeq, PosSeq)


    # compute the number of samples, given the duration of the trajectory and desired control frequency
    num_samples = int(np.floor(timeSeq[-1]*sampleFreq))
    # generating the time vector of samples
    intertimeSeq = np.linspace(0, timeSeq[-1], num_samples)
    # generating interpolated trajectory
    interPosSeq = polyFunc(intertimeSeq)

    if plotFlag:
        plotInterpolResults(intertimeSeq, interPosSeq, timeSeq, PosSeq, plotTitle)

    return intertimeSeq, interPosSeq


def interpolateCubicHermiteSplineSourceCode(timeSeq, PosSeq, derPosSeq, sampleFreq, plotFlag=False, plotTitle="PositionVsTime"):
# def interpolateCubicHermiteSplineSourceCode(timeSeq, PosSeq, sampleFreq, plotFlag=False, plotTitle="PositionVsTime"):

    key_points = [[] * 2]
    for nodes in range(0, len(timeSeq)):
        # key_points.append([timeSeq[nodes], PosSeq[nodes]])
        key_points.append([timeSeq[nodes], PosSeq[nodes], derPosSeq[nodes]])

    key_points.sort()
    del key_points[0]

    spline = TCubicHermiteSpline()
    spline.Initialize(key_points, tan_method=spline.FINITE_DIFF, c=0.0)
    # spline.Initialize(key_points)

    # compute the number of samples, given the duration of the trajectory and desired control frequency
    num_samples = int(np.floor(timeSeq[-1] * sampleFreq))
    # generating the time vector of samples
    intertimeSeq = np.linspace(0, timeSeq[-1], num_samples)

    interPosSeq = []
    for nodes in range(0, len(intertimeSeq)):
        t = intertimeSeq[nodes]
        x = spline.Evaluate(t)
        interPosSeq.append(x)

    if plotFlag:
        plotInterpolResults(intertimeSeq, interPosSeq, timeSeq, PosSeq, plotTitle)

    return intertimeSeq, np.array(interPosSeq)


def plotInterpolResults(intertimeSeq, interPosSeq, timeSeq, PosSeq, plotTitle ):
    """plot the interpolated trajectory and the knot points of the initial trajectory"""

    import matplotlib.pyplot as plt

    plt.figure(figsize=(20, 20))
    plt.plot(intertimeSeq, interPosSeq)
    plt.plot(timeSeq, PosSeq, 'b.', marker='*')
    plt.title(plotTitle)
    plt.show()


# Free of dependecies -  Manual interpolation 5th order
def poly5(x_seq, v_seq, a_seq, t_seq, tc):
    """ Manual 5th order interpolation"""
    c0 = x_seq[0]
    c1 = v_seq[0]
    c2 = a_seq[0]/2
    c3 = (20*(x_seq[1]-x_seq[0])-(8*v_seq[1]+12*v_seq[0])*(t_seq[1]-t_seq[0])+(a_seq[1]-3*a_seq[0])*(t_seq[1]-t_seq[0])**2)         /(2*(t_seq[1]-t_seq[0])**3)
    c4 = (-30*(x_seq[1]-x_seq[0])+(14*v_seq[1]+16*v_seq[0])*(t_seq[1]-t_seq[0])-(2*a_seq[1]-3*a_seq[0])*(t_seq[1]-t_seq[0])**2)         /(2*(t_seq[1]-t_seq[0])**4)
    c5 = (12*(x_seq[1]-x_seq[0])-(6*v_seq[1]+6*v_seq[0])*(t_seq[1]-t_seq[0])+(a_seq[1]-a_seq[0])*(t_seq[1]-t_seq[0])**2)         /(2*(t_seq[1]-t_seq[0])**5)

    # print 'c3', c3

    s = c0+c1*(tc-t_seq[0])+c2*(tc-t_seq[0])**2+c3*(tc-t_seq[0])**3+c4*(tc-t_seq[0])**4+c5*(tc-t_seq[0])**5
    v = c1+2*c2*(tc-t_seq[0])+3*c3*(tc-t_seq[0])**2+4*c4*(tc-t_seq[0])**3+5*c5*(tc-t_seq[0])**4
    a = 2*c2+6*c3*(tc-t_seq[0])+12*c4*(tc-t_seq[0])**2+20*c5*(tc-t_seq[0])**3

    return s, v, a
    # return v




# Impact-aware specific
# def getTimesAndAlphas(timeOptVec, alphas, modeIndexs, knotPtselection="mid", HighStiff=20):
#
#     alphaTime = None
#     alphaAllMotion = None
#
#     if knotPtselection=="start":
#
#         alphaTime = [0, timeOptVec[modeIndexs[1]], timeOptVec[modeIndexs[2]], timeOptVec[modeIndexs[3]], timeOptVec[-1]]
#         alphaAllMotion = [HighStiff, alphas[0], alphas[1], HighStiff, 1.25*HighStiff]
#
#     elif knotPtselection=="mid":
#
#         alphaTime = [0,\
#                      (0 + timeOptVec[modeIndexs[1]])/2, \
#                      (timeOptVec[modeIndexs[1]] + timeOptVec[modeIndexs[2]])/2, \
#                      (timeOptVec[modeIndexs[2]] + timeOptVec[modeIndexs[3]])/2, \
#                      (timeOptVec[modeIndexs[3]] + timeOptVec[-1])/2,\
#                      timeOptVec[-1]]
#
#         alphaAllMotion = [ 1.25*HighStiff, HighStiff, alphas[0], alphas[1], HighStiff,  1.25*HighStiff]
#
#
#     elif knotPtselection=="end":
#
#         alphaTime = [0, timeOptVec[modeIndexs[1]], timeOptVec[modeIndexs[2]], timeOptVec[modeIndexs[3]], timeOptVec[-1]]
#         alphaAllMotion = [ 1.25*HighStiff, HighStiff, alphas[0], alphas[1], HighStiff]
#
#     return alphaTime, alphaAllMotion




# ------------------------------------------------------------------------------
# Free of dependecies -  Manual interpolation HermiteSpline
# ------------------------------------------------------------------------------
#


#Generate a cubic Manually Hermite spline from a key points, similar functionality as scipy.cubicHermiteSpline
#Key points: [[t0,x0],[t1,x1],[t2,x2],...].
class TCubicHermiteSpline:
  class TKeyPoint:
    T= 0.0  #Input
    X= 0.0  #Output
    M= 0.0  #Gradient
    def __str__(self):
      return '['+str(self.T)+', '+str(self.X)+', '+str(self.M)+']'

  class TParam: pass

  def __init__(self):
    self.idx_prev= 0
    self.Param= self.TParam()

  def FindIdx(self, t, idx_prev=0):
    idx= idx_prev
    if idx>=len(self.KeyPts): idx= len(self.KeyPts)-1
    while idx+1<len(self.KeyPts) and t>self.KeyPts[idx+1].T:  idx+=1
    while idx>=0 and t<self.KeyPts[idx].T:  idx-=1
    return idx

  #Return interpolated value at t
  def Evaluate(self, t):
    idx= self.FindIdx(t,self.idx_prev)
    if abs(t-self.KeyPts[-1].T)<1.0e-6:  idx= len(self.KeyPts)-2
    if idx<0 or idx>=len(self.KeyPts)-1:
      print('WARNING: Given t= %f is out of the key points (index: %i)' % (t,idx))
      if idx<0:
        idx= 0
        t= self.KeyPts[0].T
      else:
        idx= len(self.KeyPts)-2
        t= self.KeyPts[-1].T

    h00= lambda t: t*t*(2.0*t-3.0)+1.0
    h10= lambda t: t*(t*(t-2.0)+1.0)
    h01= lambda t: t*t*(-2.0*t+3.0)
    h11= lambda t: t*t*(t-1.0)

    self.idx_prev= idx
    p0= self.KeyPts[idx]
    p1= self.KeyPts[idx+1]
    tr= (t-p0.T) / (p1.T-p0.T)
    return h00(tr)*p0.X + h10(tr)*(p1.T-p0.T)*p0.M + h01(tr)*p1.X + h11(tr)*(p1.T-p0.T)*p1.M

  #Compute a phase information (n, tp) for a cyclic spline curve.
  #n:  n-th occurrence of the base wave
  #tp: phase (time in the base wave)
  def PhaseInfo(self, t):
    t0= self.KeyPts[0].T
    te= self.KeyPts[-1].T
    T= te-t0
    mod= Mod(t-t0,T)
    tp= t0+mod  #Phase
    n= (t-t0-mod)/T
    return n, tp

  #Return interpolated value at t (cyclic version).
  #pi: Phase information.
  def EvaluateC(self, t, pi=None):
    if pi==None:
      n, tp= self.PhaseInfo(t)
    else:
      n, tp= pi
    return self.Evaluate(tp) + n*(self.KeyPts[-1].X - self.KeyPts[0].X)

  #data= [[t0,x0],[t1,x1],[t2,x2],...]
  FINITE_DIFF=0  #Tangent method: finite difference method
  CARDINAL=1  #Tangent method: Cardinal spline (c is used)
  ZERO= 0  #End tangent: zero
  GRAD= 1  #End tangent: gradient (m is used)
  CYCLIC= 2  #End tangent: treating data as cyclic (KeyPts[-1] and KeyPts[0] are considered as an identical point)
  def Initialize(self, data, tan_method=CARDINAL, end_tan=GRAD, c=0.0, m=1.0):
    if data != None:
      self.KeyPts= [self.TKeyPoint() for i in range(len(data))]
      for idx in range(len(data)):
        self.KeyPts[idx].T= data[idx][0]
        self.KeyPts[idx].X= data[idx][1]

    #Store parameters for future use / remind parameters if not given
    if tan_method==None:
        tan_method= self.Param.TanMethod
    else:
        self.Param.TanMethod= tan_method
    if end_tan==None:
        end_tan= self.Param.EndTan
    else:
        self.Param.EndTan= end_tan
    if c==None:
        c= self.Param.C
    else:
        self.Param.C= c
    if m==None:
        c= self.Param.M
    else:
        self.Param.M= m

    # grad= lambda idx1,idx2: (self.KeyPts[idx2].X-self.KeyPts[idx1].X)/(self.KeyPts[idx2].T-self.KeyPts[idx1].T)
    # print(grad)

    grad= lambda idx1,idx2: (data[idx1][2] + data[idx2][2])/2

    # grad= lambda idx1,idx2: data[idx1][2]

    if tan_method == self.FINITE_DIFF:
      for idx in range(1,len(self.KeyPts)-1):
        self.KeyPts[idx].M= 0.5*grad(idx,idx+1) + 0.5*grad(idx-1,idx)
        # self.KeyPts[idx].M= grad(idx,idx+1)

    elif tan_method == self.CARDINAL:
      for idx in range(1,len(self.KeyPts)-1):
        self.KeyPts[idx].M= (1.0-c)*grad(idx-1,idx+1)

    if end_tan == self.ZERO:
      self.KeyPts[0].M= 0.0
      self.KeyPts[-1].M= 0.0
    elif end_tan == self.GRAD:
      self.KeyPts[0].M= m*grad(0,1)
      self.KeyPts[-1].M= m*grad(-2,-1)
    elif end_tan == self.CYCLIC:
      if tan_method == self.FINITE_DIFF:
        grad_p1= grad(0,1)
        grad_n1= grad(-2,-1)
        M= 0.5*grad_p1 + 0.5*grad_n1
        self.KeyPts[0].M= M
        self.KeyPts[-1].M= M
      elif tan_method == self.CARDINAL:
        T= self.KeyPts[-1].T - self.KeyPts[0].T
        X= self.KeyPts[-1].X - self.KeyPts[0].X
        grad_2= (X+self.KeyPts[1].X-self.KeyPts[-2].X)/(T+self.KeyPts[1].T-self.KeyPts[-2].T)
        M= (1.0-c)*grad_2
        self.KeyPts[0].M= M
        self.KeyPts[-1].M= M

  def Update(self):
    self.Initialize(data=None, tan_method=None, end_tan=None, c=None, m=None)
