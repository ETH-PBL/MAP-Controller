import math
from re import A

__author__ = "Matthias Althoff"
__copyright__ = "TUM Cyber-Physical Systems Group"
__version__ = "2020a"
__maintainer__ = "Gerald Würsching"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Released"


def vehicle_dynamics_st(x, uInit, p, type):
    """
    vehicleDynamics_st - single-track vehicle dynamics
    reference point: center of mass

    Syntax:
        f = vehicleDynamics_st(x,u,p)

    Inputs:
        :param x: vehicle state vector
        :param uInit: vehicle input vector
        :param p: vehicle parameter vector

    Outputs:
        :return f: right-hand side of differential equations

    Author: Matthias Althoff
    Written: 12-January-2017
    Last update: 16-December-2017
                 03-September-2019
    Last revision: 17-November-2020
    """

    #------------- BEGIN CODE --------------

    # set gravity constant
    g = 9.81  #[m/s^2]

    #create equivalent bicycle parameters
    mu = p.mu
    if type == "pacejka":
        B_f = p.C_Pf[0]
        C_f = p.C_Pf[1]
        D_f = p.C_Pf[2]
        E_f = p.C_Pf[3]
        B_r = p.C_Pr[0]
        C_r = p.C_Pr[1]
        D_r = p.C_Pr[2]
        E_r = p.C_Pr[3]
    elif type == "linear":
        C_Sf = p.C_Sf #-p.tire.p_ky1/p.tire.p_dy1  
        C_Sr = p.C_Sr #-p.tire.p_ky1/p.tire.p_dy1  
    lf = p.l_f
    lr = p.l_r
    h = p.h_cg 
    m = p.m 
    I = p.I_z

    #states
    #x0 = x-position in a global coordinate system
    #x1 = y-position in a global coordinate system
    #x2 = steering angle of front wheels
    #x3 = velocity in x-direction
    #x4 = yaw angle
    #x5 = yaw rate
    #x6 = slip angle at vehicle center

    #u1 = steering angle
    #u2 = longitudinal acceleration

    u = uInit

    # system dynamics

    # compute lateral tire slip angles
    alpha_f = -math.atan((x[3] * math.sin(x[6]) + x[5] * lf) / (x[3] * math.cos(x[6]))) + u[0] 
    alpha_r = -math.atan((x[3] * math.sin(x[6]) - x[5] * lr) / (x[3] * math.cos(x[6])))
    if alpha_f > 0.4:
        alpha_f = 0.4
    if alpha_f < -0.4:
        alpha_f = -0.4
    if alpha_r > 0.4:
        alpha_r = 0.4
    if alpha_r < -0.4:
        alpha_r = -0.4

    # compute vertical tire forces
    F_zf = m * (-u[1] * h + g * lr) / (lr + lf)
    F_zr = m * (u[1] * h + g * lf) / (lr + lf)

    # combined slip lateral forces
    if type == "pacejka":
        F_yf = mu * F_zf * D_f * math.sin(C_f * math.atan(B_f * alpha_f - E_f*(B_f * alpha_f - math.atan(B_f * alpha_f))))
        F_yr = mu * F_zr * D_r * math.sin(C_r * math.atan(B_r * alpha_r - E_r*(B_r * alpha_r - math.atan(B_r * alpha_r))))
    elif type == "linear":
        F_yf = mu * F_zf * C_Sf * alpha_f
        F_yr = mu * F_zr * C_Sr * alpha_r

    f = [x[3]*math.cos(x[6] + x[4]), 
        x[3]*math.sin(x[6] + x[4]), 
        0, 
        u[1], 
        x[5]]

    if type == "pacejka":
        f.append(
            1 / I * (F_yf * math.cos(u[0]) * lf - F_yr * lr))
        f.append(
            -x[5] + 1 / (m * x[3]) * (F_yf * math.cos(u[0] - x[6]) \
            + F_yr * math.cos(x[6])))
    elif type == "linear":
        f.append(
            -mu*m/(x[3]*I*(lr+lf))*(lf**2*C_Sf*(g*lr-u[1]*h) + lr**2*C_Sr*(g*lf + u[1]*h))*x[5] \
                +mu*m/(I*(lr+lf))*(lr*C_Sr*(g*lf + u[1]*h) - lf*C_Sf*(g*lr - u[1]*h))*x[6] \
                +mu*m/(I*(lr+lf))*lf*C_Sf*(g*lr - u[1]*h)*u[0]) 
        f.append(
            (mu/(x[3]**2*(lr+lf))*(C_Sr*(g*lf + u[1]*h)*lr - C_Sf*(g*lr - u[1]*h)*lf)-1)*x[5] \
                -mu/(x[3]*(lr+lf))*(C_Sr*(g*lf + u[1]*h) + C_Sf*(g*lr-u[1]*h))*x[6] \
                +mu/(x[3]*(lr+lf))*(C_Sf*(g*lr-u[1]*h))*u[0])
    return f

    #------------- END OF CODE --------------
