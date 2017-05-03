from math import sin, cos, pi
from numpy import matrix, array, identity, arange
from control.matlab import *
import time

M = .6  # mass of cart+pendulum
m = .3  # mass of pendulum
Km = 2  # motor torque constant
Kg = .01  # gear ratio
R = 6  # armiture resistance
r = .01  # drive radiu3
K1 = Km*Kg/(R*r)
K2 = Km**2*Kg**2/(R*r**2)
l = .3  # length of pendulum to CG
# l = .7  # length of pendulum to CG
I = 0.006  # inertia of the pendulum
L = (I + m*l**2)/(m*l)
g = 9.81  # gravity
Vsat = 20.  # saturation voltage

A11 = -1 * Km**2*Kg**2 / ((M - m*l/L)*R*r**2)
A12 = -1*g*m*l / (L*(M - m*l/L))
A31 = Km**2*Kg**2 / (M*(L - m*l/M)*R*r**2)
A32 = g/(L-m*l/M)
A = matrix([
    [0, 1, 0, 0],
    [0, A11, A12, 0],
    [0, 0, 0, 1],
    [0, A31, A32, 0]
])

B1 = Km*Kg/((M - m*l/L)*R*r)
B2 = -1*Km*Kg/(M*(L-m*l/M)*R*r)

B = matrix([
    [0],
    [B1],
    [0],
    [B2]
])

Q = 0.25*identity(4)
# Q = matrix([ # positive definite
#     [100, 0, 0, 0],
#     [0, 1, 0, 0],
#     [0, 0, 100, 0],
#     [0, 0, 0, 1]
# ])

(K, X, E) = lqr(A, B, Q, R)
# K = array([[ 0,  -5, -30,  -7]])

def constrain(theta):
    theta = theta % (2*pi)
    if theta > pi:
        theta = -2*pi+theta
    return theta

def sat(Vsat, V):
    if abs(V) > Vsat:
        return Vsat * cmp(V, 0)
    return V

def average(x):
    x_i, k1, k2, k3, k4 = x
    return x_i + (k1 + 2.0*(k3 + k4) +  k2) / 6.0

theta = []
class Pendulum(object):
    def __init__(self, dt, init_conds, end):
        self.dt = dt
        self.t = 0.0
        self.x = init_conds[:]
        self.end = end

    def derivative(self, u):
        V = sat(Vsat, self.control(u))
        #x1 = x, x2 = x_dt, x3 = theta, x4 = theta_dt
        x1, x2, x3, x4 = u
        x1_dt, x3_dt =  x2, x4
        x2_dt = (K1*V - K2*x2 - m*l*g*cos(x3)*sin(x3)/L + m*l*sin(x3)*x4**2) / (M - m*l*cos(x3)**2/L)
        x4_dt = (g*sin(x3) - m*l*x4**2*cos(x3)*sin(x3)/L - cos(x3)*(K1*V + K2*x2)/M) / (L - m*l*cos(x3)**2/M)
        x = [x1_dt, x2_dt, x3_dt, x4_dt]
        return x

    def control(self, u):
        c = constrain(u[2])
        # if c>-pi/5 and c<pi/5:
        return float(-K*matrix(u[0:2]+[c]+[u[3]]).T)
        # else:
        # return self.swing_up(u)

    def swing_up(self, u): # students implement this
        # u[2] = theta, u[3] = dtheta
        E0 = 0.
        k = 1 
        w = (m*g*l/(4*I))**(.5)
        E = m*g*l*(.5*(u[3]/w)**2 + cos(u[2])-1)
        a = k*(E-E0)*cmp(u[3]*cos(u[2]), 0) # this is u in notes
        F = M*a
        V = (F - K2*constrain(u[2]))/K1 # students implement
        return sat(Vsat, V)

    def rk4_step(self, dt):
        dx = self.derivative(self.x)
        k2 = [ dx_i*dt for dx_i in dx ]

        xv = [x_i + delx0_i/2.0 for x_i, delx0_i in zip(self.x, k2)]
        k3 = [ dx_i*dt for dx_i in self.derivative(xv)]

        xv = [x_i + delx1_i/2.0 for x_i,delx1_i in zip(self.x, k3)]
        k4 = [ dx_i*dt for dx_i in self.derivative(xv) ]

        xv = [x_i + delx1_2 for x_i,delx1_2 in zip(self.x, k4)]
        k1 = [self.dt*i for i in self.derivative(xv)]

        self.t += dt
        self.x = map(average, zip(self.x, k1, k2, k3, k4))
        theta.append(constrain(self.x[2]))

    def integrate(self):
        x = []
        while self.t <= self.end:
            self.rk4_step(self.dt)
            x.append([self.t] + self.x)
        return array(x)








# -------------- CALCULATE BASIN OF ATTRACTION --------
# pendulum = Pendulum(
    # .001, # dt
    # [0., 0., 2*pi, 0.], # x, dx, theta, dtheta
    # 10, # end
# )
# data = pendulum.integrate()
# print data[len(data)-1]

import multiprocessing as mp
manager = mp.Manager()
out = mp.Queue()
# boa_dict = manager.dict()
# for ang in [-90, -80, -70, -60, -50, -45, -40, -35, -25, -15, -5, 0, 5, 10, 20, 30, 40, 45, 50, 55, 65, 75, 85, 95]:
    # boa_dict[ang] = {}

# d1 = manager.dict()
d1 = {}
d2 = {}
d3 = {}
d4 = {}

def solve_for_range(boa, theta_start, theta_end):
    theta_range = arange(theta_start, theta_end+10, 10)
    d_theta_range = arange(-30, 30, 1)
    for th in theta_range:
        boa[th] = {}
        for d_th in d_theta_range:
            pendulum = Pendulum(
            .001, # dt
            [0., 0., th*pi/180, d_th], # x, dx, theta, dtheta
            10, # end
            )
            data = pendulum.integrate()
            data_last = data[len(data)-1]
            success = data_last[3] < 0.05 and data_last[4] < 0.05
            boa[th][d_th] = success
            print boa
            print th
            print d_th
            print data_last
    out.put(boa)

processes = [mp.Process(target=solve_for_range, args=(d1, -90, -45))] # -90,-80..-50
processes.append(mp.Process(target=solve_for_range, args=(d2, -45, 0)))
processes.append(mp.Process(target=solve_for_range, args=(d3, 0, 45)))
processes.append(mp.Process(target=solve_for_range, args=(d4, 45, 90)))

for p in processes:
    p.start()

for p in processes:
    p.join()

boa_dict = {}
for d in [out.get() for p in processes]:
    print d
    for k in d.keys():
        boa_dict[k] = d[k]


import matplotlib.pyplot as plt
x_keys = sorted(boa_dict.keys())
print x_keys
boa_plot_x = []
boa_plot_y = []

for th in x_keys:
    for dth in boa_dict[th]:
        if boa_dict[th][dth]:
            th_neg = -360+th if th > 180 else th
            boa_plot_x.append(th_neg)
            boa_plot_y.append(dth)

plt.scatter(boa_plot_x, boa_plot_y)
plt.show()











