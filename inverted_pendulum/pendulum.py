from math import sin, cos, pi
from numpy import matrix, array, identity
from control.matlab import *
import time

M = .6  # mass of cart+pendulum
m = .3  # mass of pendulum
# m = .5  # mass of pendulum
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


(K, X, E) = lqr(A, B, Q, R)
K = array([[ 0,  -5, -30,  -7]])

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
        print 'self.x'
        print self.x
        dx = self.derivative(self.x)
        # print 'dx'
        # print dx
        k2 = [ dx_i*dt for dx_i in dx ]
        print 'k2'
        print k2
        print 'zip'
        print zip(self.x, k2)

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
        # start_time = time.time()
        while self.t <= self.end:
            # print(str((time.time()-start_time))+":"+str(abs(self.t-self.end)))
            self.rk4_step(self.dt)
            x.append([self.t] + self.x)
            # print self.t
        # print("Time elapsed: %d seconds" % (time.time()-start_time))
        return array(x)


