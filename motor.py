# coding: utf-8

import numpy as np
import math

class Motor(object):
    """docstring for Motor.
        Kt : motor torque constant
        Kv : back EMF constant
        La : motor inductance
        Ra : motor resistance
        J  : moment of inertia of the rotor
        B  : motor friction
    """

    def __init__(self, Kt, L, R, J, B, Kv):
        super(Motor, self).__init__()
        self.Kt = Kt
        self.Kv = Kv
        self.R = R
        self.L = L
        self.J  = J
        self.B  = B

        self.voltage = 0.
        self.current = 0.
        self.angularSpeed = 0.
        self.Torque = 0.
        self.load_T = 0.
        self.load_J = 0.

    def update(self, dT):
        Kt= self.Kt
        Kv= self.Kv
        R= self.R
        L= self.L
        J=self.J
        B=self.B

        load_T = self.load_T
        load_J = self.load_J
        V = self.voltage
        i = self.current
        Omega = self.angularSpeed

        E = Kv * Omega


        i = i + (V-E-R*i)/L * dT
        torque = i*Kt

        Omega = Omega + (torque - Omega*B - load_T)/(J + load_J)*dT
        self.current = i
        self.Torque = torque
        # self.angularSpeed = Omega

    # mutateurs
    def setAngularVelocity(self, angularVelocity):
        self.angularSpeed = angularVelocity

    def setVoltage(self, V):
        self.voltage = V

    def setLoad(self, load_T):
        self.load_T = load_T

    def setLoadInertia(self, load_J):
        self.load_J = load_J

    # asscenseurs
    def getVoltage(self):
        return self.voltage

    def getCurrent(self):
        return self.current

    def getSpeed(self):
        return self.angularSpeed

    def getTorque(self):
        return self.Torque


def main():
    import matplotlib.pyplot as plt

    dT = 0.0001
    time = np.arange(0,2,dT)

    Km = 12/(8060 * 2*np.pi/60.)
    Kb = 12/(8060 * 2*np.pi/60.)

    La = 1e-3
    Ra = 2.5

    J  = 1e-6
    c  = 1e-7

    load_J = 0.000
    load_torque = np.heaviside(time-0.5,1)*0

    Km = 12/(260 * 2*np.pi/60.)
    Kb = 12/(260 * 2*np.pi/60.)

    La = 8e-3
    Ra = 14.2

    J  = 1.5e-3
    c  = 5e-3

    load_J = 0.000
    load_torque = np.heaviside(time-0.5,1)*0

    motor = Motor(Km, La, Ra, J, c, Kb)
    motor.setLoad(load_J)


    Voltage = np.heaviside(time-0.5,1)*12

    Current = []
    Speed = []
    Torque = []

    for i in range(len(time)):
        t = time[i]
        motor.setVoltage(Voltage[i])
        motor.update(dT)

        Current.append(motor.getCurrent())
        Speed.append(motor.getSpeed())
        Torque.append(motor.getTorque())


    fig, ax1 = plt.subplots()

    ax1.set_xlabel('time (s)')
    ax1.grid(True)

    color = 'tab:blue'
    ax1.set_ylabel('input voltage (V)', color=color)
    ax1.plot(time, Voltage, color=color)
    ax1.plot(time, Current, 'g')
    ax1.plot(time, Torque, 'm')
    ax1.tick_params(axis='y', labelcolor=color)


    ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

    color = 'tab:red'
    ax2.set_ylabel('angular speed (rad.s^-1)', color=color)  # we already handled the x-label with ax1
    ax2.plot(time, Speed, color=color)
    ax2.tick_params(axis='y', labelcolor=color)

    fig.tight_layout()  # otherwise the right y-label is slightly clipped
    plt.show()


if __name__== "__main__":
    main()
