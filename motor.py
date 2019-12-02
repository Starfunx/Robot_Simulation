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

    def __init__(self, Kv, L, R, J, B):
        super(Motor, self).__init__()
        self.Kv = Kv
        self.Kt = Kv
        self.R = R
        self.L = L
        self.J  = J
        self.B  = B

        self.voltage = 0.
        self.current = 0.
        self.angularSpeed = 0.
        self.torque = 0.

        self.load_T = 0.
        self.load_J = 0.

    def update(self, dT):
        E = self.angularSpeed * self.Kv
        self.current += (self.voltage-E - self.R*self.current)/self.L *dT
        self.torque = self.current * self.Kt
        self.angularSpeed += (self.torque - self.angularSpeed*self.B - self.load_T)/(self.J + self.load_J)*dT

    # asscenseurs
    def getVoltage(self):
        return self.voltage

    def getCurrent(self):
        return self.current

    def getSpeed(self):
        return self.angularSpeed

    def getTorque(self):
        return self.torque

    # mutateurs
    def setVoltage(self, V):
        self.voltage = V
    def setAngularVelocity(self, omega):
        self.angularSpeed = omega

    def setLoadTorque(self, torque):
        self.load_T = torque

    def setLoadInertia(self, inertia):
        self.load_J = inertia;


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

    La = 8e-3
    Ra = 14.2

    J  = 1.5e-3
    c  = 5e-3

    load_J = 0.000
    load_torque = np.heaviside(time-0.5,1)*0

    motor = Motor(Km, La, Ra, J, c)
    motor.setLoadInertia(load_J)


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
