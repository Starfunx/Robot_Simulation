#coding utf-8
"""
@author N.L
Objet encodeur simulant le comptage des impulsions compté
d'un encodeur.

*parametre : resolution , OCR en tick.rad^-1 de l'encodeur
    (peut aussi etre donnée en tick/(unité de longuer pour 1 tick) etc..)

*read permet de donner le compte depuis le dernier reset
*readAndReset permet de read puis de reset
*à l'initialisation on commence à la pos 0 et à 0 ticks

"""

def main():
    import numpy as np
    import matplotlib.pyplot as plt
    from math import pi
    T = 2*pi #s
    dT = 0.001 #s
    t = np.arange(0, T+dT, dT)
    ticks = np.zeros_like(t)
    ticks2 = np.zeros_like(t)
    angle = np.where(t < pi, t, np.flip(t))
    encoderRes = 30/(2*pi) #ticks.rad^-1

    encoder = Encoder(encoderRes)

    for i in range(len(t)):
        encoder.pos = angle[i]
        ticks[i] = encoder.read()

    encoder.pos = 0.
    for i in range(len(t)):
        encoder.pos = angle[i]
        ticks2[i] = encoder.readAndReset()

    plt.subplot(3, 1, 1)
    plt.plot(t, angle,'r')
    plt.subplot(3,1,2)
    plt.plot(t, ticks)
    plt.subplot(3,1,3)
    plt.plot(t, ticks2)
    plt.show()

from math import floor

class Encoder(object):
    """docstring for Encoder."""

    def __init__(self, resolution):
        super(Encoder, self).__init__()
        self._resolution = resolution

        self._pos = 0.
        self._Ticks = 0
        self._lastPos = 0.

    def updateTicks(self):
        self._Ticks = floor(self.pos*self.resolution) - floor(self._lastPos*self.resolution)

    def read(self):
        """return the ticks counted since last reset"""
        self.updateTicks()
        return self._Ticks

    def readAndReset(self):
        """return the ticks counted since last reset then reset"""
        self.updateTicks()
        ret = self._Ticks
        self._lastPos = self._pos
        return ret

    def pos():
        doc = "The pos property."
        def fget(self):
            return self._pos
        def fset(self, value):
            self._pos = value
        def fdel(self):
            del self._pos
        return locals()
    pos = property(**pos())

    def Ticks():
        doc = "The Ticks property."
        def fget(self):
            return self._Ticks
        def fset(self, value):
            self._Ticks = value
        def fdel(self):
            del self._Ticks
        return locals()
    Ticks = property(**Ticks())

    def resolution():
        doc = "The resolution property."
        def fget(self):
            return self._resolution
        def fset(self, value):
            self._resolution = value
        def fdel(self):
            del self._resolution
        return locals()
    resolution = property(**resolution())

if __name__ == '__main__':
    main()
