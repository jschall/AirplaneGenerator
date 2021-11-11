from Airfoil import Airfoil
import numpy as np
from math import *
import polygonutil

class WingSectionGenerator:
    def __init__(self, airfoil=Airfoil('clarky-il.csv'), wing_length=200., root_chord=120., washout=radians(1.0), dihedral=radians(1.0), sweep=radians(0.0), taper_ratio=0.8):
        self.airfoil = airfoil
        self.wing_length = wing_length
        self.root_chord = root_chord
        self.washout = washout
        self.dihedral = dihedral
        self.sweep = sweep
        self.taper_ratio = taper_ratio

    def getChordLength(self,Z):
        assert Z >= 0 and Z <= self.wing_length
        return self.root_chord * (1. - (1.-self.taper_ratio)*Z/self.wing_length)

    def applyTransformations(self,Z,points):
        assert Z >= 0 and Z <= self.wing_length
        offset = np.array([tan(self.sweep)*Z, tan(self.dihedral)*Z])
        twist = self.washout * Z/self.wing_length
        chord = self.getChordLength(Z)
        return [x*chord+offset for x in polygonutil.rotatePoints(points,twist)]

    def getAirfoil(self,Z):
        assert Z >= 0 and Z <= self.wing_length
        return self.applyTransformations(Z,self.airfoil.normSurface)

    def getOffsetAirfoil(self,Z,offset):
        assert Z >= 0 and Z <= self.wing_length
        return polygonutil.shrinkPolygon(self.applyTransformations(Z,self.airfoil.normSurface), -offset)

    def getCamber(self,Z):
        assert Z >= 0 and Z <= self.wing_length
        return self.applyTransformations(Z,self.airfoil.normCamberLine)

    def getChord(self,Z):
        assert Z >= 0 and Z <= self.wing_length
        return self.applyTransformations(Z,self.airfoil.normChordLine)

    def getCamberPoint(self,d,Z):
        assert Z >= 0 and Z <= self.wing_length
        return self.applyTransformations(Z,[self.airfoil.getNormCamberPoint(d)])[0]

    def getLineAcrossCamber(self,d,lineAngle,Z):
        assert Z >= 0 and Z <= self.wing_length
        return self.applyTransformations(Z,self.airfoil.getNormLineAcrossCamber(d,lineAngle))

if __name__ == "__main__":
    wing_generator = WingSectionGenerator()
    print(wing_generator.getChordLength(100.))
    print(wing_generator.getAirfoil(100.))
    print(wing_generator.getCamber(100.))
    print(wing_generator.getChord(100.))
