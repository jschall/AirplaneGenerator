from Airfoil import Airfoil
import numpy as np

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
        return self.root_chord * (1. - (1.-self.taper_ratio)*Z/self.wing_length)

    def applyTransformations(self,Z,points):
        offset = np.array([tan(self.sweep)*Z, tan(self.dihedral)*Z])
        twist = self.washout * Z/self.wing_length
        chord = self.getChordLength(Z)
        return [x*chord+offset for x in polygonutil.rotatePoints(points,twist)]

    def getWingSection(self,Z):
        return self.applyTransformations(Z,self.normSurface)

    def getCamber(self,Z):
        return self.applyTransformations(Z,self.normCamberLine)

    def getChord(self,Z):
        return self.applyTransformations(Z,self.normChordLine)
