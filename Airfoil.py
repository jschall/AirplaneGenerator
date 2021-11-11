import csv
import sys
from math import *
import polygonutil
import re
import numpy as np
from scipy.optimize import minimize

class Airfoil:
    def __init__(self, fname):
        self.readFile(fname)

    def parsePoints(self, instr):
        floatRegex = r'([-+]?[0-9]*\.?[0-9]+)'
        floats = list(map(float,re.findall(floatRegex, instr)))
        ret = []
        for i in range(0, len(floats), 2):
            ret.append(np.array([floats[i], floats[i+1]]))

        return ret


    def readFile(self, fname):
        chordRegex = r'Chord\(mm\),([-+]?[0-9]*\.?[0-9]+)'
        surfaceSectionRegex = r'Airfoil surface,\nX\(mm\),Y\(mm\)\n(([-+]?[0-9]*\.?[0-9]+),([-+]?[0-9]*\.?[0-9]+)\n)+'
        chordSectionRegex = r'Chord line,\nX\(mm\),Y\(mm\)\n(([-+]?[0-9]*\.?[0-9]+),([-+]?[0-9]*\.?[0-9]+)\n)+'
        camberSectionRegex = r'Camber line,\nX\(mm\),Y\(mm\)\n(([-+]?[0-9]*\.?[0-9]+),([-+]?[0-9]*\.?[0-9]+)\n)+'


        with open(fname) as csvfile:
            csvstring = csvfile.read()

        chord=float(re.search(chordRegex, csvstring).group(1))

        # find the surface section
        surfaceSection = re.search(surfaceSectionRegex, csvstring).group(0)

        # get the points
        self.normSurface = [x/chord for x in self.parsePoints(surfaceSection)]

        # find the chord section
        chordSection = re.search(chordSectionRegex, csvstring).group(0)
        self.normChordLine = [x/chord for x in self.parsePoints(chordSection)]

        # find the camber section
        camberSection = re.search(camberSectionRegex, csvstring).group(0)

        # get the points
        self.normCamberLine = [x/chord for x in self.parsePoints(camberSection)]

        # get the camber line length
        self.normCamberLineLength = 0.
        for i in range(0, len(self.normCamberLine)-1):
            secLen = np.linalg.norm(self.normCamberLine[i+1]-self.normCamberLine[i])
            self.normCamberLineLength += secLen

        # center everything around the aerodynamic center
        aeroCenter = polygonutil.traversePolyLine(self.normChordLine,.25)
        self.normSurface = [x-aeroCenter for x in self.normSurface]
        self.normChordLine = [x-aeroCenter for x in self.normChordLine]
        self.normCamberLine = [x-aeroCenter for x in self.normCamberLine]
        self.halfChordPoint = polygonutil.traversePolyLine(self.normChordLine,0.5)

        self.thickestCamberDist = minimize(lambda d: -self.getNormThicknessAtCamberPoint(d), self.normCamberLineLength/2., bounds=((0.,1.),), method='Nelder-Mead').x[0]

    def getNormLineAcrossCamber(self,d,lineAngle=0):
        assert d >= 0 and d <= 1
        lineAngle = lineAngle+pi/2
        camberPoint = polygonutil.traversePolyLine(self.normCamberLine,d)
        camberAngle = polygonutil.getPolyLineDirection(self.normCamberLine,d)

        pt1 = polygonutil.getClosestIntersection(camberPoint, [camberPoint, camberPoint+np.array([cos(lineAngle+camberAngle), sin(lineAngle+camberAngle)])*1000], self.normSurface)
        pt2 = polygonutil.getClosestIntersection(camberPoint, [camberPoint, camberPoint-np.array([cos(lineAngle+camberAngle), sin(lineAngle+camberAngle)])*1000], self.normSurface)
        return (pt1,pt2)

    def getNormThicknessAtCamberPoint(self,d):
        assert d >= 0 and d <= 1
        pt1,pt2 = self.getNormLineAcrossCamber(d,pi/2)

        return np.linalg.norm(pt2-pt1)

    def getNormCamberPoint(self, d):
        assert d >= 0 and d <= 1
        return polygonutil.traversePolyLine(self.normCamberLine,d)

    def getNormCamberAngle(self, d):
        assert d >= 0 and d <= 1
        return polygonutil.getPolyLineDirection(self.normCamberLine,d)

if __name__ == "__main__":
    importer = Airfoil("clarky-il.csv")
    print(importer.getNormLineAcrossCamber(0.5,pi/2))
    print(importer.getNormThicknessAtCamberPoint(importer.thickestCamberDist))

