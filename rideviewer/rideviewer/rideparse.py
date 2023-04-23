from rideviewer.datatypes import *

import os
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import tkinter
import folium
import webbrowser


class RideParser:

    gpsPosition = []
    computerData = []
    gpsTime = []


    def __init__(self, fileName):
        try:
            fh = open(fileName, "r")
        except IOError as e:
            print("Error: Ride file is invalid")
            exit(0)

        for line in fh:
            if line.startswith("GPS,"):
                gpsD = GPSPosition()
                gpsD.parseData(line)
                self.gpsPosition.append(gpsD)
            if line.startswith("D,"):
                cD = ComputerData()
                cD.parseData(line)
                self.computerData.append(cD)
            if line.startswith("GPST,"):
                gpsTD = GPSTime()
                gpsTD.parseData(line)
                self.gpsTime.append(gpsTD)

        fh.close()


    def plotSpeed(self):
        time = []
        speed = []
        for data in self.computerData:
            time.append(data.rideTime)
            speed.append(data.speed)

        fig, ax = plt.subplots()
        ax.plot(time, speed)

        ax.set(xlabel='Time (S)', ylabel='Speed (MPH)',
            title='Ride Speed vs. Time')
        ax.grid()


    def plotCadence(self):
        time = []
        cadence = []
        for data in self.computerData:
            time.append(data.rideTime)
            cadence.append(data.cadence)

        fig, ax = plt.subplots()
        ax.plot(time, cadence)

        ax.set(xlabel='Time (S)', ylabel='Cadence (RPM)',
            title='Ride Cadence vs. Time')
        ax.grid()


    def plotTemperature(self):
        time = []
        temperature = []
        for data in self.computerData:
            time.append(data.rideTime)
            temperature.append(data.temperature)

        fig, ax = plt.subplots()
        ax.plot(time, temperature)

        ax.set(xlabel='Time (S)', ylabel='Temperature (F)',
            title='Ride Temperature vs. Time')
        ax.grid()


    def plotElevation(self):
        time = []
        elevation = []
        for data in self.computerData:
            time.append(data.rideTime)

        for data in self.gpsPosition:
            elevation.append(data.elevation)

        fig, ax = plt.subplots()
        ax.plot(time, elevation)

        ax.set(xlabel='Time (S)', ylabel='Elevation (Ft)',
            title='Ride Elevation vs. Time')
        ax.grid()


    def showPlots(self):
        plt.show()


    def getMaxSpeed(self):
        maxSpeed = 0
        for data in self.computerData:
            if data.speed > maxSpeed:
                maxSpeed = data.speed

        return maxSpeed


    def getDistance(self):
        return self.computerData[len(self.computerData)-1].distance


    def getAverageSpeed(self):
        return self.computerData[len(self.computerData)-1].avgSpeed


    def getAverageCadence(self):
        return self.computerData[len(self.computerData)-1].avgCadence


    def plotLocation(self):
        curDir = os.getcwd()
        mapFile = curDir + "folium_map.html"

        mapLat = 0.0
        mapLong = 0.0
        ride_coordinates = []

        for data in self.gpsPosition:
            if data.latitude != 0 and data.longitude != 0:
                mapLat = data.latitude
                mapLong = data.longitude

                ride_coordinates.append((data.latitude, data.longitude))

        m = folium.Map(location=[mapLat, mapLong])

        folium.PolyLine(ride_coordinates, "Ride").add_to(m)

        m.save(mapFile)
        webbrowser.open(mapFile)


    def getElevationGain(self):

        firstElevation = False
        elevationCount = 0
        elevationGain = 0
        curElevation = 0
        windowSize = 20

        elevationArr = []
        averagedElevationArr = []

        for data in self.gpsPosition:
            if data.elevation != 0:
              
                elevationArr.append(data.elevation)
 

        index = 0
        while index < (len(elevationArr)-windowSize):
            window = elevationArr[index: index+windowSize]
            
            averagedElevationArr.append(sum(window)/windowSize)

            index += 1


        for elevation in averagedElevationArr:
            if elevation != 0:
              
                if (firstElevation == True):
                    elevationCount += 1

                    if (elevationCount >= 200):
                        curElevation = elevation
                        firstElevation = False

                else:
                    if (elevation > curElevation - 2):
                        elevationGain += (elevation - curElevation)
                        curElevation = elevation
                        
                curElevation = elevation


        return elevationGain


