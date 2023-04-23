from rideviewer.datatypes import *
from rideviewer.rideparse import RideParser

import os
import argparse


def main():

    ride = RideParser(args.file)

    if args.max_speed:
        print(f"Ride Max Speed: {ride.getMaxSpeed()}")
    if args.avg_speed:
        print(f"Ride Average Speed: {ride.getAverageSpeed()}")
    if args.avg_cadence:
        print(f"Ride Average Cadence: {ride.getAverageCadence()}")
    if args.elevation_gain:
        print(f"Ride Elevation Gain: {ride.getElevationGain()}")
    if args.distance:
        print(f"Ride Distance: {ride.getDistance()}")
    if args.plot_cadence:
        ride.plotCadence()
    if args.plot_elevation:
        ride.plotElevation()
    if args.plot_speed:
        ride.plotSpeed()
    if args.plot_temperature:
        ride.plotTemperature()
    if args.plot_location:
        ride.plotLocation()
    if args.plot_cadence or args.plot_elevation or\
        args.plot_speed or args.plot_temperature:

        ride.showPlots()


if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        prog="RideViewer v1.0",
        description="Program that analyzes open-source bike computer logs"
    )

    parser.add_argument("-f", "--file", required=True)
    parser.add_argument("-ms", "--max_speed", action=argparse.BooleanOptionalAction)
    parser.add_argument("-eg", "--elevation_gain", action=argparse.BooleanOptionalAction)
    parser.add_argument("-as", "--avg_speed", action=argparse.BooleanOptionalAction)
    parser.add_argument("-ac", "--avg_cadence", action=argparse.BooleanOptionalAction)
    parser.add_argument("-pc", "--plot_cadence", action=argparse.BooleanOptionalAction)
    parser.add_argument("-ps", "--plot_speed", action=argparse.BooleanOptionalAction)
    parser.add_argument("-pe", "--plot_elevation", action=argparse.BooleanOptionalAction)
    parser.add_argument("-pt", "--plot_temperature", action=argparse.BooleanOptionalAction)
    parser.add_argument("-pl", "--plot_location", action=argparse.BooleanOptionalAction)
    parser.add_argument("-d", "--distance", action=argparse.BooleanOptionalAction)

    args = parser.parse_args()
    
    main()