class ComputerData:
    rideTime: int = 0
    speed: float = 0
    cadence: float = 0
    avgSpeed: float = 0
    avgCadence: float = 0
    temperature: float = 0
    elevation: float = 0
    elevationGain: float = 0
    distance: float = 0

    def parseData(self, dataLine):
        dSplit = dataLine.split(',')

        self.rideTime = float(dSplit[1])
        self.speed = float(dSplit[2])
        self.cadence = float(dSplit[3])
        self.avgSpeed = float(dSplit[4])
        self.avgCadence = float(dSplit[5])
        self.temperature = float(dSplit[6])
        self.elevation = float(dSplit[7])
        self.elevationGain = float(dSplit[8])
        self.distance = float(dSplit[9])


class GPSTime:
    hours: int = 0
    minutes: int = 0
    seconds: int = 0

    def parseData(self, dataLine):
        dSplit = dataLine.split(',')

        self.hours = int(dSplit[1])
        self.minutes = int(dSplit[2])
        self.seconds = int(dSplit[3])


class GPSPosition:
    latitude: float = 0
    longitude: float = 0
    elevation: float = 0

    def parseData(self, dataLine):
        dSplit = dataLine.split(',')

        self.latitude = float(dSplit[1])
        self.longitude = float(dSplit[2])
        self.elevation = float(dSplit[3])
