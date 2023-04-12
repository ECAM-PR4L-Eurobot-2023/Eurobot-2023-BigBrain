import math

DISTANCE_EMERGENCY = 70.0  # mm
CENTER_TO_EDGE = 115
FORWARD_ANGLE = 47


class emergencyStopDetector:
    def __init__(self, lidar):
        self.lidar = lidar

    @classmethod
    def detect_emergency_stop():
        distMap = lidar.getMeasurements()
        angleStep = lidar.getAngleStep()
        indexRange = FORWARD_ANGLE/angleStep
        distMap = distMap[-indexRange:]+distMap[indexRange:]
        for index,dist in enumerate(distMap):
            if (dist<(CENTER_TO_EDGE+DISTANCE_EMERGENCY)/math.cos(index*angleStep-FORWARD_ANGLE)):
                return True
