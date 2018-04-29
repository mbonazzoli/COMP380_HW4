import SturdyRobot
import MCLStarter
import random
import time

DISTANCE_FROM_WALL = 10
STARTING_POS = 1


def nextToWall(r, wallDist):
    """Determines if Roby is next to the a wall determined from the distance sensor."""
    dist = r.readDistance()
    if(dist > wallDist):
        return "no wall"
    else:
        return "wall"


if __name__ == "__main__":
    doorsWorld = [(0.0, 32.0, "wall"), (32.0, 48.0, "no wall"),
                  (48.0, 93.0, "wall"), (93.0, 109.0, "no wall"), (109.0, 121.0, "wall"),
                  (121.0, 137.0, "no wall"), (137.0, 182.0, "wall"), (182.0, 185.0, "no wall")]
    opposites = {"wall": "no wall", "no wall": "wall"}
    monte = MCLStarter.MonteCarloLocalizer(1000, 0, 185, doorsWorld)

    config = {SturdyRobot.SturdyRobot.LEFT_MOTOR: 'outC',
              SturdyRobot.SturdyRobot.RIGHT_MOTOR: 'outA',
              SturdyRobot.SturdyRobot.ULTRA_SENSOR: 'in1'}
    roby = SturdyRobot.SturdyRobot("roby")

    # Change depending on Roby starting position
    actualLoc = STARTING_POS
    expectedLoc = STARTING_POS
    twoNumsStr = "{0:7.3f}  {1:7.3f}"

    print("------------ Initial location, expected and actual:", twoNumsStr.format(expectedLoc, actualLoc))
    # Start moving roby forward slowly
    roby.forward(speed=0.1)

    while expectedLoc < 180:
        time.sleep(1)
        distMoved = random.gauss(2.0, 0.25)
        print("------------ Movement, expected and actual:", twoNumsStr.format(2.0, distMoved))

        expectedLoc += 2.0
        actualLoc = actualLoc + distMoved
        print("------------ New location, expected and actual:", twoNumsStr.format(expectedLoc, actualLoc))

        actualSensor = monte.getMapValue(actualLoc)
        oppSensor = opposites[actualSensor]

        reportedData = nextToWall(roby, DISTANCE_FROM_WALL)
        print("------------ Sensor value, actual and reported:", actualSensor, reportedData)

        result = monte.mclCycle(2.0, reportedData)
        monte.printPoint(expectedLoc, 'E')
        monte.printPoint(actualLoc, 'A')
        if result is not None:
            monte.printPoint(result, 'C')
            print("MCL Result:", result)
