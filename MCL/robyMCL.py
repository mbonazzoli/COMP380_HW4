import SturdyRobot
import MCLStarter
import random
import time

# Make sure roby is within 30cm from the wall or update value
DISTANCE_FROM_WALL = 30

# Calculated travel distance of roby's forward movement for 1 second at .05 speed
travel_dist = 2

# initialize with placement of roby 
# (only used in visualization of where roby should be
#  does not interfere with result)
est_dist_traveled_tot = 1 

result = None

def nextToWall(r, wallDist):
    """Determines if Roby is next to the a wall using the data from the distance sensor."""
    dist = r.readDistance()
    if(dist <= wallDist+2):
        return "wall"
    else:
        return "no wall"


if __name__ == "__main__":
    doorsWorld = [(0.0, 32.0, "wall"), (32.0, 48.0, "no wall"),
                  (48.0, 93.0, "wall"), (93.0, 109.0, "no wall"), (109.0, 121.0, "wall"),
                  (121.0, 137.0, "no wall"), (137.0, 182.0, "wall"), (182.0, 185.0, "no wall")]
    opposites = {"wall": "no wall", "no wall": "wall"}
    monte = MCLStarter.MonteCarloLocalizer(500, 0, 185, doorsWorld)

    # Sensor setup and robot initialization
    config = {SturdyRobot.SturdyRobot.LEFT_MOTOR: 'outC',
              SturdyRobot.SturdyRobot.RIGHT_MOTOR: 'outB',
              SturdyRobot.SturdyRobot.ULTRA_SENSOR: 'in1'}
    roby = SturdyRobot.SturdyRobot("roby", config)

    # Main MCL loop
    while est_dist_traveled_tot < 185 or result < 185:
        # Move roby forward ~2cm 
        roby.forward(speed=0.05, time = 1)
        est_dist_traveled_tot += travel_dist
        # Determine whether next to wall or not
        estSensorData = nextToWall(roby, DISTANCE_FROM_WALL)
        print("------------ Sensor value, reported:", estSensorData)
        # perform one cycle of MCL
        result = monte.mclCycle(travel_dist, estSensorData)
        monte.printPoint(est_dist_traveled_tot, "E")
        # Show where the MCL COM is located
        if result is not None:
            monte.printPoint(result, 'C')
            print("MCL Result:", result)

    print("Done")
    roby.beep()
    roby.stop()