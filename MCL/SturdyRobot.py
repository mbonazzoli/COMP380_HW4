import ev3dev.ev3 as ev3

class SturdyRobot(object):
    # Constants for config Dictionary
    LEFT_MOTOR = 'left-motor'
    RIGHT_MOTOR = 'right-motor'
    SERVO_MOTOR = 'servo-motor'
    LEFT_TOUCH = 'left-touch'
    RIGHT_TOUCH = 'right-touch'
    ULTRA_SENSOR = 'ultra-sensor'
    COLOR_SENSOR = 'color-sensor'
    GYRO_SENSOR = 'gyro-sensor'

    # ---------------------------------------------------------
    #Setup methods for motors and sensors
    def __init__(self, robName, configDict = None):
        self.name = robName
        self.lmot = None
        self.rmot = None
        self.mmot = None
        self.leftTouch = None
        self.rightTouch = None
        self.ultraSensor = None
        self.colorSensor = None
        self.gyroSensor = None
        if configDict is not None:
            self.setupSensorsMotors(configDict)
        if self.lmot is None:
            self.lmot = ev3.LargeMotor('outC')
        if self.rmot is None:
            self.rmot = ev3.LargeMotor('outB')
        self.lmot.stop_action = 'brake'
        self.rmot.stop_action = 'brake'
        if self.mmot is not None:
            self.mmot.stop_action = 'hold'

        self.default = 0.5
        self.maxSpeed = self.lmot.max_speed

    def setupSensorsMotors(self, configs):
        for item in configs:
            port = configs[item]
            if item == self.LEFT_MOTOR:
                self.lmot = ev3.LargeMotor(port)
            elif item == self.RIGHT_MOTOR:
                self.rmot = ev3.LargeMotor(port)
            elif item == self.SERVO_MOTOR:
                self.mmot = ev3.MediumMotor(port)
            elif item == self.LEFT_TOUCH:
                self.leftTouch = ev3.TouchSensor(port)
            elif item == self.RIGHT_TOUCH:
                self.rightTouch = ev3.TouchSensor(port)
            elif item == self.ULTRA_SENSOR:
                self.ultraSensor = ev3.UltrasonicSensor(port)
            elif item == self.COLOR_SENSOR:
                self.colorSensor = ev3.ColorSensor(port)
            elif item == self.GYRO_SENSOR:
                self.gyroSensor = ev3.GyroSensor(port)
            else:
                print("Unknown configuration item:", item)

    def setMotorPort(self, side, port):
        """Takes in which side and which port, and changes the correct variable
        to connect to that port."""
        if side == self.LEFT_MOTOR:
            self.lmot = ev3.LargeMotor(port)
        elif side == self.RIGHT_MOTOR:
            self.rmot = ev3.LargeMotor(port)
        elif side == self.SERVO_MOTOR:
            self.mmot = ev3.MediumMotor(port)
        else:  
            print("Incorrect motor description:", side)

    def setTouchSensor(self, side, port):
        """Takes in which side and which port, and changes the correct
        variable to connect to that port"""
        if side == self.LEFT_TOUCH:
            self.leftTouch = ev3.TouchSensor(port)
        elif side == self.RIGHT_TOUCH:
            self.rightTouch = ev3.TouchSensor(port)
        else:
            print("Incorrect touch sensor description:", side)

    def setColorSensor(self, port):
        """Takes in the port for the color sensor and updates object"""
        self.colorSensor = ev3.ColorSensor(port)

    def setUltrasonicSensor(self, port):
        """Takes in the port for the ultrasonic sensor and updates object"""
        self.ultraSensor = ev3.UltrasonicSensor(port)

    def setGyroSensor(self, port):
        """Takes in the port for the gyro sensor and updates object"""
        self.gyroSensor = ev3.GyroSensor(port)
    # ---------------------------------------------------------
    # methods for reading sensor values

    def readTouch(self):
        """Reports the value of both touch sensors, OR just one if only one is connected, OR
        prints an alert and returns nothing if neither is connected."""
        if self.leftTouch is not None and self.rightTouch is not None:
            return self.leftTouch.is_pressed, self.rightTouch.is_pressed
        elif self.leftTouch is not None:
            return self.leftTouch.is_pressed, None
        elif self.rightTouch is not None:
            return None, self.rightTouch
        else:
            print("Warning, no touch sensor connected")
            return None, None

    def readReflect(self):
    	"""Returns reflected light values 0 (black) - 100 (white)""" 
    	return self.robot.colorSensor.reflected_light_intensity

    def readAmbient(self):
        """Returns percentage of ambient light 0 (dark) - 100 (light)"""
        return self.robot.colorSensor.ambient_light_intensity

    def readColor(self):
    	"""Returns color value 0-7"""
    	return self.robot.colorSensor.color

    def readRGB(self):
    	"""Returns the RGB values (0-1020) read from the colorSensor for more accurate color reading"""
    	return(self.robot.colorSensor.red, self.robot.colorSensor.green, self.robot.colorSensor.blue)

    def readDistance(self):
    	"""Returns distance in centimeters from the ultrasonic sensor reading"""
    	return self.robot.ultraSensor.distance_centimeters

    def readHeading(self):
    	"""Returns the current heading of the robot from the initial gyro zero value (when it was first plugged in) 0-360"""
   
    	return self.robot.gyroSensor.angle % 360

    # ---------------------------------------------------------
    # Methods for motor control

    def forward(self, speed, time=None):
        """Takes in a speed between -1.0 and 1.0 inclusively, and an optional
        time to run (in seconds) and it sets the motors so the robot moves straight forward
        at that speed. This method blocks if a time is specified."""
        assert -1.0 <= speed <= 1.0
        assert self.lmot is not None
        assert self.rmot is not None
        motorSpeed = self.lmot.max_speed * speed
        self.lmot.speed_sp = motorSpeed
        self.rmot.speed_sp = motorSpeed
        self._moveRobot(time)
			
    def backward(self, speed, time=None):
        """Takes in a speed between -1.0 and 1.0 inclusively, and an optional
        time to run (in seconds) and it sets the motors so the robot moves straight forward
        at that speed. This method blocks if a time is specified."""
        assert -1.0 <= speed <= 1.0
        assert self.leftMotor is not None
        assert self.rightMotor is not None
        self.forward(-speed, time)

    def turnLeft(self, speed, time=None):
        """Takes in a speed between -1.0 and 1.0 inclusively, and an optional time
        to run (in seconds) and it sets the motors so the robot turns left in place at
        the given speed. This method blocks if a time is specified until the movement 
        is complete."""
        assert -1.0 <= speed <= 1.0
        assert self.lmot is not None
        assert self.rmot is not None
        motorSpeed = self.lmot.max_speed * speed
        self.lmot.speed_sp = -motorSpeed
        self.rmot.speed_sp = motorSpeed
        self._moveRobot(time)

            
    def turnRight(self, speed, time=None):
        """Takes in a speed between -1.0 and 1.0 inclusively, and an optional time
        to run (in seconds) and it sets the motors so the robot turns right in place at
        the given speed. This method blocks if a time is specified until the movement 
        is complete."""
        assert -1.0 <= speed <= 1.0
        assert self.lmot is not None
        assert self.rmot is not None
        motorSpeed = self.lmot.max_speed * speed
        self.lmot.speed_sp = motorSpeed
        self.rmot.speed_sp = -motorSpeed
        self._moveRobot(time)		
	# def turnLeft(self, speed, time=None):

	# 	""" Turns robot in counterclockwise, negative speed num turns clockwise.
	# 		Only moves right wheel """
	# 	self.rmot.speed_sp = speed*self.maxSpeed
	# 	if time == None:
	# 		self.rmot.run_forever()
	# 	else: 
	# 		self.rmot.run_timed(time_sp = time*1000)
	# 		self.rmot.wait_until_not_moving()

	# def turnRight(self, speed, time=None):
	# 	""" Turns robot in place clockwise, negative speed num turns counterclockwise.
	# 		Only moves left wheel """
	# 	self.lmot.speed_sp = speed*self.maxSpeed
	# 	if time == None:
	# 		self.lmot.run_forever()
	# 	else: 
	# 		self.lmot.run_timed(time_sp = time*1000)
	# 		self.lmot.wait_until_not_moving()
			
    def stop(self):
        """ Turns left and right motors off """
        assert self.lmot is not None
        assert self.rmot is not None
        self.lmot.stop()
        self.rmot.stop()
        print(self.rmot.state)
        self.rmot.wait_until_not_moving()

    def curve(self, leftSpeed, rightSpeed, time=None):
        """ Moves robot in curve based on different left and right motor speeds.
        Speed between -1.0 and +1.0. Time in seconds """
        assert self.lmot is not None
        assert self.rmot is not None
        assert -1.0 <= leftSpeed <= 1.0
        assert -1.0 <= rightSpeed <= 1.0
        lmotSp = leftSpeed * self.lmot.max_speed
        rmotSp = rightSpeed * self.rmot.max_speed
        self.lmot.speed_sp = lmotSp
        self.rmot.speed_sp = rmotSp
        self._moveRobot(time)
        # self.rmot.speed_sp = rightSpeed*self.maxSpeed
        # self.lmot.speed_sp = leftSpeed*self.maxSpeed
        # if time == None:
        # 	self.rmot.run_forever()
        # 	self.lmot.run_forever()
        # else: 
        # 	self.rmot.run_timed(time_sp = time*1000)
        # 	self.lmot.run_timed(time_sp = time*1000)
			
    def wait(self):
        self.rmot.wait_until_not_moving()
        self.lmot.wait_until_not_moving()

    def zeroPointer(self):
        """ Moves pointer/flag back to its original position, heading 0. 
        Speed is designated within method """
        self.mmot.run_to_abs_pos(position_sp=0, speed_sp = 0.1*self.maxSpeed)


    def pointerLeft(self,speed=None, time=None):
        """ Moves pointer/flag counterclockwise, negative speed num turns it clockwise.
        Speed between -1.0 and +1.0. Time in seconds """
        if speed == None:
            self.mmot.speed_sp = -self.default
        else:
            self.mmot.speed_sp = -speed*self.maxSpeed

        if time == None:
            self.run_forever()
        else:
            self.mmot.run_timed(time_sp=time*1000)

    def pointerRight(self, speed=None, time=None):
        """ Moves pointer/flag clockwise, negative speed num turns it counterclockwise.
        Speed between -1.0 and +1.0. Time in seconds """
        if speed == None:
            self.mmot.speed_sp = self.default
        else:
            self.mmot.speed_sp = speed*self.maxSpeed
		
        if time == None:
            self.run_forever()
        else:
            self.mmot.run_timed(time_sp=time*1000)

	
    def pointerTo(self, angle):
        """ Turns pointer/flag to a particular heading. Angle in degrees (0-360) """
        self.mmot.speed_sp = 0.2 * self.maxSpeed
        self.mmot.run_to_rel_pos(position_sp=angle)



    def move(self, translateSpeed, rotateSpeed, time=None):
        """Takes in two speeds, a translational speed in the direction the robot is facing,
        and a rotational speed both between -1.0 and 1.0 inclusively. Also takes in an 
        optional time in seconds for the motors to run.
        It converts the speeds to left and right wheel speeds, and thencalls curve."""
        wheelDist = 12 * 19.5
        assert self.lmot is not None
        assert self.rmot is not None
        assert -1.0 <= translateSpeed <= 1.0
        assert -1.0 <= rotateSpeed <= 1.0
        transMotorSp = translateSpeed * self.lmot.max_speed
        rotMotorSp = rotateSpeed * 2 # Note that technically rotational speed doesn't have the same units...

        # Here are formulas for converting from translate and rotate speeds to left and right
        # These formulas need to know the distance between the two wheels in order to work
        # which I measured to be 12 cm on my robot. But we have to watch out for units here
        # the speeds are in "ticks" (degrees) per second, so we need to map rotational ticks
        # to centimeters. I measured 360 ticks moving the robot 18.5 cm forward, so 1cm is
        # 19.5 tics. Thus the wheel distance is 12 * 19.5 = 234 ticks.
        leftSpeed = transMotorSp - (rotMotorSp * wheelDist) / 2.0
        rightSpeed = transMotorSp + (rotMotorSp * wheelDist) / 2.0
        print("SPEEDS:", leftSpeed, rightSpeed)
        self.lmot.speed_sp = leftSpeed
        self.rmot.speed_sp = rightSpeed
        # self.forward(leftSpeed,rightSpeed)
        self._moveRobot(time)

    def _moveRobot(self, time): 
        """Helper method, takes in a time in seconds, or time is None if no time limit, 
        and it runs the motors at the current speed either forever or for the given time.
        Blocks and waits if a time is given."""
        if time is None:
            self.lmot.run_forever()
            self.rmot.run_forever()
        else:
            milliSecTime = time * 1000.0
            self.lmot.run_timed(time_sp = milliSecTime)
            self.rmot.run_timed(time_sp = milliSecTime)
            self.rmot.wait_until_not_moving()
