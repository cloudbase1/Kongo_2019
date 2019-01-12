package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ArcadeDrive;

import edu.wpi.first.wpilibj.command.Subsystem;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * @author EAP
 */

public class DriveTrainPID extends Subsystem {
	double goalL;
	double goalR;
	//Control Modes
	private ControlMode velocity = com.ctre.phoenix.motorcontrol.ControlMode.Velocity;
	
	//Declare all motors variables as TalonSRXs
	public TalonSRX right = new TalonSRX(RobotMap.RIGHT_MOTOR_PORT);
	public TalonSRX left = new TalonSRX(RobotMap.LEFT_MOTOR_PORT);
	
	double leftSpeed;
	double rightSpeed;
	// Conversions from encoder ticks to speed and distance
	// calculations for Kongo ~ 6 inch wheels (0.1524 meters)and 2048 PPR encoder
	// 5.944 inches * 3.14 = 18.664 inches
	// 18.664 inches = 0.474 meters
	// With 2048 PPR encoder in quad mode this is 8192 codes per rev
	// this gives 8192/0.474 = 17282 ticks per meter for Kongo 
	public static final double kTicksPerMeter = 17282;
		
	public DriveTrainPID() {
			
		
		invertLeftSide(RobotMap.LEFT_SIDE_INVERTED);
		invertRightSide(RobotMap.RIGHT_SIDE_INVERTED);
		
		setSensorPhase(!RobotMap.LEFT_SIDE_SENSOR_PHASE_REVERSED, !RobotMap.RIGHT_SIDE_SENSOR_PHASE_REVERSED);
		
		
		left.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,0);
		right.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
			
		right.config_kP(0, RobotMap.DRIVETRAIN_KP, 0);
		right.config_kI(0, RobotMap.DRIVETRAIN_KI, 0);
		right.config_kD(0, RobotMap.DRIVETRAIN_KD, 0);
		right.config_kF(0, RobotMap.DRIVETRAIN_KF, 0);
		right.config_kP(1, RobotMap.DRIVETRAIN_KP, 0);
		right.config_kI(1, RobotMap.DRIVETRAIN_KI, 0);
		right.config_kD(1, RobotMap.DRIVETRAIN_KD, 0);
		right.config_kF(1, RobotMap.DRIVETRAIN_KF, 0);
		// EAP Again we need to give these numbers a meaningful name
		// and place them with useful comments in RobotMap
		right.config_IntegralZone(0, RobotMap.DRIVETRAIN_IZONE, 0);
		right.configMotionCruiseVelocity(RobotMap.MOTION_CRUISE_VELOCITY.intValue(), 0);
		rightSpeed = 0.225;
		right.configMotionAcceleration(3500, 0);
		right.configNominalOutputForward(0, 0);
		right.configNominalOutputReverse(0, 0);
		right.configPeakOutputForward(1, 0);
		right.configPeakOutputReverse(-1, 0);
		//right.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
		//right.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);

			
		left.config_kP(0, RobotMap.DRIVETRAIN_KP, 0);
		left.config_kI(0, RobotMap.DRIVETRAIN_KI, 0);
		left.config_kD(0, RobotMap.DRIVETRAIN_KD, 0);
		left.config_kF(0, RobotMap.DRIVETRAIN_KF, 0);
		left.config_kP(1, RobotMap.DRIVETRAIN_KP, 0);
		left.config_kI(1, RobotMap.DRIVETRAIN_KI, 0);
		left.config_kD(1, RobotMap.DRIVETRAIN_KD, 0);
		left.config_kF(1, RobotMap.DRIVETRAIN_KF, 0);
		left.config_IntegralZone(0, RobotMap.DRIVETRAIN_IZONE, 0);
		left.configMotionCruiseVelocity(RobotMap.MOTION_CRUISE_VELOCITY.intValue(), 0);
		leftSpeed = 0.225;
		left.configMotionAcceleration(3500, 0);
		left.configNominalOutputForward(0, 0);
		left.configNominalOutputReverse(0, 0);
		left.configPeakOutputForward(1, 0);
		left.configPeakOutputReverse(-1, 0);
		//left.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
		//left.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
		
		left.enableCurrentLimit(false);
		right.enableCurrentLimit(false);
	}		
	
	//Set the right and left sides of the robots to speeds based on input speed and rotation
	public void arcadeDrive(double xSpeed, double zRotation) {
		// xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is negative.
		// zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is negative
		
		double leftMotorOutput = 0;
		double rightMotorOutput = 0;
		
		
		
        // EAP this code provides an exponential function for the zRotation. This gives a 
        // an easier to drive when small stick deflections are needed. For example
        // placing a gear on the peg.
		double exponent = 2;
		zRotation = Math.copySign(Math.pow(zRotation, exponent), zRotation);

		double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);
		
		// TODO EAP This is very bad coding. Why do you just swap the zRotation sign 
		// with no explanation? Make this a RobotMap switch 
		// EAP If the robot spins the wrong way switch zRotation polarity
		if(RobotMap.INVERT_ZROTATION) {
		 zRotation = -zRotation;}
		
		// EAP Lets ook at the drive code
		// Drive Straight FW or BW 
		// xSpeed = 1.0/-1.0 zRotation = 0.0 
		// leftMotorOutput = 1.0 rightMotoOutput = 1.0
		// xSpeed = 0.0 zRotation -1.0..1.0
		//----------------------------------------------
		// xSpeed = 0.0 zRotation = 1.0
		// leftMotorOutput = 1.0 rightMotorOutput = -1.0 ie spin CW
		//----------------------------------------------
		// xSpeed = 0.0 zRotation = -1.0
		// leftMotorOutput = -1.0 rightMototOutput = 1.0 ie spin CCW
		//----------------------------------------------
		// xSpeed = 0.5 zRotation = 0.5 circle rather than spin
		// leftMotorOutput = 0.5 rightMtorOutput = 0.0 
		//----------------------------------------------
		// xSpeed = 0.5 zRotation = -0.5
		// leftMotorOutput = 0.0 rightMotorOutput = 0.5
		//----------------------------------------------
		// xSpeed = -0.5 zRotation = 0.5
		// leftMotorOutput = 0.0 rightMotorOutput = 0.5
		//----------------------------------------------
		// xSpeed = -0.5 zRotation = -0.5 
		// leftMotorOutput = 0.5 rightMotorOutput = 0.0
		//----------------------------------------------
		// xSpeed = 0.9 zRotation = 0.1
		// leftMotorOutput = 0.9 rightMtorOutput = 0.8
		// EAP So it seems when abs(xSpeed) = abs(zRotation) it resolves to a spin 
		//----------------------------------------------
		if (xSpeed >= 0.0) { // check y deflection
			// First quadrant, else second quadrant 
			if (zRotation >= 0.0) { // Forward Clockwise
				leftMotorOutput = maxInput;
				rightMotorOutput = xSpeed - zRotation;
			} else { // Forward CCW
				leftMotorOutput = xSpeed + zRotation;
				rightMotorOutput = maxInput;
			}
		} else {// Y is negative
			// Third quadrant, else fourth quadrant
			if (zRotation >= 0.0) {// Reverse CCW
				leftMotorOutput = xSpeed + zRotation;
				rightMotorOutput = maxInput;
			} else { // Reverse CW
				leftMotorOutput = maxInput;
				rightMotorOutput = xSpeed - zRotation;
			}
		}
	    // EAP Use the above calculated drive factors as a scaler for velocity
		// TODO EAP If we don't want PID we simply set the talons to PercentOutput
		// This means the subsystem DriveTrain is never used only DriveTrainPID
		// so really only need 1 so just call DribveTrainPID as DriveTain since
		// it does both PID and non-PID.
		if (RobotMap.DRIVETRAIN_PID){ 
		System.out.println(right.getSensorCollection());
		// EAP Added blocking joystick input while a command is being sent 
		// during teleop. This is to allow sending comands from the smart dashboard
		// I would like to find a more integrated way to do this rather than brute force
	    // approach but this will do for now.
		 if(RobotMap.BLOCK_JOYSTICK_INPUT == false) {
			if(RobotMap.TUNE_VEL_PID) {
				right.set(velocity, RobotMap.TUNE_VEL_SPEED);
				left.set(velocity, RobotMap.TUNE_VEL_SPEED);
			}
			else {
			right.set(velocity, RobotMap.MAX_SPEED * rightMotorOutput);
			left.set(velocity, RobotMap.MAX_SPEED * leftMotorOutput);
			}
		
			System.out.print("Right Speed ");
			System.out.println(RobotMap.MAX_SPEED * rightMotorOutput);
			System.out.print("Left Speed ");
			System.out.println(RobotMap.MAX_SPEED * leftMotorOutput);
		
		}
		else{
			right.set(ControlMode.PercentOutput, rightMotorOutput);
			left.set(ControlMode.PercentOutput, leftMotorOutput);
			System.out.print(rightMotorOutput);
			System.out.println(" ");
			System.out.println(leftMotorOutput);
		}
	}
      }
	
	
	public void limit() {
		right.configPeakCurrentLimit(RobotMap.DRIVETRAIN_PEAK_CURRENT, 0);
		left.configPeakCurrentLimit(RobotMap.DRIVETRAIN_PEAK_CURRENT, 0);
		right.enableCurrentLimit(true);
		left.enableCurrentLimit(true);
	}
	public void unlimit() {
		right.enableCurrentLimit(false);
		left.enableCurrentLimit(false);
	}
	//Set the right and left sides of the robot to speeds based on input speeds in both motor sides.
	public void tankDrive(double leftSpeed, double rightSpeed) { 
			
		leftSpeed = limit(leftSpeed);
		rightSpeed = limit(rightSpeed);
		
		if (RobotMap.DRIVETRAIN_PID){
			right.set(velocity, RobotMap.MAX_SPEED * rightSpeed);
			left.set(velocity, RobotMap.MAX_SPEED * leftSpeed);
		}
		else {
			right.set(ControlMode.PercentOutput, rightSpeed);
			left.set(ControlMode.PercentOutput, leftSpeed);
		}
	}
	public void stop() {
		left.set(ControlMode.PercentOutput, 0);
		right.set(ControlMode.PercentOutput, 0);
	}
	public double avgSpeed(){
		return (getLeftSpeed()+(getRightSpeed()))/2;
	}
	public double lThrottle(){
		return left.getMotorOutputPercent();
	}
	
	public double rThrottle(){
		return right.getMotorOutputPercent();
	}
	
	public void magicDrive (double lMeters, double rMeters){
		// EAP Convert meters to Ticks based on based on number of 
		// encoder ticks per inch.
		double leftRot = -1*kTicksPerMeter*lMeters;
		double rightRot = -1*kTicksPerMeter*rMeters;
		System.out.println(leftRot+" "+rightRot);
		System.out.println("Entering Motion Magic mode");

		left.set(ControlMode.MotionMagic, leftRot);
		System.out.println("Entering Motion Magic mode left");

		right.set(ControlMode.MotionMagic, rightRot);
		System.out.println("Entering Motion Magic mode right");

		goalL = leftRot;
		System.out.print("Goal Left = ");
		System.out.println(leftRot);


		goalR = rightRot;
		System.out.print("Goal Right = ");
		System.out.println(rightRot);
		System.out.println("Exit Motion Magic mode");
	}
		
	public void setToPosition(){
	    left.set(ControlMode.Position, 0);
		right.set(ControlMode.Position, 0);
	}
	// EAP Use turn to angle from 2018 code
	// Try this first but I would like to try DriveMagic
	// I could calculate wheel size and arc meters needed to turn X degrees
	// Just and idea.
	public double goToAngle(double target){
		System.out.println("Turning to " + target);
		double kP = 0.0055;
		double angle = getGyroAngle();
		double error = target-Math.abs(angle);
		System.out.println("Gyro angle is " + angle);
		System.out.println("Gyro Error is " + error);
		error = target - Math.abs(angle);
		angle = Robot.gyro.getAngle();
		left.set(ControlMode.PercentOutput, error*kP);
		right.set(ControlMode.PercentOutput, -error*kP);
		System.out.println("Still Moving to angle");
		return error;
	}
	public String getMode(){
		return left.getControlMode().toString();
	}

	public int getLeftSpeed(){
		return left.getSelectedSensorVelocity(0);
	}
	public int getRightSpeed(){
		return right.getSelectedSensorVelocity(0);
	}
	public int getLeftPos(){
		return left.getSelectedSensorPosition(0);
	}
	public int getRightPos(){
		return right.getSelectedSensorPosition(0);
	}
	public void resetEncoders(){
		left.setSelectedSensorPosition(0, 0, 0);
		right.setSelectedSensorPosition(0, 0, 0);
	}
	
    public void setLeftMotorPower(double power) {
        if (power > 1) {
            if (RobotMap.DRIVETRAIN_DEBUG) {
                System.out.println("[Subsystem] Power to left side of drivetrain was set too high: " + power + ", changing to full forward.");
            }
            power = 1;
        } else if (power < -1) {
            if (RobotMap.DRIVETRAIN_DEBUG) {
                System.out.println("[Subsystem] Power to left side of drivetrain was set too low: " + power + ", changing to full reverse.");
            }
            power = -1;
        }
        left.set(ControlMode.PercentOutput, power);
    }
    
    public void setRightMotorPower(double power) {
        if (power > 1) {
            if (RobotMap.DRIVETRAIN_DEBUG) {
                System.out.println("[Subsystem] Power to right side of drivetrain was set too high: " + power + ", changing to full forward.");
            }
            power = 1;
        } else if (power < -1) {
            if (RobotMap.DRIVETRAIN_DEBUG) {
                System.out.println("[Subsystem] Power to right side of drivetrain was set too low: " + power + ", changing to full reverse.");
            }
            power = -1;
        }
        right.set(ControlMode.PercentOutput, power);
    }
    
	public double getGyroAngle(){
		return Robot.gyro.getAngle();
	}
	public int avgErr(){
		return (int) (((Math.abs(getLeftPos()-goalL))+(Math.abs(getRightPos()-goalR)))/2);
	}
    public boolean nearGoal(){
		boolean yes = Math.abs(avgErr())<250 && Math.abs(avgSpeed()) < 20;
		if (yes) System.out.println("Yes");
		return yes;
	}
    


	public double getLeftError(){
		return left.getClosedLoopError(0);
	}
	public double getRightError(){
		return right.getClosedLoopError(0);
	}
	public double getLeftCurrent(){
		return left.getOutputCurrent();
	}
	public double getRightCurrent(){
		return right.getOutputCurrent();
	}

	//Set value to number between -1 and 1
	protected double limit(double value) {
		if (value > 1.0) {
			return 1.0;
		}
		if (value < -1.0) {
			return -1.0;
		}
		return value;
   }

	// EAP Must have init default command.
	@SuppressWarnings({ "unused" })
	@Override
	protected void initDefaultCommand() {
		// EAP do this to get rid of "Comparing identical" warning
		// Also DRIVE_MODE is not final. This also makes it more readable.
		//if (RobotMap.DRIVE_MODE == 1){
		if (RobotMap.DRIVE_MODE == RobotMap.ARCADE_DRIVE) {
		   setDefaultCommand(new ArcadeDrive());
		}
		else{
			// EAP right now no tank  drive in Kongo.
			//setDefaultCommand(new TankDrive());
		}
	}	

	public void invertLeftSide(boolean invert) {
		left.setInverted(invert);

	}
	public void invertRightSide(boolean invert) {
		right.setInverted(invert);

	}
	public void setSensorPhase(boolean invertLeft, boolean invertRight) {
		left.setSensorPhase(invertLeft);
		right.setSensorPhase(invertRight);
	} 
}
