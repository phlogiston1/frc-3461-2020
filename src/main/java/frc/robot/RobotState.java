/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.math.Point2d;
import frc.lib.math.PolarPoint2d;
import frc.robot.subsystems.DriveTrain;

/**
 * RobotState calculates positions of the robot and target.
 */
public class RobotState {
    //odometry
    public DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.DRIVETRAIN_TRACKWIDTH);
    public DifferentialDriveOdometry odometry;
    public PigeonIMU pigeon = new PigeonIMU(Constants.PIGEON_IMU);
    public DriveTrain driveTrain;
    double[] ypr = new double[3];
    //vision
    public Limelight camera;
    //color sensor
    I2C.Port i2cPort = I2C.Port.kOnboard;
    public ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    public final ColorMatch colorMatcher = new ColorMatch();
    private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
    public static int currentColor,goalColor = 0;
    public static double IR;
    public static ColorMatchResult match;
    public static String colorString;
    String goalColorString = "Not Received";
    /**
     * initialize RobotState
     * @param dt the drive train for odometry
     * @param l the limelight for vision
     */
    public RobotState(DriveTrain dt, Limelight l){
        driveTrain = dt;
        camera = l;
        //add target colors to color matcher.
        colorMatcher.addColorMatch(kBlueTarget);
        colorMatcher.addColorMatch(kGreenTarget);
        colorMatcher.addColorMatch(kRedTarget);
        colorMatcher.addColorMatch(kYellowTarget);
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    }
    //odometry state
    /**
     * update odometry and color sensor, and check the game specific message to see if the color to spin to has been sent yet.
     */
    public void update(){
        pigeon.getYawPitchRoll(ypr); //update the ypr
        odometry.update(Rotation2d.fromDegrees(getHeading()), driveTrain.lEncoderPosition(), driveTrain.rEncoderPosition());
        wtfIdkWhatImDoing(pigeonAcceleration());
        Color detectedColor = colorSensor.getColor();
        match = colorMatcher.matchClosestColor(detectedColor);
        IR = colorSensor.getIR();
        //check color sensor color
        if (match.color == kBlueTarget) {
            colorString = "Blue";
            currentColor = 1;
        } else if (match.color == kRedTarget) {
            colorString = "Red";
            currentColor = 2;
        } else if (match.color == kGreenTarget) {
            colorString = "Green";
            currentColor = 3;
        } else if (match.color == kYellowTarget) {
            colorString = "Yellow";
            currentColor = 4;
        } else {
            colorString = "Unknown";
            currentColor = 0;
        }
        //get the wheel of fortune color
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0){
    switch (gameData.charAt(0)){
        case 'B' :
            //Blue case code
            goalColor = 1;
            goalColorString = "Blue";
        break;
        case 'G' :
            //Green case code
            goalColor = 2;
            goalColorString = "Green";
            break;
        case 'R' :
            //Red case code
            goalColor = 3;
            goalColorString = "Red";
        break;
        case 'Y' :
            goalColor = 4;
            goalColorString = "Yellow";
            //Yellow case code
        break;
        default :
            //This is corrupt data
        break;
    }
    } else {
    //Code for no data received yet
    }
}
    /**
     * get the current location of the robot
     * @return the current pose in meters
     */
    public Pose2d getCurrentPose(){
        return odometry.getPoseMeters();
    }
    /**
     * reset the odometry of the robot by giving it a new pose that it will treat as it's current location.
     * @param pose the new pose where the robot is
     */
    public void resetOdometry(Pose2d pose){
        driveTrain.resetEncoders();
        odometry.resetPosition(pose,Rotation2d.fromDegrees(getHeading()));
    }
    /**
     * get the average distance of the robot.
     * @return the average encoder distance in ticks
     */
    public double getAverageEncoderDistance(){
        return (driveTrain.lEncoderPosition() + driveTrain.rEncoderPosition()) / 2.0;
    }
    /**
     * zero the robots heading.
     */
    public void zeroHeading(){
        pigeon.setFusedHeading(0);
    }
    /**
     * get the robots yaw from the pigeon.
     * @return the yaw in degrees
     */
    public double getYaw(){
        return ypr[0];
    }
    /**
     * get the robots pitch from the pigeon.
     * @return the pitch in degrees
     */
    public double getPitch(){
        return ypr[1];
    }
    /**
     * get the robots roll from the pigeon.
     * @return the roll in degrees
     */
    public double getRoll(){
        return ypr[2];
    }
    /**
     * get the fused accelerometer and magnetometer heading from the pigeon
     * @return the heading in degrees
     */
    public double getHeading(){
        return Math.IEEEremainder(pigeon.getFusedHeading(), 360.0d) * -1.0d;
    }
    //vision state
    /**
     * get the angle of the target from the camera
     */
    public double targetAngleFromCamera(){
        return camera.getTargetOffsetX();
    }
    /**
     * get the target location from camera as a polar point
     * @return
     */
    public PolarPoint2d visionTargetFromCamera(){
        return new PolarPoint2d(targetDistanceFromCamera(), Rotation2d.fromDegrees(targetAngleFromCamera()));
    }
    /**
     * get the targets distance from the camera. calculated using the height of the camera
     * and goal and the angle of the camera, in the units that the above are specified.
     * @return the target distance
     */
    public double targetDistanceFromCamera(){
        return camera.getTargetDistance();
    }
    /**
     * does the math to get the angle and distance to the inner goal.
     * @return the PolarPoint2d of the inner goal
     */
    public PolarPoint2d innerTargetFromCamera(){
        PolarPoint2d initialPoint = visionTargetFromCamera();
        initialPoint.cartesianTransform(0, Constants.INNER_GOAL_SPACING);
        return initialPoint;
    }
    /**
     * gets the angle to the inner target
     * @return the angle to the inner target as a double
     */
    public double innerTargetAngleFromCamera(){
        return innerTargetFromCamera().getR();
    }
    //vision + cartesian
    /**
     * get the cartesian location of the target
     * @return the Point2d to the outer goal
     */
    public Point2d cartesianTargetCoordinates(){
        return PolarPoint2d.getCartesianPoint(visionTargetFromCamera());
    }
    /**
     * updates odometry using the cartesian target coordinates.
     */
    public void updateOdometryFromVision(){
        resetOdometry(fromPoint2d(cartesianTargetCoordinates(), Rotation2d.fromDegrees(getHeading()))); //actually, this might not be right so TODO
    }
    public Pose2d fromPoint2d(Point2d point, Rotation2d rotation){
        double x = point.getX();
        double y = point.getY();
        return new Pose2d(x,y,rotation);
    }
    //color sensor
    /**
     * get the name of the color sensor detected color
     * @return The name of a color as a string
     */
    public String getCurrentColorString(){
        return colorString;
    }
    /**
     * get the name of the target color
     * @return The name of a color as a string
     */
    public String getGoalColorString(){
        return goalColorString;
    }
    /**
     * get an integer corresponding to the color sensor detected color
     * @return an int from 1 - 4
     */
    public int getCurrentColorInt(){
        return currentColor;
    }
    /**
     * get the acceleration value from the pigeon
     * @return xyz acceleration in a short
     */
    public short[] pigeonAcceleration(){
        short[] xyz = new short[3];
        pigeon.getBiasedAccelerometer(xyz);
        return xyz;
    }
    public static double accelerometerVelocityX = 0;
    public static double accelerometerVelocityY = 0;
    public static double accelerometerVelocityZ = 0;
    public static double accelerometerSuperRoughX = 0;
    public static double accelerometerSuperRoughY = 0;
    public static double accelerometerSuperRoughZ = 0;
    public static Point2d superRoughPoint;
    public static double prevTimestamp = 0;
    public static boolean bumped = false;
    /**
     * acceleration "double integration" (not that I know what that means) to get
     * xy location of robot independent of wheel rotation. maybe use to detect bumps.
     * @param xyz the xyz acceleration from the pigeon
     */
    //super rough approximation...I hope
    public void wtfIdkWhatImDoing(short[] xyz){
        double currentTime = Timer.getFPGATimestamp();
        double dTime = currentTime - prevTimestamp;
        // pigeon acceleration is scaled to 16384 = 1G
        // / 16384 * 9.80665 converts to m/s^2
        double x = xyz[0] / 16384 * 9.80665;
        double y = xyz[1] / 16384 * 9.80665;
        double z = xyz[2] / 16384 * 9.80665;
        //get velocity from acceleration
        accelerometerVelocityX += x*dTime;
        accelerometerVelocityY += y*dTime;
        accelerometerVelocityZ += z*dTime;
        //reuse the old variables to temporarily store the change
        //x/y from velocity
        x = accelerometerVelocityX*(currentTime - prevTimestamp);
        y = accelerometerVelocityY*(currentTime - prevTimestamp);
        z = accelerometerVelocityZ*(currentTime - prevTimestamp);
        //rotates the change so that x isn't always ahead of the robot.
        Point2d transformPoint = new Point2d(x,y);
        transformPoint.rotateBy(Rotation2d.fromDegrees(getHeading()));
        //add the change
        accelerometerSuperRoughX += transformPoint.getX();
        accelerometerSuperRoughY += transformPoint.getY();
        accelerometerSuperRoughZ += accelerometerVelocityZ*(currentTime - prevTimestamp);
        superRoughPoint = new Point2d(accelerometerSuperRoughX, accelerometerSuperRoughY);
        prevTimestamp = currentTime;
    }
    /**
     * get an integer corresponding to the goal color
     * @return an int from 0 - 4 (0 = corrupt data)
     */
    public int getGoalColorInt(){
        return goalColor;
    }
    /**
     * update the values of the shuffleboard/smart dashboard
     */
    public void putShuffleboard(){
        SmartDashboard.putNumber("Robot X", getCurrentPose().getTranslation().getX());
        SmartDashboard.putNumber("Robot Y", getCurrentPose().getTranslation().getY());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("R encoder dist", driveTrain.rEncoderPosition());
        SmartDashboard.putNumber("L encoder dist", driveTrain.lEncoderPosition());
        SmartDashboard.putNumber("pigeon rough x", accelerometerSuperRoughX);
        SmartDashboard.putNumber("pigeon rough y", accelerometerSuperRoughY);
        //SmartDashboard.putString("Current color", getCurrentColorString());
        //SmartDashboard.putString("Goal color", getGoalColorString());
    }
}