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
    public DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.DRIVETRAIN_TRACKWIDTH);
    public DifferentialDriveOdometry odometry;
    public PigeonIMU pigeon = new PigeonIMU(Constants.PIGEON_IMU);
    public DriveTrain driveTrain;
    public Limelight camera;
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
    String goalColorString = "Not Recieved";
    
    double[] ypr = new double[3];
    public RobotState(DriveTrain dt, Limelight l){
        driveTrain = dt;
        camera = l;
        colorMatcher.addColorMatch(kBlueTarget);
        colorMatcher.addColorMatch(kGreenTarget);
        colorMatcher.addColorMatch(kRedTarget);
        colorMatcher.addColorMatch(kYellowTarget);
    }
    //odometry state
    public void update(){
        pigeon.getYawPitchRoll(ypr); //update the ypr
        odometry.update(Rotation2d.fromDegrees(getHeading()), driveTrain.lEncoderPosition(), driveTrain.rEncoderPosition());
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
    public Pose2d getCurrentPose(){
        return odometry.getPoseMeters();
    }
    public void resetOdometry(Pose2d pose){
        driveTrain.resetEncoders();
        odometry.resetPosition(pose,Rotation2d.fromDegrees(getHeading()));
    }
    public double getAverageEncoderDistance(){
        return (driveTrain.lEncoderPosition() + driveTrain.rEncoderPosition()) / 2.0;
    }
    public void zeroHeading(){
        pigeon.setFusedHeading(0);
    }
    public double getYaw(){
        return ypr[0];
    }
    public double getPitch(){
        return ypr[1];
    }
    public double getRoll(){
        return ypr[2];
    }
    public double getHeading(){
        return pigeon.getFusedHeading();
    }
    //vision state
    public double targetAngleFromCamera(){
        return camera.getTargetOffsetX();
    }
    public PolarPoint2d visionTargetFromCamera(){
        return new PolarPoint2d(targetDistanceFromCamera(), Rotation2d.fromDegrees(targetAngleFromCamera()));
    }
    public double targetDistanceFromCamera(){
        return camera.getTargetDistance();
    }
    public PolarPoint2d innerTargetFromCamera(){
        PolarPoint2d initialPoint = visionTargetFromCamera();
        initialPoint.cartesianTransform(0, Constants.INNER_GOAL_SPACING);
        return initialPoint;
    }
    public double innerTargetAngleFromCamera(){
        return innerTargetFromCamera().getR();
    }
    //vision + cartesian
    public Point2d cartesianTargetCoordinates(){
        return PolarPoint2d.getCartesianPoint(visionTargetFromCamera());
    }
    public void updateOdometryFromVision(){
        resetOdometry(fromPoint2d(cartesianTargetCoordinates(), Rotation2d.fromDegrees(getHeading()))); //actually, this might not be right so TODO
    }
    public Pose2d fromPoint2d(Point2d point, Rotation2d rotation){
        double x = point.getX();
        double y = point.getY();
        return new Pose2d(x,y,rotation);
    }
    //color sensor
    public String getCurrentColorString(){
        return colorString;
    }
    public String getGoalColorString(){
        return goalColorString;
    }
    public int getCurrentColorInt(){
        return currentColor;
    }
    public int getGoalColorInt(){
        return goalColor;
    }

    public void putShuffleboard(){
        SmartDashboard.putNumber("Robot X", getCurrentPose().getTranslation().getX());
        SmartDashboard.putNumber("Robot Y", getCurrentPose().getTranslation().getY());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Current color", getCurrentColorString());
        SmartDashboard.putString("Goal color", getGoalColorString());
    }
}
