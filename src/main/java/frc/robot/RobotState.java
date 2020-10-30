/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Point2d;
import frc.lib.math.PolarPoint2d;
import frc.robot.subsystems.DriveTrain;
import frc.lib.Limelight;
import frc.lib.debloating.ColorSensor;
import frc.lib.debloating.Pigeon;

/**
 * RobotState calculates positions of the robot and target.
 */
public class RobotState {
    //odometry
    public DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.DRIVETRAIN_TRACKWIDTH);
    public DifferentialDriveOdometry odometry;
    public Pigeon pigeon = new Pigeon(new PigeonIMU(Constants.PIGEON_IMU));
    public DriveTrain driveTrain;
    //vision
    public Limelight camera;
    //color sensor
    ColorSensor colorSensor = new ColorSensor();
    /**
     * initialize RobotState
     * @param dt the drive train for odometry
     * @param l the limelight for vision
     */
    public RobotState(DriveTrain dt, Limelight l){
        driveTrain = dt;
        camera = l;
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    }
    //odometry state
    /**
     * update odometry and color sensor, and check the game specific message to see if the color to spin to has been sent yet.
     */
    public void update(){
        colorSensor.update();
        pigeon.update(getHeading());
        odometry.update(Rotation2d.fromDegrees(getHeading()), driveTrain.lEncoderPosition(), driveTrain.rEncoderPosition());
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
        pigeon.zeroHeading();
    }
    /**
     * get the robots yaw from the pigeon.
     * @return the yaw in degrees
     */
    public double getYaw(){
        return pigeon.getYaw();
    }
    /**
     * get the robots pitch from the pigeon.
     * @return the pitch in degrees
     */
    public double getPitch(){
        return pigeon.getPitch();
    }
    /**
     * get the robots roll from the pigeon.
     * @return the roll in degrees
     */
    public double getRoll(){
        return pigeon.getRoll();
    }
    /**
     * get the fused accelerometer and magnetometer heading from the pigeon
     * @return the heading in degrees
     */
    public double getHeading(){
        return pigeon.getHeading();
    }
    //vision state
    /**
     * get the angle of the target from the camera
     */
    public double targetAngleFromCamera(){
        return camera.getTargetOffsetX();
    }
    /**
     * get the angle of the target taking into account the angle of the turret.
     * @return the angle in degrees as a double
     */
    public double targetAngleFromRobot(){
        return camera.getTargetOffsetX() - RobotContainer.getInstance().getTurret().getPosition();
    }
    /**
     * get the target location from camera as a polar point
     * @return
     */
    public PolarPoint2d visionTargetFromCamera(){
        return new PolarPoint2d(targetDistanceFromCamera(), Rotation2d.fromDegrees(targetAngleFromRobot()));
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
    public PolarPoint2d getTargetFromOdometry(){
        Translation2d robotTranslation = odometry.getPoseMeters().getTranslation();
        PolarPoint2d error = Point2d.getPolarPoint(new Point2d(robotTranslation.getX(), robotTranslation.getY()));
        error.transformBy(0, Rotation2d.fromDegrees(-pigeon.getHeading()));
        SmartDashboard.putNumber("error from target odometry", error.getP());
        return error;
    }
    /**
     * updates odometry using the cartesian target coordinates.
     */
    public void updateOdometryFromVision(){
        //pigeon.update(RobotContainer.getInstance().getTurret().getPosition()); //FIXME needs some work 2 get right
        resetOdometry(fromPoint2d(cartesianTargetCoordinates(), Rotation2d.fromDegrees(getHeading())));// + RobotContainer.getInstance().getTurret().getPosition()))); //actually, this might not be right so TODO
    }
    /**
     * get a pose2d from a point2d and a rotation
     * @param point Point2d xy coordinates
     * @param rotation Rotation2d rotation
     * @return Pose2d combination
     */
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
        return colorSensor.getCurrentColorString();
    }
    /**
     * get the name of the target color
     * @return The name of a color as a string
     */
    public String getGoalColorString(){
        return colorSensor.getGoalColorString();
    }
    /**
     * get an integer corresponding to the color sensor detected color
     * @return an int from 1 - 4
     */
    public int getCurrentColorInt(){
        return colorSensor.getCurrentColorInt();
    }
    /**
     * get an integer corresponding to the goal color
     * @return an int from 0 - 4 (0 = corrupt data)
     */
    public int getGoalColorInt(){
        return colorSensor.getGoalColorInt();
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
        SmartDashboard.putNumber("pigeon rough x", pigeon.getRoughOdometry().getX());
        SmartDashboard.putNumber("pigeon rough y", pigeon.getRoughOdometry().getY());
        SmartDashboard.putBoolean("pigeon bumped", pigeon.wasBumped());
        //SmartDashboard.putString("Current color", getCurrentColorString());
        //SmartDashboard.putString("Goal color", getGoalColorString());
    }
}