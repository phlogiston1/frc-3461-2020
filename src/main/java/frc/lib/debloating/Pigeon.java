/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.debloating;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.lib.math.Point2d;

/**
 * Add your docs here.
 */
public class Pigeon {
    private PigeonIMU pg;


    private double[] ypr = new double[3];
    private double accelerometerVelocityX = 0;
    private double accelerometerVelocityY = 0;
    private double accelerometerVelocityZ = 0;
    private double accelerometerSuperRoughX = 0;
    private double accelerometerSuperRoughY = 0;
    private double accelerometerSuperRoughZ = 0;
    private double bump_acceleration_thresh = 5000;
    private Point2d superRoughPoint = new Point2d(0,0);
    private double prevTimestamp = 0;
    private boolean bumped = false;

    public Pigeon(PigeonIMU pigeon){
        pg = pigeon;
    }
    public void update(double heading){
        pg.getYawPitchRoll(ypr);
        updateRoughOdometry(getAcceleration(), heading);
    }
    public short[] getAcceleration(){
        short[] xyz = new short[3];
        pg.getBiasedAccelerometer(xyz);
        return xyz;
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
    public void config_bumpThresh(double thresh){
        bump_acceleration_thresh = thresh;
    }
    public Point2d getRoughOdometry(){
        return superRoughPoint;
    }
    public double getOdometryZ(){
        return accelerometerSuperRoughZ;
    }
    public boolean wasBumped(){
        return bumped;
    }
    /**
     * get the fused accelerometer and magnetometer heading from the pigeon
     * @return the heading in degrees
     */
    public double getHeading(){
        return Math.IEEEremainder(pg.getFusedHeading(), 360.0d) * -1.0d;
    }
    public void zeroHeading(){
        pg.setFusedHeading(0);
    }
    public void getRawGyro(double[] xyz){
        pg.getRawGyro(xyz);
    }
    /**
     * acceleration "double integration" (not that I know what that means) to get
     * xy location of robot independent of wheel rotation. maybe use to detect bumps.
     * @param xyz the xyz acceleration from the pigeon
     */
    //super rough approximation...I hope
    public void updateRoughOdometry(short[] xyz, double heading){
        System.out.println("Pigeon: updating odometry");
        double currentTime = Timer.getFPGATimestamp();
        double dTime = currentTime - prevTimestamp;
        // pigeon acceleration is scaled to 16384 = 1G
        // / 16384 * 9.80665 converts to m/s^2
        double x = (xyz[0]);// / 16384) * 9.80665;
        double y = (xyz[1]);// / 16384) * 9.80665;
        double z = (xyz[2]) / 16384 * 9.80665;
        System.out.println("Pigeon: x acceleration of " + x);
        System.out.println("Pigeon: y acceleration of " + y);
        if(Math.abs(x) > bump_acceleration_thresh){
            bumped = true;
        }else{
            bumped = false;
        }
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
        transformPoint.rotateBy(Rotation2d.fromDegrees(heading));
        //add the change
        accelerometerSuperRoughX += transformPoint.getX();
        accelerometerSuperRoughY += transformPoint.getY();
        accelerometerSuperRoughZ += accelerometerVelocityZ*(currentTime - prevTimestamp);
        superRoughPoint = new Point2d(accelerometerSuperRoughX, accelerometerSuperRoughY);
        prevTimestamp = currentTime;
    }
}
