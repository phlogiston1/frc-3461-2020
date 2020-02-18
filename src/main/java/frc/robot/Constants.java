/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int DRIVE_LEFT_1 = 0,
                            DRIVE_LEFT_2 = 1,
                            DRIVE_RIGHT_1 = 2,
                            DRIVE_RIGHT_2 = 3,
                            PIGEON_IMU = 4,
                            TURRET_PORT = 5,
                            SHOOTER_MOTOR_1 = 6,
                            SHOOTER_MOTOR_2 = 7;
    public static final double DRIVETRAIN_TRACKWIDTH = 0.6, // all these need to be updated + idk what units yet. all will prob be meters.
                                CAMERA_ANGLE = 0,
                                CAMERA_HEIGHT = 0.5,
                                TARGET_HEIGHT = 3,
                                INNER_GOAL_SPACING = 1,
                                DRIVE_ENCODER_CPR = 2048,
                                DRIVE_GEAR_RATIO = 9.1,
                                WHEEL_DIAMETER_METERS = 0.1524;
    public static final double AUTOAIM_kP = 0.015,
                               AUTOAIM_kI = 0.001,
                               AUTOAIM_kD = 0.0006,
                               DRIVEBASE_AUTOAIM_kP = 0.0,
                               DRIVEBASE_AUTOAIM_kI = 0.0,
                               DRIVEBASE_AUTOAIM_kD = 0.0;
	public static final double ENCODER_CPR = 0;
    public static double auto_maxspeed       = 1,
                         auto_maxacceleration = 0.2,
                         auto_maxvoltage      = 10,
                         ramsete_b            = 2.0,
                         ramsete_zeta         = 0.7,
                         odo_kS               = 0.205,
                         odo_kV               = 2.08,
                         odo_kA               = 0.208,
                         odo_kP               = 0.00322,
                         shooter_kP           = 0,
                         shooter_kI           = 0,
                         shooter_kD           = 0,
                         shooter_kF           = 0;
    public static double[] limelightSamples = {0,2,5,7,30,45}; //some data in degrees
    public static double[] limelightSampleDistances = {0,1,2,3,4,5}; //some data in meters
	public static double kWheelDiameterMeters;
}
