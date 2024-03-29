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
    //port numbers
    public static final int DRIVE_LEFT_1 = 0, //Lower
                            DRIVE_LEFT_2 = 1, //Upper 
                            DRIVE_RIGHT_1 = 2, //Lower
                            DRIVE_RIGHT_2 = 3,//Upper
                            PIGEON_IMU = 20,
                            TURRET_PORT = 9,
                            SHOOTER_MOTOR_1 = 5,
                            SHOOTER_MOTOR_2 = 4,
                            HOPPER_MOTOR = 7,
                            CHIMNEY_MOTOR = 8,
                            BEAM_BREAK_1 = 12,
                            BEAM_BREAK_2 = 13,
                            INTAKE_MOTOR = 6,
                            INTAKE_SOLENOID_A = 3,
                            INTAKE_SOLENOID_B = 2,
                            SHIFT_SOLENOID_A = 0,
                            SHIFT_SOLENOID_B = 1;
    //physical dimensions
    public static final double DRIVETRAIN_TRACKWIDTH = 0.6, // all these need to be updated + idk what units yet. all will prob be meters.
                                CAMERA_ANGLE = 0,
                                CAMERA_HEIGHT = 0.467,
                                TARGET_HEIGHT = 1,
                                INNER_GOAL_SPACING = 1,
                                DRIVE_ENCODER_CPR = 2048,
                                DRIVE_HIGH_GEAR_RATIO = 9.1,
                                DRIVE_LOW_GEAR_RATIO = 24,
                                WHEEL_DIAMETER_METERS = 0.1524;
    //gains
    public static final double AUTOAIM_kP = 0.015,
                               AUTOAIM_kI = 0.001,
                               AUTOAIM_kD = 0.0006,
                               DRIVEBASE_AUTOAIM_kP = 0.0,
                               DRIVEBASE_AUTOAIM_kI = 0.0,
                               DRIVEBASE_AUTOAIM_kD = 0.0,
                               TURRET_POSITION_kP = 0.0,
                               TURRET_POSITION_kI = 0.0,
                               TURRET_POSITION_kD = 0.0;
    //gains that aren't capitalized
    public static double auto_maxspeed       = 2,
                         auto_maxacceleration = 2,
                         auto_maxvoltage      = 10,
                         ramsete_b            = 2.0,
                         ramsete_zeta         = 1.2,
                         odo_kS               = 0.205,
                         odo_kV               = 2.08,
                         odo_kA               = 0.208,
                         odo_kP               = 0.02,
                         shooter_kP           = 0.5,
                         shooter_kI           = 0,
                         shooter_kD           = 0,
                         shooter_kF           = 0.06,
                         bump_acceleration_thresh = 0;
    //limelight
    public static double[] hoodAngles =           {0,2,5,7,30,45}; //some data in degrees
    public static double[] limelightSampleDistances =   {0,1,2,3,4,5}; //some data in meters
    //shifting
    public static double[] shiftPoint = {0,0,0,0,0,0};
    public static double[] percentOutputSamples =       {0,0,0,0,0,0,0};
    //public static double[] outputVelocitySamples =      {0,0,0,0,0,0,0};
    public static double kWheelDiameterMeters;
    public static double lowGearAutoShiftMultiplier = 0;
    public static double shiftThreshold = 0.2;

    //default speeds:
    public static double hopperSpeed = -0.3,
                         chimneySpeed = 1;

    public static class ButtonMappings{
        public static final int driveJoystickPort = 0,
                                operatorJoystickPort = 1;
        //todo everything
        public static final int autoAim = 6,
                                limelightAimSelector = 6,
                                intakeOn = 5,
                                intakeUp = 7,
                                runBallFeed = 1,
                                emergencyReverseBallFeed = 10;
    }
}
