/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.CubicSplineInterpolate;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;

public class ArcadeDrive extends CommandBase {
  /**
   * Creates a new Drive.
   */
  public DriveTrain _drive;
  public static Joystick drvJoy = new Joystick(0);
  public boolean autoShifting = false;
  public static Gear gear = Gear.HIGH_GEAR;
  public CubicSplineInterpolate percentVSShiftVelocity = new CubicSplineInterpolate();
  public CubicSplineInterpolate percentVSVelocityOutput = new CubicSplineInterpolate();
  public ArcadeDrive(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    _drive = driveTrain;
    System.out.println("arcadeDrive");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    percentVSShiftVelocity.setSamples(Constants.percentOutputSamples, Constants.shiftPoint);
    //percentVSVelocityOutput.setSamples(Constants.percentOutputSamples, Constants.outputVelocitySamples);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //get the joystick x/y
    double x = drvJoy.getX();
    double y = -drvJoy.getY();
    //slow it down to half speed so that it can actually work like a differential.
    if(drvJoy.getRawButton(2)){
      y /= 2;
      x /= 2;
    }
    //multiply the output to low gear if we're auto shifting so that a lower
    //joystick output corresponds to max output. This way, it lines up to
    //shift into high gear
    if(gear == Gear.LOW_GEAR && autoShifting){
      y *= Constants.lowGearAutoShiftMultiplier;
      x *= Constants.lowGearAutoShiftMultiplier;
    }
    //just...drive
    _drive.percentageDrive(y - x, y + x);
    //manual shifting
    if(drvJoy.getRawButton(1)){
      gear = Gear.HIGH_GEAR;
    }else{
      gear = Gear.LOW_GEAR;
    }
    //button to reset odometry (for debugging)
    if(drvJoy.getRawButton(5)){
      RobotContainer.getRobotState().zeroHeading();
      RobotContainer.getRobotState().resetOdometry(new Pose2d(0,0,Rotation2d.fromDegrees(RobotContainer.getRobotState().getHeading())));
    }
    //auto shift
    if(autoShifting){
      DifferentialDriveWheelSpeeds velocity = _drive.getWheelSpeeds(); //get wheel speeds
      boolean lgr = velocity.leftMetersPerSecond > velocity.rightMetersPerSecond; //which wheel is faster/slower
      double highVelocity = lgr ? velocity.rightMetersPerSecond : velocity.leftMetersPerSecond; //the faster wheel speed
      double lowVelocity = !lgr ? velocity.rightMetersPerSecond : velocity.leftMetersPerSecond; //the slower wheel speed
      //if we're in high gear, check to see if we want to shift to low, and vise versa
      if(gear == Gear.HIGH_GEAR){
        //subtracts the shift threshold to prevent excessive shifting
        if(lowVelocity < percentVSShiftVelocity.cubicSplineInterpolate(y) - Constants.shiftThreshold){
          gear = Gear.LOW_GEAR;
        }
      }else{
        //subtracts the shift threshold
        if(highVelocity > percentVSShiftVelocity.cubicSplineInterpolate(y) + Constants.shiftThreshold){
          gear = Gear.HIGH_GEAR;
        }
      }
    }
    //_drive.shift(gear); TODO get the pneumatics in the robot so I can test this!
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
