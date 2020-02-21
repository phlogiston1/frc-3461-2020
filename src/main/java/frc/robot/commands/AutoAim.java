/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.CubicSplineInterpolate;
import frc.lib.math.PolarPoint2d;
import frc.robot.Constants;
import frc.lib.Limelight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;

public class AutoAim extends CommandBase {
  /**
   * Creates a new AutoAim.
   */
  Turret turret;
  double integral;
  double prevError = 0;
  boolean cameraAim = true;
  boolean driveBaseAim = false;
  boolean resetTimer = true;
  Limelight camera_;
  double timeSinceAcquiredTarget;
  DriveTrain driveTrain;
  Timer timer = new Timer();
  CubicSplineInterpolate hoodSpline = new CubicSplineInterpolate();
  public AutoAim(Turret subsystem, DriveTrain dt, Limelight camera) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    turret = subsystem;
    camera_ = camera;
    driveTrain = dt;
    hoodSpline.setSamples(Constants.limelightSampleDistances, Constants.hoodAngles);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //initialize gains
    double kP = Constants.AUTOAIM_kP,
           kI = Constants.AUTOAIM_kI,
           kD = Constants.AUTOAIM_kD;
    //get the vision error
    double errorFromVision = RobotContainer.robotState.innerTargetAngleFromCamera();
    //get the odometry error
    Rotation2d errorFromOdometry = PolarPoint2d.fromPose(RobotContainer.robotState.getCurrentPose()).getRotation2dP();
    //switch between vision and odometry error
    double error = (cameraAim ? errorFromVision : errorFromOdometry.getDegrees() - turret.getPosition());
    //get the hood angle by interpolating distance with empirical data
    double hoodAngle = hoodSpline.cubicSplineInterpolate(RobotContainer.robotState.targetDistanceFromCamera());
    //set the hood position to the correct angle or 0 if we aren't aiming
    turret.setHoodPosition(cameraAim ? hoodAngle : 0);
    //adjust the constants if the drivebase turret backup is on
    if(driveBaseAim){
      kP = Constants.DRIVEBASE_AUTOAIM_kP;
      kI = Constants.DRIVEBASE_AUTOAIM_kI;
      kD = Constants.DRIVEBASE_AUTOAIM_kD;
    }
    //turn on the led's if we're trying to aim
    if(cameraAim){
      camera_.setLedOn();
      camera_.setModeVision();
      timer.start();
    }else{
      camera_.setLedOff();
      camera_.setModeDrive();
    }
    //run the pid loop
    prevError = error;
    if(integral > .25 || error == 0) integral = 0;
    integral += error * kI;
    double PIDOut = kP * error + kD * (error - prevError) + integral;
    //set the speed of the turret
    if(turret.autoAiming){
      if(driveBaseAim){
        driveTrain.percentageDrive(-PIDOut, PIDOut);
      }
      turret.setSpeed(PIDOut);
    }
    //make sure the turret is functional. If it isn't enable the drivebase backup
    if(timer.hasPeriodPassed(0.2) && error > 0.02){ //TODO update to accurate values
      driveBaseAim = true;
    }
    //reset the drivebase backup once the aim is complete
    if(error < 0.02){ //TODO update to accurate values
      driveBaseAim = false;
      timer.stop();
      timer.reset();
    }
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
