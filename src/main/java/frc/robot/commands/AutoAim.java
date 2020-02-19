/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.CubicSplineInterpolate;
import frc.lib.math.PolarPoint2d;
import frc.robot.Constants;
import frc.robot.Limelight;
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
    double kP = Constants.AUTOAIM_kP,
           kI = Constants.AUTOAIM_kI,
           kD = Constants.AUTOAIM_kD;
    double errorFromVision = RobotContainer.robotState.innerTargetAngleFromCamera();
    double errorFromOdometry = PolarPoint2d.fromPose(RobotContainer.robotState.getCurrentPose()).getP();
    double error = (cameraAim ? errorFromVision : errorFromOdometry) - turret.getPosition();
    double hoodAngle = hoodSpline.cubicSplineInterpolate(RobotContainer.robotState.targetDistanceFromCamera());
    turret.setHoodPosition(hoodAngle);
    if(driveBaseAim){
      kP = Constants.DRIVEBASE_AUTOAIM_kP;
      kI = Constants.DRIVEBASE_AUTOAIM_kI;
      kD = Constants.DRIVEBASE_AUTOAIM_kD;
    }
    if(cameraAim){
      camera_.setLedOn();
      camera_.setModeVision();
      timer.start();
    }else{
      camera_.setLedOff();
      camera_.setModeDrive();
    }
    prevError = error;
    if(integral > .25 || error == 0) integral = 0;
    integral += error * kI;
    double PIDOut = kP * error + kD * (error - prevError) + integral;
    if(turret.autoAiming){
      if(driveBaseAim){
        driveTrain.percentageDrive(-PIDOut, PIDOut);
      }
      turret.setSpeed(PIDOut);
    }
    if(timer.hasPeriodPassed(0.2) && error > 0.02){
      driveBaseAim = true;
    }
    if(error < 0.02){
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
