/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  double kI, kP, kD;
  boolean cameraAim = true;
  boolean driveBaseAim = false;
  boolean resetTimer = true;
  Limelight camera_;
  double loopTime = 20;
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

  public Limelight getCamera(){
    return camera_;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(">>>>>>>STARTED<<<<<<<<<<");
    camera_.setLedOn();
    camera_.setModeVision();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(">>>>>>>>EXIc<<<<<<<<<<");
    kP = .15;
    kI = 0.003;
    kD = 0.02;
    double error = camera_.getTargetOffsetX();
    integral += kI * (error/loopTime);
    if(integral > .25 || error < .01)
      integral = 0;
    double PID = kP * error + integral + kD * ((prevError - error)/loopTime);
    prevError = error;
    System.out.println(PID);
    turret.setSpeed(PID);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    camera_.setLedOff();
    camera_.setModeDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
