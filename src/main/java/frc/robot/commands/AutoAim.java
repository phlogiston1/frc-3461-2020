/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.CubicSplineInterpolate;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.lib.Limelight;
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
  Limelight camera_;
  double loopTime = 20;
  DriveTrain driveTrain;
  boolean odometryAim = true;
  CubicSplineInterpolate hoodSpline = new CubicSplineInterpolate();
  RobotState state;
  double targetOdometryErrorCorrector;
  boolean scanDirection = false;

  /**
   * AutoAim to automatically adjust the turret to the target.
   * @param subsystem The turret
   * @param dt The drivetrain
   * @param camera Limelight for vision
   * @param rs RobotState to get the angle stuff.
   */
  public AutoAim(Turret subsystem, DriveTrain dt, Limelight camera, RobotState rs) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    turret = subsystem;
    camera_ = camera;
    driveTrain = dt;
    state = rs;
    hoodSpline.setSamples(Constants.limelightSampleDistances, Constants.hoodAngles);
  }

  public Limelight getCamera(){
    return camera_;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println(">>>>>>>STARTED<<<<<<<<<<");
    //prepare the camera for vision
    camera_.setLedOn();
    camera_.setModeVision();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //set the constants
    //todo move these to constants
    kP = .15;
    kI = 0.003;
    kD = 0.02;

    double error = camera_.getTargetOffsetX(); //get how far the turret is from pointing at the target

    //this code tries to keep the turret pointed at the target all the time using odometry, even when the camera is off
    double odometryError = state.getTargetFromOdometry().getRotation2dP().getDegrees(); //wow error from odometry
    odometryError -= turret.getPosition(); //robotStates odometryError doesn't take into account turret position


    if(odometryAim){ //if we're using odometry we dont need the camera to be ready
      camera_.setLedOff();
      camera_.setModeDrive();
    }else{
      camera_.setLedOn();
      camera_.setModeVision();
      state.updateOdometryFromVision();
    }

    //debugging info help me pls
    SmartDashboard.putNumber("odometry error",odometryError);
    SmartDashboard.putNumber("odometry error error",targetOdometryErrorCorrector);

    // if we're using odometry, we dont need vision error, so set error to odometry error for simplicity
    if(odometryAim) error = odometryError;

    //calculate PID:
    integral += kI * (error/loopTime);
    if(integral > .25 || error < .01)
      integral = 0;

    double PID = kP * error + integral + kD * ((prevError - error)/loopTime); //do the pid calculations
    //end calculate PID

    prevError = error;

    //System.out.println(PID);
    if(camera_.hasTarget() == 1){
      turret.setSpeed(PID);
    }else{
      if(scanDirection){ //TODO test me
        if(turret.getPosition() < 180){ //FIXME will have issues with soft limit
          turret.setSpeed(0.4);
        }else{
          scanDirection = !scanDirection;
        }
      }else{
        if(turret.getPosition() > -180){
          turret.setSpeed(-0.4);
        }else{
          scanDirection = !scanDirection;
        }
      }
    }
    odometryAim = !RobotContainer.getInstance().getOperatorJoystick().getRawButton(Constants.ButtonMappings.limelightAimSelector);
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
