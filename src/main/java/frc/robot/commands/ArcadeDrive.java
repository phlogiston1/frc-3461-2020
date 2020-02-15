/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;

public class ArcadeDrive extends CommandBase {
  /**
   * Creates a new Drive.
   */
  public DriveTrain _drive;
  public static Joystick drvJoy = new Joystick(0);
  public ArcadeDrive(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    _drive = driveTrain;
    System.out.println("arcadeDrive");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = drvJoy.getX();
    double y = drvJoy.getY();
    if(drvJoy.getRawButton(2)){
      y /= 2;
      x /= 2;
    }
    _drive.percentageDrive(y - x, y + x);
    if(drvJoy.getRawButton(1)){
      _drive.shift(Gear.HIGH_GEAR);
    }else{
      _drive.shift(Gear.LOW_GEAR);
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
