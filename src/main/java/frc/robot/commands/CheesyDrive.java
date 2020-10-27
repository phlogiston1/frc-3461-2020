/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.CheesyDriveHelper;
import frc.robot.subsystems.DriveTrain;

public class CheesyDrive extends CommandBase {
  /**
   * Creates a new CheesyDrive.
   */
  DriveTrain dt;
  public static Joystick drvJoy = new Joystick(0);
  CheesyDriveHelper cd = new CheesyDriveHelper();
  public CheesyDrive(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    dt = driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double spd = drvJoy.getY();
    double rot = drvJoy.getX();
    boolean quickturn = false;
    if(drvJoy.getRawButton(7)){
      quickturn = true;
    }else{
      quickturn = false;
    }
    dt.arcadeDrive(cd.cheesyDrive(spd, rot, quickturn, true)); //todo implement ishighgear
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
