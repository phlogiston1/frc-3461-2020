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

/**
 * cheesy drive using car-game style controls with an xbox controller
 */
public class CarGameCheesyDrive extends CommandBase {
  /**
   * Creates a new CarGameCheesyDrive.
   */
  DriveTrain dt;
  public static Joystick drvJoy = new Joystick(0);
  CheesyDriveHelper cd = new CheesyDriveHelper(); //Thank you team 254, your code is now mine!
  public CarGameCheesyDrive(DriveTrain driveTrain) {
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
    double spd = drvJoy.getRawAxis(0) - drvJoy.getRawAxis(0);//todo get throttle and joystick axes
    double rot = drvJoy.getRawAxis(0);
    boolean quickturn = false;
    if(drvJoy.getRawButton(0)){
      quickturn = true;
    }else{
      quickturn = false;
    }
    dt.arcadeDrive(cd.cheesyDrive(spd,rot,quickturn,true)); //todo implement ishighgear
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
