
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.BallHandling;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.chimneySpeed;
import static frc.robot.Constants.hopperSpeed;

/**
 * An example command that uses an example subsystem.
 */
public class CycleBalls extends CommandBase { //FIXME remove example command
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private BallHandling ballHandler;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CycleBalls(BallHandling subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    ballHandler = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      ballHandler.setChimneySpeed(chimneySpeed);
      ballHandler.setHopperSpeed(hopperSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      ballHandler.setChimneySpeed(0);
      ballHandler.setHopperSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
