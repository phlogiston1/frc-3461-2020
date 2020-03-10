/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
//import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class IntakeOn extends CommandBase {
  /**
   * Creates a new Intake_ON.
   */
  Intake intake;
  public IntakeOn(Intake subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = subsystem;
    addRequirements(intake);
  }
  // Called when the command is initially scheduled.
  @Override
   public void initialize() {
     intake.extend();
    intake.setSpeed(1);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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
