/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.lib.math.CubicSplineInterpolate;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class RunFlywheel extends CommandBase { //FIXME remove example command
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  CubicSplineInterpolate interpolate = new CubicSplineInterpolate();
    private Shooter shooter;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunFlywheel(Shooter subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    shooter = subsystem;
    double[] distances = {0};
    double[] velocities = {0};
    interpolate.setSamples(distances, velocities); //TODO DO THIS IN THE 20 MINUTES BEFORE OUR FIRST MATCH
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distanceToTarget = RobotContainer.getRobotState().targetDistanceFromCamera();
    shooter.setFlywheelSpeed(interpolate.cubicSplineInterpolate(distanceToTarget));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      shooter.flywheelSpeedPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}