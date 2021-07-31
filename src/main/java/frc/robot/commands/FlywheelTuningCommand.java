package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class FlywheelTuningCommand extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private ShuffleboardTab tab = Shuffleboard.getTab("shooter");
    private NetworkTableEntry p = tab.add("p",Constants.shooter_kP).getEntry();
    private NetworkTableEntry i = tab.add("i",Constants.shooter_kI).getEntry();
    private NetworkTableEntry d = tab.add("d",Constants.shooter_kD).getEntry();
    private NetworkTableEntry f = tab.add("f",Constants.shooter_kF).getEntry();
    private NetworkTableEntry speed = tab.add("target velocity",0.1).getEntry();
    private NetworkTableEntry vel = tab.add("flywheel velocity",0.1).getEntry();
    Shooter shooter;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FlywheelTuningCommand(Shooter subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    shooter = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.shooter_kP = p.getDouble(0);
    Constants.shooter_kI = i.getDouble(0);
    Constants.shooter_kD = d.getDouble(0);
    Constants.shooter_kF = f.getDouble(0);
    shooter.setTalonGains(p.getDouble(0), i.getDouble(0), d.getDouble(0), f.getDouble(0));
      shooter.setFlywheelSpeed(speed.getDouble(0));
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
    vel.setDouble(shooter.getFlywheelVelocity());
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
