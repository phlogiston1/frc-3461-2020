/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoAim;
import frc.robot.commands.CarGameCheesyDrive;
import frc.robot.commands.CheesyDrive;
import frc.robot.commands.CycleBalls;
import frc.robot.commands.IntakeOff;
import frc.robot.commands.IntakeOn;
import frc.robot.commands.IntakeRetract;
import frc.robot.commands.TankDrive;
import frc.robot.commands.auto.paths.PathBase;
import frc.robot.commands.auto.paths.TestPath;
import frc.lib.Limelight;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final DriveTrain driveTrain = new DriveTrain();
  private static final Limelight camera = new Limelight(Constants.TARGET_HEIGHT, Constants.CAMERA_HEIGHT);
  private static final Turret turret = new Turret();
  private static final Intake intake = new Intake();
  private static final BallHandling ballHandler = new BallHandling();

  private static RobotContainer instance;

  private static SendableChooser<Command> driveChooser = new SendableChooser<Command>();

  private static final RobotState robotState = new RobotState(driveTrain, camera);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private final Joystick oprJoy = new Joystick(1);

  public RobotContainer() {
    driveChooser.addOption("Arcade Drive", new ArcadeDrive(driveTrain));
    driveChooser.setDefaultOption("Cheesy Drive", new CheesyDrive(driveTrain));
    driveChooser.addOption("Tank Drive", new TankDrive(driveTrain));
    driveChooser.addOption("Car Game Cheesy Drive", new CarGameCheesyDrive(driveTrain));
    // Configure the button bindings
    System.out.println("initializing robot container");
    configureButtonBindings();
    SmartDashboard.putData(driveChooser);
  }


  public static RobotContainer getInstance(){
    if (instance == null) {
      instance = new RobotContainer();
    }
    return instance;
  }

  public static Command getDriveCommand(){
    //todo why is this set like this??
    return new CheesyDrive(driveTrain);//driveChooser.getSelected();
  }

  public Limelight getLimelight() {
    return camera;
  }

  public static RobotState getRobotState() {
    return robotState;
  }

  public DriveTrain getDriveTrain() {
    return driveTrain;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  JoystickButton autoAimBtn = new JoystickButton(oprJoy, Constants.ButtonMappings.autoAim);
  JoystickButton intakeBtn = new JoystickButton(oprJoy, Constants.ButtonMappings.intakeOn);
  JoystickButton intakeUp = new JoystickButton(oprJoy, Constants.ButtonMappings.intakeUp);
  JoystickButton runTraversal = new JoystickButton(oprJoy, Constants.ButtonMappings.runBallFeed);
  private void configureButtonBindings() {
    autoAimBtn.whileHeld(new AutoAim(turret, driveTrain, camera, robotState));
    intakeBtn.whenPressed(new IntakeOn(intake));
    intakeBtn.whenReleased(new IntakeOff(intake));
    intakeUp.whenPressed(new IntakeRetract(intake));
    runTraversal.whileHeld(new CycleBalls(ballHandler));
  }

  public Joystick getOperatorJoystick() {
    return oprJoy;
  }
  public Turret getTurret(){
    return turret;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * @throws IOException
   */
  public PathBase getAutonomousCommand() throws IOException {
    // An ExampleCommand will run in autonomous
    TestPath testAuto = new TestPath(driveTrain);
    return testAuto;
  }
}
