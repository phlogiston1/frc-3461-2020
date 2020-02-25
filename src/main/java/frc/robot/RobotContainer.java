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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoAim;
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
  private static RobotContainer instance;

  private static final RobotState robotState = new RobotState(driveTrain, camera);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private final Joystick oprJoy = new Joystick(1);

  public RobotContainer() {
    // Configure the button bindings
    System.out.println("initializing robot container");
    configureButtonBindings();
  }
  public static RobotContainer getInstance(){
    if (instance == null) {
      instance = new RobotContainer();
    }
    return instance;
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
  JoystickButton autoAimBtn = new JoystickButton(oprJoy, 6);

  private void configureButtonBindings() {
    autoAimBtn.whileHeld(new AutoAim(turret, driveTrain, camera, robotState));
  }

  public Joystick getOperatorJoystick() {
    return oprJoy;
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
