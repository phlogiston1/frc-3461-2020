/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.paths;

import java.io.IOException;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.actions.ActionBase;
import frc.robot.subsystems.DriveTrain;

/**
 * Add your docs here.
 */
public class PathBase extends CommandBase implements ActionBase{
    DriveTrain driveTrain;
    Trajectory trajectory_;
    DifferentialDriveVoltageConstraint autoVoltageConstraint;
    RamseteCommand ramsete;
    public boolean finished = false;

    public PathBase(DriveTrain subsystem) {
        driveTrain = subsystem;
        setVoltageConstraint(Constants.auto_maxvoltage);
    }
    public Command getPathbaseCommand(){
        return this;
    }
    public void setVoltageConstraint(double voltage) {
        autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.odo_kS, Constants.odo_kV, Constants.odo_kA), RobotContainer.robotState.kinematics,
                Constants.auto_maxvoltage);
    }

    public Trajectory getPathweaverTrajectory(String uri) throws IOException {
        return TrajectoryUtil.fromPathweaverJson(Paths.get(uri));
    }
    public void setTrajectory(Trajectory trajectory){
        trajectory_ = trajectory;
    }
    public TrajectoryConfig getTrajectoryConfig(){
        return new TrajectoryConfig(Constants.auto_maxspeed, Constants.auto_maxacceleration)
        .setKinematics(RobotContainer.robotState.kinematics).addConstraint(autoVoltageConstraint);
    }
   public Command getAutoCommand(){
       return ramsete;
   }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        ramsete = new RamseteCommand(
        trajectory_, 
        RobotContainer.robotState::getCurrentPose,
        new RamseteController(0, 0),
        new SimpleMotorFeedforward(
            Constants.odo_kS, 
            Constants.odo_kV, 
            Constants.odo_kA
        ), 
        RobotContainer.robotState.kinematics,
        driveTrain::getWheelSpeeds, 
        new PIDController(Constants.odo_kP, 0, 0), 
        new PIDController(Constants.odo_kP, 0, 0),
        driveTrain::voltageDrive, driveTrain
    );

ramsete.andThen(() -> driveTrain.voltageDrive(0,0));
finished = true;
    }
}
