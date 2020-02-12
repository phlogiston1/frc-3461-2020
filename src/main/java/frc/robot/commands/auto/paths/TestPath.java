/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.paths;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.commands.auto.PathBase;
import frc.robot.subsystems.DriveTrain;

/**
 * Add your docs here.
 */
public class TestPath extends PathBase{
    public TestPath(DriveTrain subsystem) {
        super(subsystem);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0,new Rotation2d(0)), 
        List.of(
            new Translation2d(0.5,1),
            new Translation2d(1,-0.5)
        ),
        new Pose2d(1.5,0, new Rotation2d(0)),
        getTrajectoryConfig()
        );
        setTrajectory(trajectory);
	}
}
