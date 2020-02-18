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
import frc.robot.commands.auto.paths.PathBase;
import frc.robot.subsystems.DriveTrain;

/**
 * Testing the PathBase framework. just call start on this to drive the path.
 */
public class TestPath extends PathBase{
    /**
     * creates a new trajectory,
     * and then sets it in the PathBase as the one to follow.
     * Call start on this to drive the path.
     * @param subsystem drive train to pass to PathBase
     */
    public TestPath(DriveTrain subsystem) {
        super(subsystem);

        //create a new trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0,new Rotation2d(0)),
        List.of(
            new Translation2d(0.5,0)
        ),
        new Pose2d(1,0, new Rotation2d(0)),
        getTrajectoryConfig()
        );
        //set the trajectory
        setTrajectory(trajectory);
        System.out.println("trajectory ready");
	}
}
