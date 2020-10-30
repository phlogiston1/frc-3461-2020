/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.paths;

import java.io.IOException;
import frc.robot.subsystems.DriveTrain;

/**
 * Testing the PathBase framework. just call start on this to drive the path.
 */
public class TestPath extends PathBase {
    /**
     * creates a new trajectory, and then sets it in the PathBase as the one to
     * follow. Call start on this to drive the path.
     * 
     * @param subsystem drive train to pass to PathBase
     * @throws IOException
     */
    public TestPath(DriveTrain subsystem) throws IOException {
        super(subsystem);
        //create a new trajectory
        /*Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0,new Rotation2d(0)),
        List.of(
            new Translation2d(1,0.5),
            new Translation2d(4,0.75),
            new Translation2d(3,0),
            new Translation2d(2,0)
        ),
        new Pose2d(1,0.5, Rotation2d.fromDegrees(0)),
        getTrajectoryConfig()
        );*/
        //set the trajectory
        setTrajectory(getPathweaverTrajectory("output/Unnamed_0.wpilib.json"));
        System.out.println("trajectory ready");
	}
}
