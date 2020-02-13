/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.math;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

/**
 * Test of the Point2d and PolarPoint2d classes.
 */
public class Test {
    public static void main(String args[]){
        System.out.println("Conversion test:");
        Point2d point1 = new Point2d(3,8);
        PolarPoint2d point2 = Point2d.getPolarPoint(point1);
        Point2d point3 = PolarPoint2d.getCartesianPoint(point2);
        System.out.println(point1.toString() + " = " + point2.toString() + " = " + point3.toString());
        System.out.println("transformation test:");
        PolarPoint2d point4 = new PolarPoint2d(3, Rotation2d.fromDegrees(90));
        System.out.println(point4.toString());
        point4.cartesianTransform(0, 3);
        System.out.println(point4.toString());
        System.out.println("(" + point4.getRotation2dP().getDegrees() + " Degrees of Rotation)");
        System.out.println(PolarPoint2d.getCartesianPoint(point4).toString());
        System.out.println("Target test:");
        PolarPoint2d target = new PolarPoint2d(4, Rotation2d.fromDegrees(20 + 90));
        System.out.println("Outer target location 4m away, 20 degrees, " + target.toString());
        target.cartesianTransform(0, 0.5);
        System.out.println("new target angle: " + (target.getRotation2dP().getDegrees() - 90));
        System.out.println("inner target info: " + target.toString());
    }
}