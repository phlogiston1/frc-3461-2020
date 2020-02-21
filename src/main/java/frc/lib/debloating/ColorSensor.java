/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.debloating;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Add your docs here.
 */
public class ColorSensor {
    I2C.Port i2cPort = I2C.Port.kOnboard;
    public ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    public final ColorMatch colorMatcher = new ColorMatch();
    private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
    public static int currentColor,goalColor = 0;
    public static double IR;
    public static ColorMatchResult match;
    public static String colorString;
    String goalColorString = "Not Received";
    public ColorSensor(){
        colorMatcher.addColorMatch(kBlueTarget);
        colorMatcher.addColorMatch(kGreenTarget);
        colorMatcher.addColorMatch(kRedTarget);
        colorMatcher.addColorMatch(kYellowTarget);
    }
    public void update(){
        Color detectedColor = colorSensor.getColor();
        match = colorMatcher.matchClosestColor(detectedColor);
        IR = colorSensor.getIR();
        //check color sensor color
        if (match.color == kBlueTarget) {
            colorString = "Blue";
            currentColor = 1;
        } else if (match.color == kRedTarget) {
            colorString = "Red";
            currentColor = 2;
        } else if (match.color == kGreenTarget) {
            colorString = "Green";
            currentColor = 3;
        } else if (match.color == kYellowTarget) {
            colorString = "Yellow";
            currentColor = 4;
        } else {
            colorString = "Unknown";
            currentColor = 0;
        }
        //get the wheel of fortune color
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length() > 0){
            switch (gameData.charAt(0)){
            case 'B' :
                //Blue case code
                goalColor = 1;
                goalColorString = "Blue";
            break;
            case 'G' :
                //Green case code
                goalColor = 2;
                goalColorString = "Green";
            break;
            case 'R' :
                //Red case code
                goalColor = 3;
                goalColorString = "Red";
            break;
            case 'Y' :
                goalColor = 4;
                goalColorString = "Yellow";
                //Yellow case code
            break;
            default :
                //This is corrupt data
            break;
        }
        } else {
            //Code for no data received yet
        }
    }
    //color sensor
    /**
     * get the name of the color sensor detected color
     * @return The name of a color as a string
     */
    public String getCurrentColorString(){
        return colorString;
    }
    /**
     * get the name of the target color
     * @return The name of a color as a string
     */
    public String getGoalColorString(){
        return goalColorString;
    }
    /**
     * get an integer corresponding to the color sensor detected color
     * @return an int from 1 - 4
     */
    public int getCurrentColorInt(){
        return currentColor;
    }
    /**
     * get an integer corresponding to the goal color
     * @return an int from 0 - 4 (0 = corrupt data)
     */
    public int getGoalColorInt(){
        return goalColor;
    }
}
