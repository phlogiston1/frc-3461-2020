package frc.robot;

import edu.wpi.first.networktables.*;

/*
 *Simple class for interfacing with the limelight camera.
*/
public class Limelight {
  public NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  private final double tHeight;
  private final double cHeight;

  public Limelight(double targetHeight, double cameraHeight){
    tHeight = targetHeight;
    cHeight = cameraHeight;
  }
  public double limelightTableValue(String entry){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry(entry).getDouble(0.0);
  }
  public void writeTableValue(String entry, double value){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry(entry).setNumber(value);
  }
  public double getTargetDistance(){
    double targetOffsetY = getTargetOffsetY();
    double distance = 0;
    distance = (tHeight - cHeight) / Math.tan(Math.toRadians(Constants.CAMERA_ANGLE) + Math.toRadians(targetOffsetY));
    return distance;
  }
  public double getTargetOffsetX(){
    return limelightTableValue("tx"); //-27 to 27 degrees
  }
  public double getTargetOffsetY(){
    return limelightTableValue("ty");
  }
  public double skew(){
    return limelightTableValue("ts");
  }
  public double getTargetShortSidelength(){ //pixels
    return limelightTableValue("tshort");
  }
  public double getTargetLongSidelength(){ //pixels
    return limelightTableValue("tlong");
  }
  public double getRoughWidth(){ //0-320 pixels
    return limelightTableValue("thor");
  }
  public double getRoughHeight(){ //0 - 320 pixels
    return limelightTableValue("tvert");
  }
  public double hasTarget() {
    double value = limelightTableValue("tv"); //0 or 1
    return value;
  }
  public void setLedOn(){
    writeTableValue("ledMode", 3);
  }
  public void setLedOff(){
    writeTableValue("ledMode", 1);
  }
  public void setLedBlink(){
    writeTableValue("ledMode", 2);
  }
  public void setLedDefault(){
    writeTableValue("ledMode", 0);
  }
  public void setModeVision(){
    writeTableValue("camMode", 0);
  }
  public void setModeDrive(){
    writeTableValue("camMode", 1);
  }
  public void setPipeline(int pipe){
    writeTableValue("pipeline", pipe);
  }
}