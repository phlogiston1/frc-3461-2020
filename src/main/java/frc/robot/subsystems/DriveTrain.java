
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DriveSignal;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.CheesyDrive;

public class DriveTrain extends SubsystemBase {
    public WPI_TalonFX lFalcon1 = new WPI_TalonFX(Constants.DRIVE_LEFT_1);
    public WPI_TalonFX lFalcon2 = new WPI_TalonFX(Constants.DRIVE_LEFT_2);
    public WPI_TalonFX rFalcon1 = new WPI_TalonFX(Constants.DRIVE_RIGHT_1);
    public WPI_TalonFX rFalcon2 = new WPI_TalonFX(Constants.DRIVE_RIGHT_2);
    public static final double wheelDiameter = Math.PI * Constants.WHEEL_DIAMETER_METERS;
    public DifferentialDrive dDrive = new DifferentialDrive(lFalcon1, rFalcon1);
    public DoubleSolenoid shiftSolenoid = new DoubleSolenoid(0,1);
    private Gear _gear = Gear.LOW_GEAR;
    double[] ypr = new double[3];
    public DriveTrain() {
        setDefaultCommand(new ArcadeDrive(this)); //TODO testme idk if this works
        lFalcon2.follow(lFalcon1);
        rFalcon2.follow(rFalcon1);
        //set inverted motors
        lFalcon1.setInverted(true);
        lFalcon2.setInverted(InvertType.FollowMaster);
        rFalcon1.setInverted(false);
        rFalcon2.setInverted(InvertType.FollowMaster);
        //set neutral modes:
        setNeutralModes(NeutralMode.Brake);
        lFalcon1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 40, 5));
        rFalcon1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 40, 5));
        rFalcon1.setSensorPhase(false);
        lFalcon1.setSensorPhase(false);

    }
    public void shift(Gear gear){
        if(gear == Gear.HIGH_GEAR){
            shiftSolenoid.set(Value.kForward);
        }
        if(gear == Gear.LOW_GEAR){
            shiftSolenoid.set(Value.kReverse);
        }
        if(gear == Gear.NEUTRAL){
            shiftSolenoid.set(Value.kOff);
        }
        _gear = gear;
    }
    public Gear getGear(){
        return  _gear;
    }
    public void percentageDrive(double rSpeed, double lSpeed){
        lFalcon1.set(ControlMode.PercentOutput, lSpeed);
        rFalcon1.set(ControlMode.PercentOutput, rSpeed);
        dDrive.feed();
    }
    public void arcadeDrive(double spd, double rot){
        dDrive.arcadeDrive(spd, rot);
    }
    public void arcadeDrive(DriveSignal signal){
        lFalcon1.set(signal.getLeft());
        rFalcon1.set(signal.getRight());
        dDrive.feed();
    }
    public void voltageDrive(double lVolts, double rVolts){
        lFalcon1.setVoltage(lVolts);
        rFalcon1.setVoltage(rVolts);
        dDrive.feed();
    }
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
          lFalcon1.getSelectedSensorVelocity(0)/Constants.DRIVE_ENCODER_CPR / Constants.DRIVE_HIGH_GEAR_RATIO * wheelDiameter
        , rFalcon1.getSelectedSensorVelocity(0)/Constants.DRIVE_ENCODER_CPR / Constants.DRIVE_HIGH_GEAR_RATIO * wheelDiameter
        );
      }
    public void resetEncoders(){
        lFalcon1.setSelectedSensorPosition(0);
        rFalcon1.setSelectedSensorPosition(0);
    }
    public double getAverageEncoderDistance(){
        return (lEncoderPosition() + rEncoderPosition()) / 2.0;
    }
    public double getYaw(){
        return ypr[0];
    }
    public double getPitch(){
        return ypr[1];
    }
    public double getRoll(){
        return ypr[2];
    }
    public void setVoltageConstraint(double maxVolts){
        lFalcon1.configVoltageCompSaturation(maxVolts);
        rFalcon1.configVoltageCompSaturation(maxVolts);
    }
    public void voltageConstraintEnabled(boolean isEnabled){
        lFalcon1.enableVoltageCompensation(isEnabled);
        rFalcon1.enableVoltageCompensation(isEnabled);
    }
    public double lEncoderPosition(){
        return lFalcon1.getSelectedSensorPosition()/Constants.DRIVE_ENCODER_CPR / Constants.DRIVE_HIGH_GEAR_RATIO * wheelDiameter;
    }
    public double rEncoderPosition(){
        return rFalcon1.getSelectedSensorPosition()/Constants.DRIVE_ENCODER_CPR / Constants.DRIVE_HIGH_GEAR_RATIO * wheelDiameter;
    }
    public void setNeutralModes(NeutralMode mode){
        lFalcon1.setNeutralMode(mode);
        rFalcon1.setNeutralMode(mode);
    }
    @Override
    public void periodic() {
    }
    public double getTurnRate(){
        double[] xyz = new double[3];
        RobotContainer.getRobotState().pigeon.getRawGyro(xyz);
        return xyz[2];
    }
    public enum Gear{
        HIGH_GEAR,
        LOW_GEAR,
        NEUTRAL;
    }
}
