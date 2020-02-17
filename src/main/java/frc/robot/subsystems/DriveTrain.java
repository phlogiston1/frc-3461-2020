
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ArcadeDrive;

public class DriveTrain extends SubsystemBase {
    public WPI_TalonFX lFalcon1 = new WPI_TalonFX(Constants.DRIVE_LEFT_1);
    public WPI_TalonFX lFalcon2 = new WPI_TalonFX(Constants.DRIVE_LEFT_2);
    public WPI_TalonFX rFalcon1 = new WPI_TalonFX(Constants.DRIVE_RIGHT_1);
    public WPI_TalonFX rFalcon2 = new WPI_TalonFX(Constants.DRIVE_RIGHT_2);
    //public DoubleSolenoid shiftSolenoid = new DoubleSolenoid(0,1);
    private Gear _gear = Gear.LOW_GEAR;
    double[] ypr = new double[3];
    public DriveTrain() {
        setDefaultCommand(new ArcadeDrive(this));
        lFalcon2.follow(lFalcon1);
        rFalcon2.follow(rFalcon1);
        //set inverted motors
        lFalcon1.setInverted(false);
        lFalcon2.setInverted(InvertType.FollowMaster);
        rFalcon1.setInverted(true);
        rFalcon2.setInverted(InvertType.FollowMaster);
        //set neutral modes:
        setNeutralModes(NeutralMode.Brake);
        lFalcon1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 40, 5));
        rFalcon1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 40, 5));
        //rFalcon1.setSensorPhase(false);

    }
    /*
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
    }*/
    public Gear getGear(){
        return  _gear;
    }
    public void percentageDrive(double lSpeed, double rSpeed){
        lFalcon1.set(ControlMode.PercentOutput, lSpeed);
        rFalcon1.set(ControlMode.PercentOutput, rSpeed);
    }
    public void voltageDrive(double lVolts, double rVolts){
        lFalcon1.setVoltage(lVolts);
        rFalcon1.setVoltage(rVolts);
    }
    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(lFalcon1.getSelectedSensorVelocity(), rFalcon1.getSelectedSensorVelocity());
    }
    public void resetEncoders(){
        lFalcon1.setSelectedSensorPosition(0);
        rFalcon1.setSelectedSensorPosition(0);
    }
    public double getAverageEncoderDistance(){
        return (lFalcon1.getSelectedSensorPosition() + rFalcon1.getSelectedSensorPosition()) / 2.0;
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
        return lFalcon1.getSelectedSensorPosition()/2048 / 9.1 * 0.1524;
    }
    public double rEncoderPosition(){
        return rFalcon1.getSelectedSensorPosition()/2048 / 9.1 * 0.1524;
    }
    public void setNeutralModes(NeutralMode mode){
        lFalcon1.setNeutralMode(mode);
        rFalcon1.setNeutralMode(mode);
    }
    @Override
    public void periodic() {
    }
    public enum Gear{
        HIGH_GEAR,
        LOW_GEAR,
        NEUTRAL;
    }
}
