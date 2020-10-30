
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

public class DriveTrain extends SubsystemBase {
    //make some falcons
    public WPI_TalonFX lFalcon1 = new WPI_TalonFX(Constants.DRIVE_LEFT_1);
    public WPI_TalonFX lFalcon2 = new WPI_TalonFX(Constants.DRIVE_LEFT_2);
    public WPI_TalonFX rFalcon1 = new WPI_TalonFX(Constants.DRIVE_RIGHT_1);
    public WPI_TalonFX rFalcon2 = new WPI_TalonFX(Constants.DRIVE_RIGHT_2);

    //stuff for kinematics
    public static final double wheelDiameter = Math.PI * Constants.WHEEL_DIAMETER_METERS;
    public DifferentialDrive dDrive = new DifferentialDrive(lFalcon1, rFalcon1);

    //shifter stuff
    public DoubleSolenoid shiftSolenoid = new DoubleSolenoid(0,1);
    private Gear _gear = Gear.LOW_GEAR;

    double[] ypr = new double[3]; //todo cant remember wat this does
    public DriveTrain() {
        setDefaultCommand(new ArcadeDrive(this)); //TODO testme idk if this works

        //set motors on each side to follow each other
        lFalcon2.follow(lFalcon1);
        rFalcon2.follow(rFalcon1);

        //set inverted motors (to make sure nothing blows up)
        lFalcon1.setInverted(true);
        lFalcon2.setInverted(InvertType.FollowMaster);
        rFalcon1.setInverted(false);
        rFalcon2.setInverted(InvertType.FollowMaster);

        //set neutral modes:
        setNeutralModes(NeutralMode.Brake); //todo set to coast when other people trying out for drive team

        //make sure nothing blows up if I set the inverted motors wrong
        lFalcon1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 40, 5));
        rFalcon1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 40, 5));

        //make sure sensors read right
        rFalcon1.setSensorPhase(false);
        lFalcon1.setSensorPhase(false);

    }

    /**
     * shift the drivetrain into high, low, or MAYBE neutral gear
     * @param gear the drivetrain.gear to shift into
     */
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

    /**
     * get the current gear of the drivetrain gearboxes
     * @return
     */
    public Gear getGear(){
        return  _gear;
    }

    /**
     * set left and right speeds as a percentage of full power
     * @param rSpeed right side power percentage
     * @param lSpeed left side power percentage
     */
    public void percentageDrive(double rSpeed, double lSpeed){
        lFalcon1.set(ControlMode.PercentOutput, lSpeed);
        rFalcon1.set(ControlMode.PercentOutput, rSpeed);
        dDrive.feed(); //keep the wpilib arcade drive that we need for auto from freaking out
    }

    /**
     * use the wpilib arcade drive like a crazy person
     * @param spd forward speed
     * @param rot rotation left-right duh
     */
    public void arcadeDrive(double spd, double rot){
        dDrive.arcadeDrive(spd, rot);
    }

    /**
     * takes a lib.DriveSiganl from custom drivetrain code bcuz i am not a crazy person
     * @param signal the DriveSignal to set the motors to to.
     */
    public void arcadeDrive(DriveSignal signal){
        lFalcon1.set(signal.getLeft());
        rFalcon1.set(signal.getRight());
        dDrive.feed();
    }

    /**
     * drive using left and right motor voltages.
     * This is required for auto
     * @param lVolts voltage to feed to the left motor
     * @param rVolts voltage to feed to the right motor
     */
    public void voltageDrive(double lVolts, double rVolts){
        lFalcon1.setVoltage(lVolts);
        rFalcon1.setVoltage(rVolts);
        dDrive.feed();
    }

    /**
     * get wheel velocity (for auto)
     * @return wplilib differentialdrivewheelspeeds containing velocity info
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
          lFalcon1.getSelectedSensorVelocity(0)/Constants.DRIVE_ENCODER_CPR / Constants.DRIVE_HIGH_GEAR_RATIO * wheelDiameter
        , rFalcon1.getSelectedSensorVelocity(0)/Constants.DRIVE_ENCODER_CPR / Constants.DRIVE_HIGH_GEAR_RATIO * wheelDiameter
        );
    }

    /**
     * reset the falcon encoder positions back to 0
     */
    public void resetEncoders(){
        lFalcon1.setSelectedSensorPosition(0);
        rFalcon1.setSelectedSensorPosition(0);
    }

    /**
     * @return average of left and right encoder positions
     */
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

    /**
     * set maximum voltage to motors.
     * for auto, or when somebody else is trying out for drive team
     * @param maxVolts the voltage limit
     */
    public void setVoltageConstraint(double maxVolts){
        lFalcon1.configVoltageCompSaturation(maxVolts);
        rFalcon1.configVoltageCompSaturation(maxVolts);
    }

    /**
     * turn off the voltage constraint
     * @param isEnabled enable or disable the voltage constraint
     */
    public void voltageConstraintEnabled(boolean isEnabled){
        lFalcon1.enableVoltageCompensation(isEnabled);
        rFalcon1.enableVoltageCompensation(isEnabled);
    }

    /**
     * @return position of left side encoder
     */
    public double lEncoderPosition(){
        return lFalcon1.getSelectedSensorPosition()/Constants.DRIVE_ENCODER_CPR / Constants.DRIVE_HIGH_GEAR_RATIO * wheelDiameter;
    }
    /**
     * @return position of right side encoder
     */
    public double rEncoderPosition(){
        return rFalcon1.getSelectedSensorPosition()/Constants.DRIVE_ENCODER_CPR / Constants.DRIVE_HIGH_GEAR_RATIO * wheelDiameter;
    }

    /**
     * set the neutral mode of left+right motors at same time
     * This way, some doosh like me doesn't only turn off brake for one side only
     * @param mode Neutral mode to set the motors to
     */
    public void setNeutralModes(NeutralMode mode){
        lFalcon1.setNeutralMode(mode);
        rFalcon1.setNeutralMode(mode);
    }
    @Override
    public void periodic() {
    }

    /**
     * get rate of turn, I guess. I dont remember.
     * @return
     */
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
