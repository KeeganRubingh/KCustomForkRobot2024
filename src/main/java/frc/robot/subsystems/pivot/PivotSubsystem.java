package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IOs.TalonPosIO;

public class PivotSubsystem extends SubsystemBase{
    private final TalonPosIO io;

    private final double GEAR_RATIO = 1;

    private final Slot0Configs PID_FF_CONFIGS = new Slot0Configs()
        .withKP(1) //TODO: Configure
        .withKI(0) //TODO: Configure
        .withKD(0) //TODO: Configure
        .withKA(1) //TODO: Configure
        .withKG(0) //TODO: Configure
        .withKS(0) //TODO: Configure
        .withKV(1);//TODO: Configure

    private final MotionMagicConfigs MM_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(80)//TODO: Configure
        .withMotionMagicExpo_kV(1)
        .withMotionMagicExpo_kA(4);//TODO: Configure

    private final String pivotName;

    /**
     * <h3>PivotSubsystem</h3>
     * Creates a subsystem that represents the turret's vertical pivot on the robot.
     * <p>By default, angular measures are positive going up, negative going down, and 0 at the default horizontal
     * @param motorID
     */
    public PivotSubsystem(int motorID, String CANbus) {
        this.io = new PivotIORobot(new TalonFX(motorID,CANbus), GEAR_RATIO,PID_FF_CONFIGS,MM_CONFIGS);
        pivotName = "" + this.hashCode();
    }

    /**
     * <h3>setPosition</h3>
     * Sets the target angle of the subsystem
     * @param angle The angle in degrees from the horizontal
     */
    public void setPosition(double angle) {
        io.setTarget(MathUtil.clamp(angle,0,180));
        
    }

    /**
     * <h3>getSetPoint</h3>
     * Gets the angle that the subsystem is currently trying to turn to
     * @param angle The angle in degrees from the horizontal
     */
    public double getSetPoint() {
        return io.getTarget();
    }

    /**
     * <h3>getPosition</h3>
     * Gets the current angle of the subsystem.
     * @return The angle in degrees from the horizontal
     */
    public double getPosition() {
        return io.getPos();
    }

    /**
     * <h3>getVelocity</h3>
     * Gets the velocity in degrees per second where 0 is the horizontal and positive is up.
     * @return
     */
    public double getVelocity() {
        return io.getVelocity();
    }

    /**
     * <h3>getVoltage</h3>
     * Gets the current voltage of the subystem.
     * @return The voltage the motor is running at.
     */
   // public double getVoltage() {

   // }

    @Override
    public void periodic() {
        io.runSim();
        SmartDashboard.putNumber("Pivot-" + pivotName + "/Velocity", getVelocity());
        SmartDashboard.putNumber("Pivot-" + pivotName + "/Height", getPosition());
        SmartDashboard.putNumber("Pivot-" + pivotName + "/SetPoint", getSetPoint());
        SmartDashboard.putNumber("Pivot-" + pivotName + "/Voltage", io.getVoltage());
        
    }

    public StartEndCommand getTestCommand() {
        return new StartEndCommand(()->{setPosition(90); System.out.println("Pivot Test Start");}, ()->{setPosition(0);}, this);
    }

}
