package frc.robot.subsystems.pivot;

import javax.management.ImmutableDescriptor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.ImmutableMeasure;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.UnitBuilder;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.IOs.TalonPosIO;
import frc.robot.utilities.CommandFactoryUtility;
import frc.robot.utilities.SpeakerScoreUtility;

/**
 * <h3>PivotSubsystem</h3>
 * A subsystem that represents the pivot
 */
public class PivotSubsystem extends SubsystemBase{

    private final TalonPosIO m_io;

    private final String pivotName;

    public static final double PIVOT_POSITION_DEADBAND = 2.0;
    public static final double PIVOT_VELOCITY_DEADBAND = 1.0;

    private static final double PIVOT_MIN = 0;
    private static final double PIVOT_MAX = 90;

    /**
     * <h3>PivotSubsystem</h3>
     * Creates a subsystem that represents the turret's vertical pivot on the robot.
     * <p>By default, angular measures are positive going up, negative going down, and 0 at the default horizontal
     * @param motorID
     */
    public PivotSubsystem(TalonPosIO io) {
        m_io = io;
        pivotName = "" + this.hashCode();
        //Setting stow pos on robot startup
        setPosition(CommandFactoryUtility.PIVOT_STOW_POS);
    }

    /**
     * <h3>setPosition</h3>
     * Sets the target angle of the subsystem
     * @param angle The angle in degrees from the horizontal
     */
    public void setPosition(double angle) {
        m_io.setTarget(MathUtil.clamp(angle,0.0,90.0));
        
    }

    /**
     * <h1>!!DO NOT USE!!</h1>UNTESTED<p>NO DOCUMENTATION UNTIL TESTED
     */
    public void setTarget(Measure<Angle> angle) {
        m_io.setTarget(MathUtil.clamp(angle.in(Units.Radians), PIVOT_MIN, PIVOT_MAX));
    }

    /**
     * <h3>getSetPoint</h3>
     * Gets the angle that the subsystem is currently trying to turn to
     * @param angle The angle in degrees from the horizontal
     */
    public double getTarget() {
        return m_io.getTarget();
    }

    /**
     * <h3>getPosition</h3>
     * Gets the current angle of the subsystem.
     * @return The angle in degrees from the horizontal
     */
    public double getPosition() {
        return m_io.getPos();
    }

    public Measure<Angle> getAngleMeasure() {
        return Units.Degrees.of(m_io.getPos());
    }

    /**
     * <h3>getVelocity</h3>
     * Gets the velocity in degrees per second where 0 is the horizontal and positive is up.
     * @return
     */
    public double getVelocity() {
        return m_io.getVelocity();
    }

    public Measure<Velocity<Angle>> getVelocityMeasure() {
        return Units.DegreesPerSecond.of(m_io.getVelocity());
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
        m_io.runSim();
        Logger.recordOutput(this.getClass().getSimpleName() + "/Velocity", getVelocity());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Angle", getPosition());
        Logger.recordOutput(this.getClass().getSimpleName() + "/SetPoint", getTarget());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Voltage", m_io.getVoltage());
        
    }

    public Command newSetPosCommand(double pos) {
        return new InstantCommand(() -> setPosition(pos), this);
    }

    public Command newSetPosCommand(SpeakerScoreUtility speakerUtil) {
        return new InstantCommand(() -> setPosition(speakerUtil.getPivotAngle()), this);
    }

    public boolean atSetpoint() {
        double velocity = getVelocity();
        double pos = getPosition();
        double target = getTarget();
        return MathUtil.isNear(target,pos,PIVOT_POSITION_DEADBAND) && MathUtil.isNear(0.0,velocity,PIVOT_VELOCITY_DEADBAND);

    }

    public Command newWaitUntilSetpointCommand(double seconds) {
        return new WaitCommand(seconds).until(() -> atSetpoint()); // Not dependent on subsystem because can run parralel with set position
    }
}
