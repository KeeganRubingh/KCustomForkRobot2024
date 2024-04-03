package frc.robot.utilities;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.Odometry;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;

public abstract class AimingMathUtil {

    // This math is not exact. The point is more to minimize the amount of variables, while getting the model closer to ideal and giving us a nice easy set of constants to tune.
    // It is also far more compatible with regression, so I can calculate these values with decent precision using desmos.
    // My overall goal with this is to create a turret model that is easier to calibrate than our current one, because I don't think we have time to perfect the current one and I really don't like it.
    
    // Calibrate the trajectory offset first, then the velocity offset.
    // The trajectory offset should compensate for the note's curved path. *Ideally*, it allows us to accurately hit a 2d position on the field from any other position.
    // The velocity offset should allow the robot to shoot while moving.


    // A guesstimate on how fast the note goes. 43.5 m/s is the speed of a minor league baseball pitch.
    private static final double NOTE_VELOCITY = 43.5; // m/s
    //Fudge factor because the shooter isn't ideal. This factor changes how much the robot velocities influence aim. Setting this to 0 disables velocity-based aiming entirely.
    private static final double VELOCITY_OFFSET_FACTOR = 1.0;
    
    //These factors control how severely note trajectory curvature is accounted for.
    //Small values for this reduce how much we compensate for initial velocity.
    private static final double TRAJ_OFFSET_LINEAR_FACTOR = 0.01;
    //Small values for this reduce how much the note's path curves.
    private static final double TRAJ_OFFSET_EXPONENTIAL_FACTOR = 0.01;

    // Desmos graph for modeling help (If we go with this model, I will finish this)
    // https://www.desmos.com/calculator/34g1rniufz

    

    //This method is primarily here so the model can be easily switched out if we need more accuracy
    public static double getFlightTime(double flightDistance) {
        return NOTE_VELOCITY / flightDistance;
    }

    /**
     * Given a certain target, applies an offset to the target to adjust for the robot's velocities.
     * I would suggest adding a deadband for the robot velocities so we don't end up shooting 
     */
    public static Pose2d adjustTargetForRobotVelocities(Pose2d target, double robotXVelocity, double robotYVelocity, double flightTime) {
        //This math calculates the offsets by acting like the goal is moving instead of the robot and leading it.
        //Theoretically, this will hit the target with a good degree of accuracy.
        //Fudge factors are built in because the shooter isn't ideal.
        
        return new Pose2d(target.getX() - robotXVelocity * flightTime * VELOCITY_OFFSET_FACTOR,target.getY() - robotYVelocity * flightTime * VELOCITY_OFFSET_FACTOR,target.getRotation());
    }

    /**
     * Given a certain distance, returns a certain offset to the turret to adjust for the notes flight characteristics
     * If this is somehow reversed based on alliance, just invert what this method returns. I don't think it is, but maybe.
     * Takes inches.
     */
    public static double getTurretOffsetForDistance(double distance) {
        //Quick test for if this offsets in the right direction, since small offsets being in the wrong direction could go unnoticed until late.
        //return 25;

        // This models the note's path as a parabolic trajectory, and gets the angle from the turret to a point on that trajectory. It then returns the negative of that angle.
        return -Math.toDegrees(Math.atan((TRAJ_OFFSET_LINEAR_FACTOR * distance + TRAJ_OFFSET_EXPONENTIAL_FACTOR * distance * distance) / distance));
    }
}
