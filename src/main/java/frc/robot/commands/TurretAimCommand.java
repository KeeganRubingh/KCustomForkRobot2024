package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedSystemStats;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.utilities.AimingMathUtil;
import frc.robot.utilities.RobotOdometryUtility;
import frc.robot.utilities.SpeakerScoreUtility;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

//Automatically aims the turret to one of the speakers based on the alliance.
public class TurretAimCommand extends Command{
    private static final boolean leadShotsWhileMovingDefault = false;
    private static final boolean adjustForNoteTrajectory = false;

    private static final double AIM_OFFSET = Units.inchesToMeters(23.0); // May be dynamic
    private static final double NON_AMP_AIM_OFFSET = Units.inchesToMeters(13.0); // May be dynamic

    private TurretSubsystem m_TurretSubsystem;

    private Pose2d m_AmpSideBlueTargetPose;
    private Pose2d m_AmpSideRedTargetPose;
    private Pose2d m_NonAmpSideBlueTargetPose;
    private Pose2d m_NonAmpSideRedTargetPose;

    /** Our target position before offsets or fudges*/
    private Pose2d m_RedSpeakerTarget;
    /** Our target position before offsets or fudges*/
    private Pose2d m_BlueSpeakerTarget;

    private Pose2d m_TargetPose;
    private Pose2d m_CurrentPose;
    private final boolean useProxyPose;
    private double m_CurrentRobotHeading;
    private double m_DesiredHeading;

    private AprilTagFieldLayout m_AprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private double tx; //target x
    private double ty; //target y
    private double rx; //robot x
    private double ry; //robot y
    private boolean ampSide;
    private Pose2d m_ProxyPoseRed;
    private Pose2d m_ProxyPoseBlue;

    private DoubleSupplier xVelocitySupplier = null;
    private DoubleSupplier yVelocitySupplier = null;
    
    public TurretAimCommand(TurretSubsystem turretSubsystem) {
        this(turretSubsystem,null,null,null,null);
    }

    public TurretAimCommand(TurretSubsystem turretSubsystem, DoubleSupplier xVelocitySupplier, DoubleSupplier yVelocitySupplier) {
        this(turretSubsystem,null,null, xVelocitySupplier, yVelocitySupplier);
    }

    public TurretAimCommand(TurretSubsystem turretSubsystem, Pose2d proxyPoseRed, Pose2d proxyPoseBlue) {
        this(turretSubsystem, proxyPoseRed, proxyPoseBlue,null,null);
    }
    /** Aims the turret to the speaker apriltags based on the current alliance.
     * Constructor
     * @param turretSubsystem the subsystem that controls the turret.
     */
    public TurretAimCommand(TurretSubsystem turretSubsystem,Pose2d proxyPoseRed, Pose2d proxyPoseBlue, DoubleSupplier xVelocitySupplier, DoubleSupplier yVelocitySupplier) {
        Pose2d redPose = m_AprilTagFieldLayout.getTagPose(4).get().toPose2d();
        Pose2d bluePose = m_AprilTagFieldLayout.getTagPose(7).get().toPose2d();
        
        m_AmpSideRedTargetPose = new Pose2d(
            redPose.getX() + 0.5,
            redPose.getY() - Units.inchesToMeters(23.0 + 12.0 - 6.0), 
            redPose.getRotation());

        m_AmpSideBlueTargetPose = new Pose2d(
            bluePose.getX() - 0.5, 
            bluePose.getY() + Units.inchesToMeters(10.0), 
            bluePose.getRotation());
                
        m_NonAmpSideRedTargetPose = new Pose2d(
            redPose.getX() + 1.0, 
            redPose.getY() - Units.inchesToMeters(23.0 + 12.0 - 6.0) + NON_AMP_AIM_OFFSET, 
            redPose.getRotation());
        
        m_NonAmpSideBlueTargetPose = new Pose2d(
            bluePose.getX() - 1.0, 
            bluePose.getY() + Units.inchesToMeters(30.0 + 24.0 + 10.0), 
            bluePose.getRotation());

        m_BlueSpeakerTarget = bluePose;
        m_RedSpeakerTarget = redPose;

        //Set default target pose
        m_TargetPose = m_AmpSideBlueTargetPose;
        
        //Set proxy poses if they exist
        if(proxyPoseRed != null && proxyPoseBlue != null) {
            m_ProxyPoseRed = proxyPoseRed;
            m_ProxyPoseBlue = proxyPoseBlue;
            useProxyPose = true;
        } else {
            useProxyPose = false;
        }

        m_TurretSubsystem = turretSubsystem;
        addRequirements(m_TurretSubsystem);

        //Log offset targets
        Logger.recordOutput("AutoAim/turretTargets/ampSideRedTarget", m_AmpSideRedTargetPose);
        Logger.recordOutput("AutoAim/turretTargets/nonAmpSideRedTarget", m_NonAmpSideRedTargetPose);
        Logger.recordOutput("AutoAim/turretTargets/ampSideBlueTarget", m_AmpSideBlueTargetPose);
        Logger.recordOutput("AutoAim/turretTargets/nonAmpSideBlueTarget", m_NonAmpSideBlueTargetPose);
        Logger.recordOutput("AutoAim/adjustForNoteTrajectory",adjustForNoteTrajectory);
        Logger.recordOutput("AutoAim/leadShotWhileMoving",adjustForNoteTrajectory);
    }
    
    @Override
    public void execute() {
        // If there is an alliance present it sets the target pose based on the alliance; otherwise defaults to blue.
        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        Alliance alliance;
        if (optionalAlliance.isPresent()){
            alliance = optionalAlliance.get();
        } else {
            alliance = Alliance.Blue;
        }

        if(!useProxyPose) {
            // gets the robots position, and gets the robots heading.
            m_CurrentPose = RobotOdometryUtility.getInstance().getRobotOdometry();
        } else {
            m_CurrentPose = alliance == Alliance.Red ? m_ProxyPoseRed : m_ProxyPoseBlue;
            m_CurrentPose = new Pose2d(m_CurrentPose.getX(),m_CurrentPose.getY(), RobotOdometryUtility.getInstance().getRobotOdometry().getRotation());
        }

        m_CurrentRobotHeading = m_CurrentPose.getRotation().getDegrees();
        // logs the robot heding
        // SmartDashboard.putNumber("AutoAim/RobotHeading", m_CurrentRobotHeading);
        
        // sets the target x/y, and sets the robots x/y
        rx = m_CurrentPose.getX();
        ry = m_CurrentPose.getY();
        
        ampSide = ry >= 4.5 /*Below front pillar y (in meters)*/;
        boolean feedShot = SpeakerScoreUtility.inchesToSpeaker() > Units.metersToInches(8.0);

        Logger.recordOutput("AutoAim/ampSide", ampSide);

        if(feedShot) {
            setTargetFeedShot(alliance); // If we are doing a feed shot, use its target.
            Logger.recordOutput("AutoAim/model", 0);
        } else if (adjustForNoteTrajectory) {
            setTargetBasicModel(alliance); // If we are adjusting for trajectory, set the target to the speaker
            Logger.recordOutput("AutoAim/model", 1);
        } else {
            setTargetMidwestModel(alliance); // If we are not adjusting for trajectory, use the offset targets.
            Logger.recordOutput("AutoAim/model", 2);
        }

        if(leadShotsWhileMovingDefault) {
            leadShot();
        }
        
        tx = m_TargetPose.getX();
        ty = m_TargetPose.getY();

        // We will offset our aim to adjust for trajectory here.
        double aimOffset = adjustForNoteTrajectory ? AimingMathUtil.getTurretOffsetForDistance(SpeakerScoreUtility.inchesToSpeaker()) : 0.0;
        Logger.recordOutput("AutoAim/Offset",  aimOffset);
        
        //Logs the values above.
        Logger.recordOutput("AutoAim/tx", tx);
        Logger.recordOutput("AutoAim/ty", ty);
        Logger.recordOutput("AutoAim/rx", rx);
        Logger.recordOutput("AutoAim/ry", ry);

        // calculates how far we need to rotate the turret to get to the desired position based on:
        // robots turret heading - the robots base heading
        m_DesiredHeading = -Math.IEEEremainder(Math.toDegrees(Math.atan2(ty - ry, tx - rx)) - m_CurrentRobotHeading, 360);

        // Logs the desired heading
        // SmartDashboard.putNumber("AutoAim/Math", Math.toDegrees(Math.atan2(ty - ry, tx - rx)));
        Logger.recordOutput("AutoAim/DesiredHeading", m_DesiredHeading);

        // actually moves the robots turret to the desired position
        // TODO sussex back in
        m_TurretSubsystem.setPosition(m_DesiredHeading);
    }

    /**
     * Sets the target to the position of the alliance speaker with no offsets.
     * @param alliance
     */
    private void setTargetBasicModel(Alliance alliance) {
        if (alliance == Alliance.Red) {
            m_TargetPose = m_RedSpeakerTarget; 
        } else {
            m_TargetPose = m_BlueSpeakerTarget;
        }
    }

    /**
     * Sets the target to the position of the alliance speaker, offset to adjust for note trajectory curving.
     * @param alliance
     */
    private void setTargetMidwestModel(Alliance alliance) {
        if (alliance == Alliance.Red) {
            m_TargetPose = ampSide?m_AmpSideRedTargetPose:m_NonAmpSideRedTargetPose; 
        } else {
            m_TargetPose = ampSide?m_AmpSideBlueTargetPose:m_NonAmpSideBlueTargetPose;
        }
    }

    /**
     * Sets the target to the feed shot position.
     * @param alliance
     */
    private void setTargetFeedShot(Alliance alliance) {
        if (alliance == Alliance.Red) {
            m_TargetPose = new Pose2d(m_TargetPose.getX(), m_TargetPose.getY() - Units.inchesToMeters(70.0), m_TargetPose.getRotation());
        } else {
            m_TargetPose = new Pose2d(m_TargetPose.getX(), m_TargetPose.getY() + 4.25, m_TargetPose.getRotation());
        }
    }

    /**
     * Offsets the target to account for the robot's velocity
     */
    private void leadShot() {
        if(xVelocitySupplier == null || yVelocitySupplier == null) {
            return;
        }
        m_TargetPose = AimingMathUtil.adjustTargetForRobotVelocities(m_TargetPose, xVelocitySupplier.getAsDouble(), yVelocitySupplier.getAsDouble(), AimingMathUtil.getFlightTime(Units.inchesToMeters(Math.hypot(SpeakerScoreUtility.inchesToSpeaker(),6*12+6.5))));
    }

    // Makes it so that this command never ends.
    @Override
    public boolean isFinished() {
        return false;
    }
}
