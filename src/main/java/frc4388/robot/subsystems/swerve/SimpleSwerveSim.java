package frc4388.robot.subsystems.swerve;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc4388.robot.subsystems.vision.VisionIO.PoseObservation;

/**
 * Minimal swerve simulator implementation for SIM mode.
 * - Accepts SwerveRequest controls (tries to extract ChassisSpeeds via reflection)
 * - Integrates chassis speeds into a Pose2d each updateInputs call
 * - Provides reset/tare/vision correction hooks
 *
 * Lightweight: does not simulate voltages or module dynamics. Suitable to make
 * the robot move in the simulator and provide pose/velocity feedback.
 */
public class SimpleSwerveSim implements SwerveIO {
    private Pose2d pose = new Pose2d();
    private Pose2d lastPose = pose;
    private double vx = 0.0;     // m/s (robot-relative)
    private double vy = 0.0;     // m/s (robot-relative)
    private double omega = 0.0;  // rad/s (robot-relative)

    private long lastTimeNs = System.nanoTime();

    public SimpleSwerveSim() {
    }

    @Override
    public synchronized void setControl(SwerveRequest ctrl) {
        if (ctrl == null) return;

        // Try to extract a ChassisSpeeds field first (common approach)
        ChassisSpeeds cs = tryGetSpeedsField(ctrl);
        if (cs != null) {
            vx = cs.vxMetersPerSecond;
            vy = cs.vyMetersPerSecond;
            omega = cs.omegaRadiansPerSecond;
            return;
        }

        // Fallbacks: try to pull numeric fields by common names
        try {
            Class<?> cls = ctrl.getClass();
            double vxF = tryGetDoubleField(ctrl, cls, "VelocityX", "velocityX", "velocityx", "VelX");
            double vyF = tryGetDoubleField(ctrl, cls, "VelocityY", "velocityY", "velocityy", "VelY");
            double rotF = tryGetDoubleField(ctrl, cls, "RotationalRate", "rotationalRate", "rotationalrate", "omega", "Omega");
            vx = vxF;
            vy = vyF;
            omega = rotF;
        } catch (Exception e) {
            // Silently ignore reflection failures and keep previous speeds.
        }
    }

    private ChassisSpeeds tryGetSpeedsField(SwerveRequest ctrl) {
        try {
            java.lang.reflect.Field f = ctrl.getClass().getDeclaredField("Speeds");
            f.setAccessible(true);
            Object o = f.get(ctrl);
            if (o instanceof ChassisSpeeds) {
                return (ChassisSpeeds) o;
            }
        } catch (NoSuchFieldException nsf) {
            // ignore
        } catch (IllegalAccessException iae) {
            // ignore
        } catch (SecurityException se) {
            // ignore
        }
        return null;
    }

    private double tryGetDoubleField(Object obj, Class<?> cls, String... names) {
        for (String n : names) {
            try {
                java.lang.reflect.Field f = cls.getDeclaredField(n);
                f.setAccessible(true);
                Object val = f.get(obj);
                if (val instanceof Number) {
                    return ((Number) val).doubleValue();
                }
            } catch (NoSuchFieldException nsf) {
                // try next name
            } catch (IllegalAccessException iae) {
                // try next name
            } catch (Throwable t) {
                // ignore other reflection issues
            }
        }
        return 0.0;
    }

    @Override
    public synchronized void updateInputs(SwerveState state) {
        if (state == null) return;

        long now = System.nanoTime();
        double dt = Math.max(1e-6, (now - lastTimeNs) / 1.0e9);
        lastTimeNs = now;

        // Save previous pose
        lastPose = pose;

        // Compute robot-relative motion over dt
        double dxRobot = vx * dt;
        double dyRobot = vy * dt;
        double dTheta = omega * dt;

        // Transform robot-relative delta into field frame using current rotation
        Rotation2d r = pose.getRotation();
        double cos = r.getCos();
        double sin = r.getSin();

        double dxField = dxRobot * cos - dyRobot * sin;
        double dyField = dxRobot * sin + dyRobot * cos;

        Translation2d newTrans = pose.getTranslation().plus(new Translation2d(dxField, dyField));
        Rotation2d newRot = r.plus(Rotation2d.fromRadians(dTheta));
        pose = new Pose2d(newTrans, newRot);

        // Populate provided SwerveState for callers
        state.lastPose = lastPose;
        state.currentPose = pose;
        state.speeds = new ChassisSpeeds(vx, vy, omega);
        state.odometryRate = dt;
    }

    
    public synchronized void driveFacingPosition(Translation2d leftStick, Translation2d fieldPos, double aimLeadTime) {
        System.out.println("It has worked!");
        System.out.println("It has worked!");
        System.out.println("It has worked!");
        System.out.println("It has worked!");
        System.out.println("It has worked!");
        System.out.println("It has worked!");
        System.out.println("It has worked!");
        if (leftStick == null || fieldPos == null) return;

        // current robot speed (robot-relative)
        Translation2d robotSpeed = new Translation2d(vx, vy);

        // lead the target by robot motion over aimLeadTime
        Translation2d fieldPosLead = fieldPos.plus(robotSpeed.times(aimLeadTime));

        // compute angle from robot to lead point (field frame)
        Rotation2d toLead = fieldPosLead.minus(pose.getTranslation()).getAngle();
        // Rotation2d ang = fieldPosLead.minus(getPose2d().getTranslation()).getAngle();

        // compute shortest angle error (ang_error = desired - current) normalized to [-pi,pi]
        double currentYaw = pose.getRotation().getRadians();
        double desiredYaw = toLead.getRadians();
        double error = desiredYaw - currentYaw;
        // normalize
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;

        // simple P controller for rotation
        final double KP_ROT = 2.0;      // tune as needed
        final double MAX_OMEGA = 6.0;   // rad/s cap
        double commandedOmega = KP_ROT * error;
        if (commandedOmega > MAX_OMEGA) commandedOmega = MAX_OMEGA;
        if (commandedOmega < -MAX_OMEGA) commandedOmega = -MAX_OMEGA;
        this.omega = commandedOmega;

        // apply translational command from leftStick (assume stick in [-1,1])
        final double STICK_SPEED_MPS = 3.0; // 3 m/s at full stick; tune as needed
        this.vx = leftStick.getX() * STICK_SPEED_MPS;
        this.vy = leftStick.getY() * STICK_SPEED_MPS;
    }

    @Override
    public synchronized void resetPose(Pose2d p) {
        if (p == null) return;
        pose = p;
        lastPose = p;
        lastTimeNs = System.nanoTime();
    }

    @Override
    public synchronized void tareEverything() {
        pose = new Pose2d();
        lastPose = pose;
        vx = 0.0;
        vy = 0.0;
        omega = 0.0;
        lastTimeNs = System.nanoTime();
    }

    @Override
    public synchronized void addVisionMeasurement(List<PoseObservation> poses) {
        if (poses == null || poses.isEmpty()) return;
        Pose2d visPose = poses.get(poses.size() - 1).pose();
        if (visPose != null) {
            pose = visPose;
            lastPose = visPose;
        }
    }

    @Override
    public synchronized void setLimits(double limitInAmps) {
        // No-op for this simple sim
    }
}