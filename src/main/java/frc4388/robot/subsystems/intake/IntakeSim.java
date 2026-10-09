package frc4388.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;

/**
 * Lightweight intake model for desktop simulation.
 *
 * <p>The arm is modeled as a position bounded between the same retracted and extended limits used
 * on the robot. This intentionally keeps the model simple: it is meant to exercise controls,
 * state transitions, limit behavior, and AdvantageKit telemetry rather than tune mechanism
 * physics.
 */
public class IntakeSim implements IntakeIO {
    private static final double ARM_MAX_SPEED_ROTATIONS_PER_SECOND = 1.0;
    private double armPositionRotations = IntakeConstants.ARM_LIMIT_RETRACTED.get();
    private double armPercentOutput;
    private double rollerPercentOutput;
    private double lastTimestampSeconds = nowSeconds();

    @Override
    public void armOutput(double percentOutput) {
        armPercentOutput = MathUtil.clamp(percentOutput, -1.0, 1.0);
    }

    @Override
    public void armFix(double percentOutput) {
        // Encoder-fix mode is allowed to drive through the normal retracted soft limit, just like
        // the real implementation. The position is still kept within the physical range.
        armPercentOutput = MathUtil.clamp(percentOutput, -1.0, 1.0);
    }

    @Override
    public void stopArm() {
        armPercentOutput = 0.0;
    }

    @Override
    public void setRollerOutput(IntakeState state, double rollerOutput) {
        rollerPercentOutput = MathUtil.clamp(rollerOutput, -1.0, 1.0);
        state.rollerTargetOutput = rollerPercentOutput;
    }

    @Override
    public void fixEncoder() {
        armPositionRotations = IntakeConstants.ARM_LIMIT_RETRACTED.get();
    }

    @Override
    public void updateInputs(IntakeState state) {
        double now = nowSeconds();
        double dt = MathUtil.clamp(now - lastTimestampSeconds, 0.0, 0.1);
        lastTimestampSeconds = now;

        double retractedLimit = IntakeConstants.ARM_LIMIT_RETRACTED.get();
        double extendedLimit = IntakeConstants.ARM_LIMIT_EXTENDED.get();
        double requestedVelocity = armPercentOutput * ARM_MAX_SPEED_ROTATIONS_PER_SECOND;
        armPositionRotations = MathUtil.clamp(
            armPositionRotations + requestedVelocity * dt, retractedLimit, extendedLimit);

        boolean atRetractedLimit = armPositionRotations <= retractedLimit;
        boolean atExtendedLimit = armPositionRotations >= extendedLimit;
        double actualVelocity = (atRetractedLimit && requestedVelocity < 0)
                || (atExtendedLimit && requestedVelocity > 0)
            ? 0.0
            : requestedVelocity;

        state.retractedLimitSwitch = atRetractedLimit;
        state.retractedSoftLimit = atRetractedLimit;
        state.extendedSoftLimit = atExtendedLimit;
        state.encoderConnected = true;
        state.intakeEncoder = Rotations.of(armPositionRotations);
        state.armAngle = Rotations.of(armPositionRotations);
        state.armMotorVelocity = RotationsPerSecond.of(actualVelocity);
        state.armMotorCurrent = Amps.of(Math.abs(armPercentOutput) * 6.0);

        state.rollerOutput = rollerPercentOutput;
        state.rollerTargetOutput = rollerPercentOutput;
        state.rollerMotorCurrent = Amps.of(Math.abs(rollerPercentOutput) * 8.0);
    }

    private static double nowSeconds() {
        return System.nanoTime() / 1.0e9;
    }
}
