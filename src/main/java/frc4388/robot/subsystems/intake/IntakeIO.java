package frc4388.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;

public interface IntakeIO {
    @AutoLog
    public class IntakeState {
        Angle armAngle = Rotations.of(0);
        Angle armTargetAngle = Rotations.of(0);
        Current armMotorCurrent = Amps.of(0);

        // Angle shooterPitch = Rotations.of(0);
        // Angle shooterTargetPitch = Rotations.of(0);
        // Current pitchMotorCurrent = Amps.of(0);

        AngularVelocity rollerVelocity = RotationsPerSecond.of(0);
        AngularVelocity rollerTargetVelocity = RotationsPerSecond.of(0);
        Current rollerMotorCurrent = Amps.of(0);

        // LinearVelocity feederVelocity = InchesPerSecond.of(0);
        // LinearVelocity feederTargetVelocity = InchesPerSecond.of(0);
        // Current feederMotorCurrent = Amps.of(0);
    }

    // public default void setShooterAngle(ShooterState state, Angle angle) {}
    // public default void setShooterPitch(ShooterState state, Angle angle) {}
    public default void setArmAngle(IntakeState state, Angle angle) {}
    public default void setRollerVelocity(IntakeState state, AngularVelocity angularVelocity) {}

    public default void updateInputs(IntakeState state) {}
}