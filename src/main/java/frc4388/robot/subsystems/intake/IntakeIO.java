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
        // Angle IntakeAngle = Rotations.of(0);
        // Angle IntakeTargetAngle = Rotations.of(0);
        // Current angleMotorCurrent = Amps.of(0);

        // Angle IntakePitch = Rotations.of(0);
        // Angle IntakeTargetPitch = Rotations.of(0);
        // Current pitchMotorCurrent = Amps.of(0);

        AngularVelocity flywheelVelocity = RotationsPerSecond.of(0);
        AngularVelocity flywheelTargetVelocity = RotationsPerSecond.of(0);
        Current flywheelMotorCurrent = Amps.of(0);

        LinearVelocity feederVelocity = InchesPerSecond.of(0);
        LinearVelocity feederTargetVelocity = InchesPerSecond.of(0);
        Current feederMotorCurrent = Amps.of(0);
    }

    // public default void setIntakeAngle(IntakeState state, Angle angle) {}
    // public default void setIntakePitch(IntakeState state, Angle angle) {}
    public default void setFlywheelVelocity(IntakeState state, AngularVelocity angularVelocity) {}
    public default void setFeederVelocity(IntakeState state, LinearVelocity linearVelocity) {}

    public default void updateInputs(IntakeState state) {}
}