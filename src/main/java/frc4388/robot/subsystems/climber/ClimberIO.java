package frc4388.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public interface ClimberIO {
    @AutoLog
    public class ClimberState {
        Distance climberDistance = Inches.of(0);
        Distance climberTargetDistance = Inches.of(0);
        Current climberMotorCurrent = Amps.of(0);

        // Distance shooterPitch = Rotations.of(0);
        // Angle shooterTargetPitch = Rotations.of(0);
        // Current pitchMotorCurrent = Amps.of(0);

        // AngularVelocity rollerVelocity = RotationsPerSecond.of(0);
        // AngularVelocity rollerTargetVelocity = RotationsPerSecond.of(0);
        // Current rollerMotorCurrent = Amps.of(0);

        // LinearVelocity feederVelocity = InchesPerSecond.of(0);
        // LinearVelocity feederTargetVelocity = InchesPerSecond.of(0);
        // Current feederMotorCurrent = Amps.of(0);
    }

    // public default void setShooterAngle(ShooterState state, Angle angle) {}
    // public default void setShooterPitch(ShooterState state, Angle angle) {}
    public default void setClimberDistance(ClimberState state, Distance distance) {}

    public default void updateInputs(ClimberState state) {}
}