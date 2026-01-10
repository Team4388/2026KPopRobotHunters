package frc4388.robot.subsystems.shooter;

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

public interface ShooterIO {
    @AutoLog
    public class ShooterState {
        Angle shooterAngle = Rotations.of(0);
        Angle shooterTargetAngle = Rotations.of(0);
        Current angleMotorCurrent = Amps.of(0);

        Angle shooterPitch = Rotations.of(0);
        Angle shooterTargetPitch = Rotations.of(0);
        Current pitchMotorCurrent = Amps.of(0);

        AngularVelocity flywheelVelocity = RotationsPerSecond.of(0);
        AngularVelocity flywheelTargetVelocity = RotationsPerSecond.of(0);
        Current flywheelMotorCurrent = Amps.of(0);

        LinearVelocity feederVelocity = InchesPerSecond.of(0);
        LinearVelocity feederTargetVelocity = InchesPerSecond.of(0);
        Current feederMotorCurrent = Amps.of(0);
    }

    public default void setShooterAngle(ShooterState state, Angle angle) {}
    public default void setShooterPitch(ShooterState state, Angle angle) {}
    public default void setFlywheelVelocity(ShooterState state, AngularVelocity angularVelocity) {}
    public default void setFeederVelocity(ShooterState state, LinearVelocity linearVelocity) {}

    public default void updateInputs(ShooterState state) {}
}