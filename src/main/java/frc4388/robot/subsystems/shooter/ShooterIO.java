package frc4388.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public interface ShooterIO {
    @AutoLog
    public class ShooterState {
        // Angle shooterAngle = Rotations.of(0);
        // Angle shooterTargetAngle = Rotations.of(0);
        // Current angleMotorCurrent = Amps.of(0);

        // Angle shooterPitch = Rotations.of(0);
        // Angle shooterTargetPitch = Rotations.of(0);
        // Current pitchMotorCurrent = Amps.of(0);

        AngularVelocity motor1TargetVelocity = RotationsPerSecond.of(0);
        AngularVelocity motor2TargetVelocity = RotationsPerSecond.of(0);
        double indexerTargetOutput = 0;


        AngularVelocity motor1Velocity = RotationsPerSecond.of(0);
        AngularVelocity motor2Velocity = RotationsPerSecond.of(0);
        AngularVelocity indexerForwardVelocity = RotationsPerSecond.of(42.53); //guestimate
        double indexerOutput = 0;
        
        Current motor1Current = Amps.of(0);
        Current motor2Current = Amps.of(0);
        Current indexerCurrent = Amps.of(0);

        // LinearVelocity motorLinearVelocity = InchesPerSecond.of(0);
    }

    // public default void setShooterAngle(ShooterState state, Angle angle) {}
    // public default void setShooterPitch(ShooterState state, Angle angle) {}
    public default void setShooterVelocity(ShooterState state, AngularVelocity angularVelocity) {}
    public default void setShooterCurrentLimitSpeed(
        ShooterState state, 
        double percentOutput
        // Current currentLimit,
        // AngularVelocity targetVelocity
    ) {}
    // public default void setMotor2Velocity(ShooterState state, AngularVelocity angularVelocity) {}
    public default void setIndexerOutput(ShooterState state, double percentOutput) {}

    public default void updateInputs(ShooterState state) {}

    
    public default void updateGains() {}
}