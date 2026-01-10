package frc4388.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;

public class ShooterConstants {
    // Motor conversions
    public static final double ANGLE_MOTOR_GEAR_RATIO = 1.;
    public static final double PITCH_MOTOR_GEAR_RATIO = 1.;
    public static final double FLYWHEEL_GEAR_RATIO = 1.;
    public static final double FEEDER_INCHES_PER_ROT = 1.;

    // Limits

    // 0 is the forward angle on the robot.
    // negative is left, positive is right
    public static final Angle ANGLE_LIMIT_LEFT = Degrees.of(-180);
    public static final Angle ANGLE_LIMIT_RIGHT = Degrees.of(180);

    // 0 is paralell to the ground, 90 is directly up
    public static final Angle PITCH_LIMIT_UPPER = Degrees.of(90);
    public static final Angle PITCH_LIMIT_LOWER = Degrees.of(0);
    
    // Motor configs
    public static final TalonFXConfiguration ANGLE_MOTOR_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40) // TODO: tune???
                .withStatorCurrentLimitEnable(true)
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake) // Must be break because this has to be accurate
                    .withDutyCycleNeutralDeadband(0.04) // TODO: Figure out what this means
    );
    public static final TalonFXConfiguration PITCH_MOTOR_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40) // TODO: tune???
                .withStatorCurrentLimitEnable(true)
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake) // Must be break because this has to be accurate
                    .withDutyCycleNeutralDeadband(0.04) // TODO: Figure out what this means
    );
    public static final TalonFXConfiguration FLYWHEEL_MOTOR_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40) // TODO: tune???
                .withStatorCurrentLimitEnable(true) // TODO: Figure out what this means
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast) // Must be coast because this is spinny spinny
                    .withDutyCycleNeutralDeadband(0.04) // TODO: Figure out what this means
    );
    public static final TalonFXConfiguration FEEDER_MOTOR_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40) // TODO: tune???
                .withStatorCurrentLimitEnable(true)
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast) // Must be coast because this is spinny spinny
                    .withDutyCycleNeutralDeadband(0.04) // TODO: Figure out what this means
    );
}
