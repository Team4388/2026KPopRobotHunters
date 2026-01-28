package frc4388.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import frc4388.utility.status.CanDevice;

public class ShooterConstants {
    // Motor conversions
    
    public static final double FEEDER_INCHES_PER_ROT = 1.;
    public static final double SHOOTERMOTOR1_GEAR_RATIO = 1.;
    public static final double SHOOTERMOTOR2_GEAR_RATIO = 1.;
    public static final double INDEXER_GEAR_RATIO = 1.;

    public static Slot0Configs SHOOTER_PID = new Slot0Configs()
        .withKV(0.0)
        .withKP(0.0)
        .withKI(0.0)
        .withKD(0.0);

    // Limits

    // 0 is the forward angle on the robot.
    // negative is left, positive is right
    // public static final Angle ANGLE_LIMIT_LEFT = Degrees.of(-180);
    // public static final Angle ANGLE_LIMIT_RIGHT = Degrees.of(180);

    // 0 is paralell to the ground, 90 is directly up
    public static final Angle PITCH_LIMIT_UPPER = Degrees.of(90);
    public static final Angle PITCH_LIMIT_LOWER = Degrees.of(0);
    
    // Motor configs
    // public static final TalonFXConfiguration ANGLE_MOTOR_CONFIG = new TalonFXConfiguration()
    //     .withCurrentLimits(
    //         new CurrentLimitsConfigs()
    //             .withStatorCurrentLimit(40) // TODO: tune???
    //             .withStatorCurrentLimitEnable(true)
    //         ).withMotorOutput(
    //             new MotorOutputConfigs()
    //                 .withNeutralMode(NeutralModeValue.Brake) // Must be break because this has to be accurate
    //                 .withDutyCycleNeutralDeadband(0.04) // TODO: Figure out what this means
    // );

    public static final class IDs {
        public static final CanDevice FLYWHEEK_CAN_DEVICE = new CanDevice("Flywheel", 22);
    }

    public static final TalonFXConfiguration SHOOTER1_MOTOR_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40) // TODO: tune???
                .withStatorCurrentLimitEnable(true)
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake) // Must be break because this has to be accurate
                    .withDutyCycleNeutralDeadband(0.04) // TODO: Figure out what this means
    );
    public static final TalonFXConfiguration SHOOTER2_MOTOR_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40) // TODO: tune???
                .withStatorCurrentLimitEnable(true) // TODO: Figure out what this means
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast) // Must be coast because this is spinny spinny
                    .withDutyCycleNeutralDeadband(0.04) // TODO: Figure out what this means
    );
    public static final TalonFXConfiguration INDEXER_MOTOR_CONFIG = new TalonFXConfiguration()
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
