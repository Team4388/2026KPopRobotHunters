package frc4388.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc4388.utility.status.CanDevice;

public class IntakeConstants {
    // Motor conversions

    public static final double ARM_MOTOR_GEAR_RATIO = 1.;
    public static final double ROLLER_MOTOR_GEAR_RATIO = 1.;

    // Limits

    // 0 is the forward angle on the robot.
    // negative is left, positive is right
    public static final Angle ARM_LIMIT_LOWER = Degrees.of(-180);
    public static final Angle ARM_LIMIT_UPPER = Degrees.of(180);
    public static final AngularVelocity ROLLER_MAX_VELOCITY = RotationsPerSecond.of(0.0);

    public static final Slot0Configs ARM_PID = new Slot0Configs()
        .withKP(2.0)
        .withKI(0.0)
        .withKD(10.0);

    public static final Slot1Configs ROLLER_PID = new Slot1Configs()
        .withKP(2.0)
        .withKI(0.0)
        .withKD(10.0);

    // 0 is paralell to the ground, 90 is directly up
    // public static final Angle PITCH_LIMIT_UPPER = Degrees.of(90);
    // public static final Angle PITCH_LIMIT_LOWER = Degrees.of(0);
    
    // Motor configs
    public static final TalonFXConfiguration ARM_MOTOR_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40) // TODO: tune???
                .withStatorCurrentLimitEnable(true)
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake) // Must be break because this has to be accurate
                    .withDutyCycleNeutralDeadband(0.04) // TODO: Figure out what this means
    );

    public static final class IDs {
        public static final CanDevice FLYWHEEK_CAN_DEVICE = new CanDevice("Flywheel", 22);
    }
    // public static final TalonFXConfiguration ARM_MOTOR_CONFIG = new TalonFXConfiguration()
    //     .withCurrentLimits(
    //         new CurrentLimitsConfigs()
    //             .withStatorCurrentLimit(40) // TODO: tune???
    //             .withStatorCurrentLimitEnable(true) // TODO: Figure out what this means
    //         ).withMotorOutput(
    //             new MotorOutputConfigs()
    //                 .withNeutralMode(NeutralModeValue.Brake) // Brake so it stop
    //                 .withDutyCycleNeutralDeadband(0.04) // TODO: Figure out what this means
    // );
    public static final TalonFXConfiguration ROLLER_MOTOR_CONFIG = new TalonFXConfiguration()
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
