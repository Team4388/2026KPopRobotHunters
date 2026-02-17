package frc4388.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc4388.utility.configurable.ConfigurableDouble;
import frc4388.utility.status.CanDevice;

public class    ShooterConstants {
    // Motor conversions
    
    public static final double SHOOTERMOTOR_GEAR_RATIO = 1.5;
    public static final double INDEXER_GEAR_RATIO = 1.;
    
    // public static final AngularVelocity SHOOTER_ACTIVE_VELOCITY = RotationsPerSecond.of(30);
    // public static final AngularVelocity SHOOTER_RESTING_VELOCITY = RotationsPerSecond.of(15);
    // public static final AngularVelocity SHOOTER_INACTIVE_VELOCITY = RotationsPerSecond.of(0.0);

    // public static final AngularVelocity INDEXER_ACTIVE_VELOCITY = RotationsPerSecond.of(10);
    // public static final AngularVelocity INDEXER_INACTIVE_VELOCITY = RotationsPerSecond.of(0.0);

    public static final ConfigurableDouble SHOOTER_ACTIVE_VELOCITY = new ConfigurableDouble("Shooter Active Velocity", -35);
    public static final ConfigurableDouble SHOOTER_RESTING_VELOCITY = new ConfigurableDouble("Shooter Resting Velocity", -10);
    
    public static final ConfigurableDouble INDEXER_FORWARD_OUTPUT = new ConfigurableDouble("Indexer FWD Velocity", -10);
    public static final ConfigurableDouble INDEXER_REVERSE_OUTPUT = new ConfigurableDouble("Indexer reverse Velocity", 10);


    // Tolerances
    public static final ConfigurableDouble ROBOT_MIN_HUB = new ConfigurableDouble("Shoot min dist M", 0);
    public static final ConfigurableDouble ROBOT_MAX_HUB = new ConfigurableDouble("Shoot max dist M", 99);

    public static final ConfigurableDouble ROBOT_ANG_TOLERANCE = new ConfigurableDouble("Ang tolerance DEG", 360);

    public static final ConfigurableDouble ROBOT_SPEED_TOLERANCE = new ConfigurableDouble("Speed tolerance MS", 1);
    public static final ConfigurableDouble ROBOT_ANG_SPEED_TOLERANCE = new ConfigurableDouble("Shoot Ang speed tolerance DEG", 3);

    public static final ConfigurableDouble SHOOTER_SPEED_TOLERANCE = new ConfigurableDouble("Shooter speed tolerance RPS", 5); 

    

    public static Slot0Configs SHOOTER_PID = new Slot0Configs()
        .withKV(0.0)
        .withKP(0.0)
        .withKI(0.1)
        .withKD(0.08);

    public static Slot0Configs INDEXER_PID = new Slot0Configs()
        .withKV(0.0)
        .withKP(0.0)
        .withKI(0.0)
        .withKD(0.2);
    
    
    public static ConfigurableDouble indexer_kP = new ConfigurableDouble("Indexer KP", 0.2);
    public static ConfigurableDouble indexer_kI = new ConfigurableDouble("Indexer KI", 0);
    public static ConfigurableDouble indexer_kD = new ConfigurableDouble("Indexer KD", 0);
    
    public static ConfigurableDouble shooter_kP = new ConfigurableDouble("Shooter KP", 0.08);
    public static ConfigurableDouble shooter_kI = new ConfigurableDouble("Shooter KI", 0.1);
    public static ConfigurableDouble shooter_kD = new ConfigurableDouble("Shooter KD", 0);

    // Limits

    // 0 is the forward angle on the robot.
    // negative is left, positive is right
    // public static final Angle ANGLE_LIMIT_LEFT = Degrees.of(-180);
    // public static final Angle ANGLE_LIMIT_RIGHT = Degrees.of(180);

    // 0 is paralell to the ground, 90 is directly up
    // public static final Angle PITCH_LIMIT_UPPER = Degrees.of(90);
    // public static final Angle PITCH_LIMIT_LOWER = Degrees.of(0);
    
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


    public static final CanDevice SHOOTER1_ID = new CanDevice("SHOOTER 1", 22);
    public static final CanDevice SHOOTER2_ID = new CanDevice("SHOOTER 2", 23);
    public static final CanDevice INDEXER_ID  = new CanDevice("INDEXER",24);
    

    public static final TalonFXConfiguration SHOOTER1_MOTOR_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40) // TODO: tune???
                .withStatorCurrentLimitEnable(true)
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast) // Must be coast because this is spinny spinny
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
