package frc4388.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import frc4388.utility.configurable.ConfigurableDouble;
import frc4388.utility.status.CanDevice;

public class    ShooterConstants {
    // Motor conversions
    
    public static final double SHOOTERMOTOR_GEAR_RATIO = 1.5;
    public static final double INDEXER_GEAR_RATIO = 1.;

    public static final ConfigurableDouble SHOOTER_ACTIVE_VELOCITY = new ConfigurableDouble("Shooter Active Velocity", -40);
    public static final ConfigurableDouble SHOOTER_RESTING_VELOCITY = new ConfigurableDouble("Shooter Resting Velocity", 0.0);
    
    public static final ConfigurableDouble INDEXER_FORWARD_OUTPUT = new ConfigurableDouble("Indexer FWD % Output", -0.1);
    public static final ConfigurableDouble INDEXER_REVERSE_OUTPUT = new ConfigurableDouble("Indexer reverse % Output", 0.0);

    public static final ConfigurableDouble AIM_LEAD_TIME = new ConfigurableDouble("Aim lead time", 0);

    // Shoot mode tolerances
    public static final ConfigurableDouble ROBOT_MIN_HUB = new ConfigurableDouble("Shoot min dist M", 0);
    public static final ConfigurableDouble ROBOT_MAX_HUB = new ConfigurableDouble("Shoot max dist M", 99);

    public static final ConfigurableDouble ROBOT_ANG_TOLERANCE = new ConfigurableDouble("Ang tolerance DEG", 360);

    public static final ConfigurableDouble ROBOT_SPEED_TOLERANCE = new ConfigurableDouble("Speed tolerance MS", 1);
    public static final ConfigurableDouble ROBOT_ANG_SPEED_TOLERANCE = new ConfigurableDouble("Shoot Ang speed tolerance DEG", 3);

    public static final ConfigurableDouble SHOOTER_SPEED_TOLERANCE = new ConfigurableDouble("Shooter speed tolerance RPS", 5); 

    // 
    public static AngularVelocity getTargetShooterSpeed(double hubDistMeters) {
        double speed = SHOOTER_ACTIVE_VELOCITY.get();

        return RotationsPerSecond.of(speed);
    }

    // Motor Configuration
    public static Slot0Configs SHOOTER_PID = new Slot0Configs()
        .withKV(0.0)
        .withKP(0.0)
        .withKI(0.1)
        .withKD(0.08);

    
    
    
    
    
    public static ConfigurableDouble shooter_kP = new ConfigurableDouble("Shooter KP", 0.08);
    public static ConfigurableDouble shooter_kI = new ConfigurableDouble("Shooter KI", 0.1);
    public static ConfigurableDouble shooter_kD = new ConfigurableDouble("Shooter KD", 0);


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
