package frc4388.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.utility.configurable.ConfigurableDouble;

public class TestRobot extends SubsystemBase {

    // TalonFX m_intakeMotor;
    // TalonFX m_armMotor;
    // TalonFX m_storageMotor;
    TalonFX m_outerShooter;
    TalonFX m_innerShooter;

    ConfigurableDouble intakeRate = new ConfigurableDouble("Intake Rate", 0);


    ConfigurableDouble armUpPosition = new ConfigurableDouble("Arm Up Position", 0);
    ConfigurableDouble armDownPosition = new ConfigurableDouble("Arm Down Position", 0);

    ConfigurableDouble storageRate = new ConfigurableDouble("Storage Rate", 0);
    ConfigurableDouble outerRate = new ConfigurableDouble("Outer Rate", 0);
    ConfigurableDouble innerRate = new ConfigurableDouble("Inner Rate", 0);

    VelocityDutyCycle outerVelocity = new VelocityDutyCycle(0);
    VelocityDutyCycle innerVelocity = new VelocityDutyCycle(0);

    public static final double MAX_OUTER_VELOCITY = 1200; // Rots per second

    public enum RobotStare {
        IntakeDown,
        Idle,
        Shoot
    }

    public RobotStare robotStare = RobotStare.Idle;


    public TestRobot(
        // TalonFX intakeMotor,
        // TalonFX armMotor,
        // TalonFX storageMotor,
        TalonFX outerShooter,
        TalonFX innerShooter
    ) {
        // m_intakeMotor = intakeMotor;
        // m_armMotor = armMotor;
        // m_storageMotor = storageMotor;
        m_outerShooter = outerShooter;
        m_innerShooter = innerShooter;

        // m_intakeMotor.getConfigurator().apply(INTAKE_MOTOR_CONFIG);
        // m_armMotor.getConfigurator().apply(ARM_MOTOR_CONFIG);
        // m_storageMotor.getConfigurator().apply(STORAGE_MOTOR_CONFIG);
        m_outerShooter.getConfigurator().apply(OUTER_MOTOR_CONFIG);
        m_innerShooter.getConfigurator().apply(INNER_MOTOR_CONFIG);

        m_outerShooter.getConfigurator().apply(SHOOTER_PID);
        m_innerShooter.getConfigurator().apply(SHOOTER_PID);

        outerVelocity.Slot = 0;
        innerVelocity.Slot = 0;
    }

    // public static final TalonFXConfiguration INTAKE_MOTOR_CONFIG = new TalonFXConfiguration()
    //     .withCurrentLimits(
    //         new CurrentLimitsConfigs()
    //             .withStatorCurrentLimit(40) // TODO: tune???
    //             .withStatorCurrentLimitEnable(true) // TODO: Figure out what this means
    //         ).withMotorOutput(
    //             new MotorOutputConfigs()
    //                 .withNeutralMode(NeutralModeValue.Coast) // Must be coast because this is spinny spinny
    //                 .withDutyCycleNeutralDeadband(0.04) // TODO: Figure out what this means
    // );
    // public static final TalonFXConfiguration ARM_MOTOR_CONFIG = new TalonFXConfiguration()
    //     .withCurrentLimits(
    //         new CurrentLimitsConfigs()
    //             .withStatorCurrentLimit(40) // TODO: tune???
    //             .withStatorCurrentLimitEnable(true)
    //         ).withMotorOutput(
    //             new MotorOutputConfigs()
    //                 .withNeutralMode(NeutralModeValue.Brake) // Must be break because this has to be accurate
    //                 .withDutyCycleNeutralDeadband(0.04) // TODO: Figure out what this means
    // );

    // public static final TalonFXConfiguration STORAGE_MOTOR_CONFIG = new TalonFXConfiguration()
    //     .withCurrentLimits(
    //         new CurrentLimitsConfigs()
    //             .withStatorCurrentLimit(40) // TODO: tune???
    //             .withStatorCurrentLimitEnable(true) // TODO: Figure out what this means
    //         ).withMotorOutput(
    //             new MotorOutputConfigs()
    //                 .withNeutralMode(NeutralModeValue.Coast) // Must be coast because this is spinny spinny
    //                 .withDutyCycleNeutralDeadband(0.04) // TODO: Figure out what this means
    // );
    public static final TalonFXConfiguration OUTER_MOTOR_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40) // TODO: tune???
                .withStatorCurrentLimitEnable(true)
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast) // Must be coast because this is spinny spinny
                    .withDutyCycleNeutralDeadband(0.04) // TODO: Figure out what this means
    );
    public static final TalonFXConfiguration INNER_MOTOR_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40) // TODO: tune???
                .withStatorCurrentLimitEnable(true)
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast) // Must be coast because this is spinny spinny
                    .withDutyCycleNeutralDeadband(0.04) // TODO: Figure out what this means
                    
    );

    public static Slot0Configs SHOOTER_PID;

    static {
        SHOOTER_PID = new Slot0Configs();
        SHOOTER_PID.kV = 0.12;
        // SHOOTER_PID.kP = 0.11;
        // SHOOTER_PID.kI = 0.48;
        // SHOOTER_PID.kD = 0.01;
        SHOOTER_PID.kP = 0.3;
        SHOOTER_PID.kI = 0.0;
        SHOOTER_PID.kD = 0.0;
    }

    
    ConfigurableDouble kV = new ConfigurableDouble("kV", 0.12);
    ConfigurableDouble kP = new ConfigurableDouble("kP", 0.11);
    ConfigurableDouble kI = new ConfigurableDouble("kI", 0.48);
    ConfigurableDouble kD = new ConfigurableDouble("kD", 0.01);

    @Override
    public void periodic() {
        
        SHOOTER_PID = new Slot0Configs();
        SHOOTER_PID.kV = kV.get();
        SHOOTER_PID.kP = kP.get();
        SHOOTER_PID.kI = kI.get();
        SHOOTER_PID.kD = kD.get();

        m_outerShooter.getConfigurator().apply(SHOOTER_PID);
        m_innerShooter.getConfigurator().apply(SHOOTER_PID);

        
        SmartDashboard.putNumber("Outer Velocity", m_outerShooter.getVelocity().getValue().in(RotationsPerSecond));
        SmartDashboard.putNumber("Inner Velocity", m_innerShooter.getVelocity().getValue().in(RotationsPerSecond));


        switch (robotStare) {
            case Idle:
            
                // Move the arm to the up position
                PositionDutyCycle armPosition = new PositionDutyCycle(armUpPosition.get());
                // m_armMotor.setControl(armPosition);
            

                // // Stop the intake motor
                // m_intakeMotor.set(0);

                // // Stop the storage motor
                // m_storageMotor.set(0);

                
                // Stop the outer shooter motor
                m_outerShooter.set(0);
                // Stop the inner shooter motor
                m_innerShooter.set(0);

                break;
            case IntakeDown:
                // Move the arm to the down sposition
                PositionDutyCycle armPosition1 = new PositionDutyCycle(armDownPosition.get());
                // m_armMotor.setControl(armPosition1);

                // Move balls into the robot because the arm is down
                VelocityDutyCycle intakeVelocity = new VelocityDutyCycle(intakeRate.get());
                // m_intakeMotor.setControl(intakeVelocity);

                // Move balls into the main box
                VelocityDutyCycle storageVelocity = new VelocityDutyCycle(storageRate.get());
                // m_storageMotor.setControl(storageVelocity);

                
                // Stop the outer shooter motor
                m_outerShooter.set(0);
                // Stop the inner shooter motor
                m_innerShooter.set(0);

                break;
            case Shoot:

                // Move the arm to the up position
                PositionDutyCycle armPosition2 = new PositionDutyCycle(armUpPosition.get());
                // m_armMotor.setControl(armPosition2);

                // // Stop the intake motor
                // m_intakeMotor.set(0);

                // // Move the balls into the 
                VelocityDutyCycle storageVelocity1 = new VelocityDutyCycle(-storageRate.get());
                // m_storageMotor.setControl(storageVelocity1);
                        
                // outerVelocity.
                // m_outerShooter.setControl(outerVelocity);
                double outerRPM = outerRate.get();
                m_outerShooter.setControl(outerVelocity.withVelocity(outerRPM));//.withFeedForward(outerRPM/MAX_OUTER_VELOCITY));


                
                // m_innerShooter.setControl(innerVelocity);
                // m_innerShooter.set(innerRate.get());

                double innerRPM = innerRate.get();
                m_innerShooter.setControl(innerVelocity.withVelocity(innerRPM));//.withFeedForward(outerRPM/MAX_OUTER_VELOCITY));

                
                // SmartDashboard.putNumber("Test", outerRate.get());

                break;
            default:
                break;
        }


        
    }
    
}
