package frc4388.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class ShooterReal implements ShooterIO {

    TalonFX m_shooter1Motor;
    TalonFX m_shooter2Motor;
    TalonFX m_indexerMotor;

    VelocityDutyCycle shooter1Velocity = new VelocityDutyCycle(0);
    VelocityDutyCycle shooter2Velocity = new VelocityDutyCycle(0);
    // VelocityDutyCycle m_indexerVelocity = new VelocityDutyCycle(0);


    public ShooterReal(
         TalonFX shooter1Motor,
         TalonFX shooter2Motor,
         TalonFX indexerMotor
    ) {
        m_shooter1Motor = shooter1Motor;
        m_shooter2Motor = shooter2Motor;
        m_indexerMotor  = indexerMotor;

        m_shooter1Motor.getConfigurator().apply(ShooterConstants.SHOOTER_PID);
        m_shooter2Motor.getConfigurator().apply(ShooterConstants.SHOOTER_PID);

        m_shooter1Motor.getConfigurator().apply(ShooterConstants.SHOOTER1_MOTOR_CONFIG);
        m_shooter2Motor.getConfigurator().apply(ShooterConstants.SHOOTER2_MOTOR_CONFIG);
        m_indexerMotor.getConfigurator().apply(ShooterConstants.INDEXER_MOTOR_CONFIG);

        shooter1Velocity.Slot = 0;
        shooter2Velocity.Slot = 0;
        // m_indexerVelocity.Slot = 0;
    }
    
    @Override
    public void setShooterVelocity(ShooterState state, AngularVelocity target) {
        state.motor1TargetVelocity = target;
        state.motor2TargetVelocity = target;

        if(target.baseUnitMagnitude() == 0) {
            m_shooter1Motor.set(0);
            m_shooter2Motor.set(0);
            return;
        }

        AngularVelocity motorRps = target.times(ShooterConstants.SHOOTERMOTOR_GEAR_RATIO);

        m_shooter1Motor.setControl(shooter1Velocity.withVelocity(motorRps));
        m_shooter2Motor.setControl(shooter2Velocity.withVelocity(motorRps));
    }

    @Override
    public void setShooterCurrentLimitSpeed(
        ShooterState state, 
        double percentOutput
        // Current currentLimit,
        // AngularVelocity targetVelocity
    ) {
        // state.motor1TargetVelocity = targetVelocity;
        // state.motor2TargetVelocity = targetVelocity;

        // double current = Math.abs(state.motor1Current.in(Amps)) + Math.abs(state.motor2Current.in(Amps));
        // double velocity = (Math.abs(state.motor1Velocity.in(RotationsPerSecond)) + Math.abs(state.motor2Velocity.in(RotationsPerSecond))) / 2;

        // if(
        //     Math.abs(currentLimit.in(Amps)) > current &&
        //     Math.abs(targetVelocity.in(RotationsPerSecond)) > velocity
        // ) {
            m_shooter1Motor.set(percentOutput);
            m_shooter2Motor.set(percentOutput);
        // } else {
        //     m_shooter1Motor.set(0);
        //     m_shooter2Motor.set(0);
        // }
    }

    @Override
    public void setIndexerOutput(ShooterState state, double percentOutput) {
        state.indexerTargetOutput = percentOutput;
        m_indexerMotor.set(percentOutput);
    }


    @Override
    public void updateInputs(ShooterState state) {

        state.motor1Velocity = m_shooter1Motor.getVelocity().getValue().div(ShooterConstants.SHOOTERMOTOR_GEAR_RATIO);
        state.motor2Velocity = m_shooter2Motor.getVelocity().getValue().div(ShooterConstants.SHOOTERMOTOR_GEAR_RATIO);
        state.indexerOutput = m_indexerMotor.get();
        // state.indexerOutput = m_indexerMotor.getVelocity().getValue().div(ShooterConstants.INDEXER_GEAR_RATIO);

        // state.motorLinearVelocity = InchesPerSecond.of(m_shooter1Motor.getVelocity().getValue().in(RotationsPerSecond) * ShooterConstants.FEEDER_INCHES_PER_ROT);
        
        state.motor1Current = m_shooter1Motor.getStatorCurrent().getValue();
        state.motor2Current = m_shooter2Motor.getStatorCurrent().getValue();
        state.indexerCurrent = m_indexerMotor.getStatorCurrent().getValue();

    }

    @Override 
    public void updateGains() {
        // TEMPORARY PIDs
        ShooterConstants.SHOOTER_PID.kP = ShooterConstants.shooter_kP.get();
        ShooterConstants.SHOOTER_PID.kI = ShooterConstants.shooter_kI.get();
        ShooterConstants.SHOOTER_PID.kD = ShooterConstants.shooter_kD.get();
        m_shooter1Motor.getConfigurator().apply(ShooterConstants.SHOOTER_PID);
        m_shooter2Motor.getConfigurator().apply(ShooterConstants.SHOOTER_PID);
    }
    
}
