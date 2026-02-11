package frc4388.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc4388.robot.subsystems.intake.IntakeConstants;
import frc4388.utility.configurable.ConfigurableDouble;

public class ShooterReal implements ShooterIO {

    TalonFX m_shooter1Motor;
    TalonFX m_shooter2Motor;
    TalonFX m_indexerMotor;

    VelocityDutyCycle shooter1Velocity = new VelocityDutyCycle(0);
    VelocityDutyCycle shooter2Velocity = new VelocityDutyCycle(0);
    VelocityDutyCycle m_indexerVelocity = new VelocityDutyCycle(0);


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
        m_indexerMotor.getConfigurator().apply(ShooterConstants.SHOOTER_PID);

        m_shooter1Motor.getConfigurator().apply(ShooterConstants.SHOOTER1_MOTOR_CONFIG);
        m_shooter2Motor.getConfigurator().apply(ShooterConstants.SHOOTER2_MOTOR_CONFIG);
        m_indexerMotor.getConfigurator().apply(ShooterConstants.INDEXER_MOTOR_CONFIG);

        shooter1Velocity.Slot = 0;
        shooter2Velocity.Slot = 0;
        m_indexerVelocity.Slot = 0;
    }

    private Angle clampAng(Angle x, Angle min, Angle max){
        if(x.gt(max)) {
            return max;
        }else if(x.lt(min)) {
            return min;
        }else{
            return x;
        }
    }



   

    // // TODO: Test
    // @Override
    // public void setShooterAngle(ShooterState state, Angle angle) {
    //     state.shooterTargetAngle = angle;
    //     // Assume that the angle is always accurate, because I think we will use a shaft encoder
    //     // Assume that 0 degrees = forwards. Might need an offset here

    //     Angle boundedAngle = clampAng(angle, ShooterConstants.ANGLE_LIMIT_LEFT, ShooterConstants.ANGLE_LIMIT_RIGHT);
    //     // (REAL_ROT) * (MOTOR_ROT / REAL_ROT) = MOTOR_ROT
    //     double motorTargetAngle = boundedAngle.in(Rotations) / ShooterConstants.ANGLE_MOTOR_GEAR_RATIO;
    //     PositionDutyCycle posRequest = new PositionDutyCycle(motorTargetAngle);
    //     m_angleMotor.setControl(posRequest);
    // }


    // TODO: Test
    // @Override
    // public void setShooterPitch(ShooterState state, Angle angle) {
    //     state.shooterTargetPitch = angle;
    //     // TODO: Test
    //     // This assumes that the 0 is paralell to the ground. Might need an offset here


    //     Angle boundedAngle = clampAng(angle, ShooterConstants.PITCH_LIMIT_UPPER, ShooterConstants.PITCH_LIMIT_LOWER);
    //     // (REAL_ROT) * (MOTOR_ROT / REAL_ROT) = MOTOR_ROT
    //     double motorTargetAngle = boundedAngle.in(Rotations) / ShooterConstants.PITCH_MOTOR_GEAR_RATIO;
    //     PositionDutyCycle posRequest = new PositionDutyCycle(motorTargetAngle);
    //     m_pitchMotor.setControl(posRequest);
    // }
    

    @Override
    public void setShooterVelocity(ShooterState state, AngularVelocity target) {
        state.motor1TargetVelocity = target;
        state.motor2TargetVelocity = target;

        if(target.baseUnitMagnitude() == 0) {
            m_shooter1Motor.set(0);
            m_shooter2Motor.set(0);
            return;
        }

        AngularVelocity motorRps = target.times(ShooterConstants.INDEXER_GEAR_RATIO);

        m_shooter1Motor.setControl(shooter1Velocity.withVelocity(motorRps));
        m_shooter2Motor.setControl(shooter2Velocity.withVelocity(motorRps));
    }

    @Override
    public void setIndexerVelocity(ShooterState state, AngularVelocity target) {
        state.indexerTargetVelocity = target;

        if(target.baseUnitMagnitude() == 0) {
            m_indexerMotor.set(0);
            return;
        }

        AngularVelocity motorRps = target.times(ShooterConstants.INDEXER_GEAR_RATIO);
        m_indexerMotor.setControl(m_indexerVelocity.withVelocity(motorRps));
    }


    @Override
    public void updateInputs(ShooterState state) {

        state.motor1Velocity = m_shooter1Motor.getVelocity().getValue().div(ShooterConstants.SHOOTERMOTOR1_GEAR_RATIO);
        state.motor2Velocity = m_shooter2Motor.getVelocity().getValue().div(ShooterConstants.SHOOTERMOTOR2_GEAR_RATIO);
        state.indexerVelocity = m_indexerMotor.getVelocity().getValue().div(ShooterConstants.INDEXER_GEAR_RATIO);

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

        ShooterConstants.INDEXER_PID.kP = ShooterConstants.indexer_kP.get();
        ShooterConstants.INDEXER_PID.kI = ShooterConstants.indexer_kI.get();
        ShooterConstants.INDEXER_PID.kD = ShooterConstants.indexer_kD.get();
        m_indexerMotor.getConfigurator().apply(ShooterConstants.INDEXER_PID);
    }
    
}
