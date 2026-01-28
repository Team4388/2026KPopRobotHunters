package frc4388.robot.subsystems.intake;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.*;

public class IntakeReal implements IntakeIO {

    // TalonFX m_angleMotor;
    // TalonFX m_pitchMotor;
    TalonFX m_flywheelMotor;
    TalonFX m_feederMotor;

    public IntakeReal(
        // TalonFX angleMotor,
        // TalonFX pitchMotor,
        TalonFX flywheelMotor,
        TalonFX feederMotor
    ) {
        // m_angleMotor = angleMotor;
        // m_pitchMotor = pitchMotor;
        m_flywheelMotor = flywheelMotor;
        m_feederMotor = feederMotor;

        // Apply the configs
        // m_angleMotor.getConfigurator().apply(IntakeConstants.ANGLE_MOTOR_CONFIG);
        // m_pitchMotor.getConfigurator().apply(IntakeConstants.PITCH_MOTOR_CONFIG);
        m_flywheelMotor.getConfigurator().apply(IntakeConstants.FLYWHEEL_MOTOR_CONFIG);
        m_feederMotor.getConfigurator().apply(IntakeConstants.FEEDER_MOTOR_CONFIG);
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
    // public void setIntakeAngle(IntakeState state, Angle angle) {
    //     state.IntakeTargetAngle = angle;
    //     // Assume that the angle is always accurate, because I think we will use a shaft encoder
    //     // Assume that 0 degrees = forwards. Might need an offset here

    //     Angle boundedAngle = clampAng(angle, IntakeConstants.ANGLE_LIMIT_LEFT, IntakeConstants.ANGLE_LIMIT_RIGHT);
    //     // (REAL_ROT) * (MOTOR_ROT / REAL_ROT) = MOTOR_ROT
    //     double motorTargetAngle = boundedAngle.in(Rotations) / IntakeConstants.ANGLE_MOTOR_GEAR_RATIO;
    //     PositionDutyCycle posRequest = new PositionDutyCycle(motorTargetAngle);
    //     m_angleMotor.setControl(posRequest);
    // }


    // TODO: Test
    // @Override
    // public void setIntakePitch(IntakeState state, Angle angle) {
    //     state.IntakeTargetPitch = angle;
    //     // TODO: Test
    //     // This assumes that the 0 is paralell to the ground. Might need an offset here


    //     Angle boundedAngle = clampAng(angle, IntakeConstants.PITCH_LIMIT_UPPER, IntakeConstants.PITCH_LIMIT_LOWER);
    //     // (REAL_ROT) * (MOTOR_ROT / REAL_ROT) = MOTOR_ROT
    //     double motorTargetAngle = boundedAngle.in(Rotations) / IntakeConstants.PITCH_MOTOR_GEAR_RATIO;
    //     PositionDutyCycle posRequest = new PositionDutyCycle(motorTargetAngle);
    //     m_pitchMotor.setControl(posRequest);
    // }
    
    @Override
    public void setFlywheelVelocity(IntakeState state, AngularVelocity angularVelocity) {
        state.flywheelTargetVelocity = angularVelocity;
        // (REAL_ROT / SEC) * (MOTOR_ROT / REAL_ROT) = (MOTOR_ROT / SEC)
        double motorSpeed = angularVelocity.in(RotationsPerSecond) / IntakeConstants.FLYWHEEL_GEAR_RATIO;
        VelocityDutyCycle velocity = new VelocityDutyCycle(motorSpeed);
        m_feederMotor.setControl(velocity);
    }

    @Override
    public void setFeederVelocity(IntakeState state, LinearVelocity linearVelocity) {
        state.feederTargetVelocity = linearVelocity;
        // (IN / SEC) * (ROT / IN) = (ROT / SEC)
        double motorSpeed = linearVelocity.in(InchesPerSecond) / IntakeConstants.FEEDER_INCHES_PER_ROT;
        VelocityDutyCycle velRequest = new VelocityDutyCycle(motorSpeed);
        m_feederMotor.setControl(velRequest);
    }

    @Override
    public void updateInputs(IntakeState state) {
        // state.IntakeAngle = m_angleMotor.getPosition().getValue().times(IntakeConstants.ANGLE_MOTOR_GEAR_RATIO);
        // state.angleMotorCurrent = m_angleMotor.getStatorCurrent(false).getValue();

        // state.IntakePitch = m_pitchMotor.getPosition().getValue().times(IntakeConstants.PITCH_MOTOR_GEAR_RATIO);
        // state.pitchMotorCurrent = m_pitchMotor.getStatorCurrent().getValue();

        state.flywheelVelocity = m_flywheelMotor.getVelocity().getValue();
        state.flywheelMotorCurrent = m_flywheelMotor.getStatorCurrent().getValue();

        state.feederVelocity = InchesPerSecond.of(m_feederMotor.getVelocity().getValue().in(RotationsPerSecond) * IntakeConstants.FEEDER_INCHES_PER_ROT);
        state.feederMotorCurrent = m_feederMotor.getStatorCurrent().getValue();
    }
    
}
