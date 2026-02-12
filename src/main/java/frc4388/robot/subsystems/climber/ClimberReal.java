package frc4388.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.*;

public class ClimberReal implements ClimberIO {


    TalonFX m_climberMotor;

    public ClimberReal(

        TalonFX climberMotor
    ) {
        // m_angleMotor = angleMotor;
        // m_pitchMotor = pitchMotor;
        m_climberMotor = climberMotor;
        // Apply the configs
        m_climberMotor.getConfigurator().apply(ClimberConstants.CLIMBER_PID);
        m_climberMotor.getConfigurator().apply(ClimberConstants.CLIMBER_MOTOR_CONFIG);

    }

    private Distance clampDistance(Distance distance, Distance climberLimitLower, Distance climberLimitUpper){
            if(distance.gt(climberLimitUpper)) {
                return climberLimitUpper;
            }else if(distance.lt(climberLimitLower)) {
                return climberLimitLower;
            }else{
                return distance;
        }
    }


    

    @Override
    public void setClimberDistance(ClimberState state, Distance distance) {
        state.climberTargetDistance = distance;
        // Assume that the angle is always accurate, because I think we will use a shaft encoder
        // Assume that 0 degrees = forwards. Might need an offset here
        
        Distance boundedDistance = clampDistance(distance, ClimberConstants.CLIMBER_LIMIT_LOWER, ClimberConstants.CLIMBER_LIMIT_UPPER);
        // (REAL_ROT) * (MOTOR_ROT / REAL_ROT) = MOTOR_ROT
        double motorTargetDistance = boundedDistance.in(Inches) / ClimberConstants.CLIMBER_MOTOR_GEAR_RATIO;
        PositionDutyCycle posRequest = new PositionDutyCycle(motorTargetDistance);
        m_climberMotor.setControl(posRequest);
    }

    @Override
    public void updateInputs(ClimberState state) {
        double motorRotations = m_climberMotor.getPosition().getValue().in(Rotations);
        double linearInches = motorRotations * ClimberConstants.CLIMBER_MOTOR_GEAR_RATIO;
        state.climberDistance = Inches.of(linearInches);
        state.climberMotorCurrent = m_climberMotor.getStatorCurrent(false).getValue();

        // state.shooterPitch = m_pitchMotor.getPosition().getValue().times(ShooterConstants.PITCH_MOTOR_GEAR_RATIO);
        // state.pitchMotorCurrent = m_pitchMotor.getStatorCurrent().getValue();

        // state.armAngle = m_armMotor.getPosition().getValue();
        // state.armMotorCurrent = m_armMotor.getStatorCurrent().getValue();


    }
}
