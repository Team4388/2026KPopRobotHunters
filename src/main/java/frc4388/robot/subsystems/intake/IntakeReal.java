package frc4388.robot.subsystems.intake;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.utility.configurable.ConfigurableDouble;

public class IntakeReal implements IntakeIO {


    TalonFX m_armMotor;
    TalonFX m_rollerMotor;

    public IntakeReal(

        TalonFX armMotor,
        TalonFX rollerMotor
    ) {
        // m_angleMotor = angleMotor;
        // m_pitchMotor = pitchMotor;
        m_armMotor = armMotor;
        m_rollerMotor = rollerMotor;

        // Apply the configs
        m_armMotor.getConfigurator().apply(IntakeConstants.ARM_PID);
        m_armMotor.getConfigurator().apply(IntakeConstants.ARM_MOTOR_CONFIG);
        m_rollerMotor.getConfigurator().apply(IntakeConstants.ROLLER_PID);
        m_rollerMotor.getConfigurator().apply(IntakeConstants.ROLLER_MOTOR_CONFIG);
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


    
    @Override
    public void setRollerVelocity(IntakeState state, AngularVelocity angularVelocity) {
        state.rollerTargetVelocity = angularVelocity;
        // (REAL_ROT / SEC) * (MOTOR_ROT / REAL_ROT) = (MOTOR_ROT / SEC)
        double motorSpeed = angularVelocity.in(RotationsPerSecond) / IntakeConstants.ROLLER_MOTOR_GEAR_RATIO;
        VelocityDutyCycle velocity = new VelocityDutyCycle(motorSpeed);
        m_rollerMotor.setControl(velocity);
    }

    @Override
    public void setArmAngle(IntakeState state, Angle angle) {
        state.armTargetAngle = angle;
        // Assume that the angle is always accurate, because I think we will use a shaft encoder
        // Assume that 0 degrees = forwards. Might need an offset here
        
        Angle boundedAngle = clampAng(angle, IntakeConstants.ARM_LIMIT_LOWER, IntakeConstants.ARM_LIMIT_UPPER);
        // (REAL_ROT) * (MOTOR_ROT / REAL_ROT) = MOTOR_ROT
        double motorTargetAngle = boundedAngle.in(Rotations) / IntakeConstants.ARM_MOTOR_GEAR_RATIO;
        PositionDutyCycle posRequest = new PositionDutyCycle(motorTargetAngle);
        m_armMotor.setControl(posRequest);
    }

    @Override
    public void updateInputs(IntakeState state) {
        state.armAngle = m_armMotor.getPosition().getValue().times(IntakeConstants.ARM_MOTOR_GEAR_RATIO);
        state.armMotorCurrent = m_armMotor.getStatorCurrent(false).getValue();

        // state.shooterPitch = m_pitchMotor.getPosition().getValue().times(ShooterConstants.PITCH_MOTOR_GEAR_RATIO);
        // state.pitchMotorCurrent = m_pitchMotor.getStatorCurrent().getValue();

        // state.armAngle = m_armMotor.getPosition().getValue();
        // state.armMotorCurrent = m_armMotor.getStatorCurrent().getValue();

        state.rollerVelocity = m_rollerMotor.getVelocity().getValue();
        state.rollerMotorCurrent = m_rollerMotor.getStatorCurrent().getValue();
    }
}
