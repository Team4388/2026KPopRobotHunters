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

    PositionDutyCycle armPosition = new PositionDutyCycle(0);
    VelocityDutyCycle rollerVelocity = new VelocityDutyCycle(0);

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

        armPosition.Slot = 0;
        rollerVelocity.Slot = 0;
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
        AngularVelocity motorSpeed = angularVelocity.div(IntakeConstants.ROLLER_MOTOR_GEAR_RATIO);

        // m_rollerMotor.set(motorSpeed);
        // VelocityDutyCycle velocity = new VelocityDutyCycle(motorSpeed);
        m_rollerMotor.setControl(rollerVelocity.withVelocity(motorSpeed));
    }

    @Override
    public void setArmAngle(IntakeState state, Angle angle) {
        state.armTargetAngle = angle;
        // Assume that the angle is always accurate, because I think we will use a shaft encoder
        // Assume that 0 degrees = forwards. Might need an offset here


        // (REAL_ROT) * (MOTOR_ROT / REAL_ROT) = MOTOR_ROT
        Angle motorAngle = angle.div(IntakeConstants.ARM_MOTOR_GEAR_RATIO);
        
        // PositionDutyCycle posRequest = new PositionDutyCycle(motorTargetAngle);
        m_armMotor.setControl(armPosition.withPosition(motorAngle));
    }

    ConfigurableDouble arm_kP = new ConfigurableDouble("ARM KP", 0.2);
    ConfigurableDouble arm_kI = new ConfigurableDouble("ARM KP", 0);
    ConfigurableDouble arm_kD = new ConfigurableDouble("ARM KP", 0);
    
    ConfigurableDouble roller_kP = new ConfigurableDouble("Roller KP", 0.2);
    ConfigurableDouble roller_kI = new ConfigurableDouble("Roller KI", 0);
    ConfigurableDouble roller_kD = new ConfigurableDouble("Roller KD", 0);

    @Override
    public void updateInputs(IntakeState state) {
        state.armAngle = m_armMotor.getPosition().getValue().times(IntakeConstants.ARM_MOTOR_GEAR_RATIO);
        state.armMotorCurrent = m_armMotor.getStatorCurrent(false).getValue();

        state.rollerVelocity = m_rollerMotor.getVelocity().getValue();
        state.rollerMotorCurrent = m_rollerMotor.getStatorCurrent().getValue();

        IntakeConstants.ARM_PID.kP = arm_kP.get();
        IntakeConstants.ARM_PID.kI = arm_kI.get();
        IntakeConstants.ARM_PID.kD = arm_kD.get();
        m_armMotor.getConfigurator().apply(IntakeConstants.ARM_MOTOR_CONFIG);

        IntakeConstants.ROLLER_PID.kP = roller_kP.get();
        IntakeConstants.ROLLER_PID.kI = roller_kI.get();
        IntakeConstants.ROLLER_PID.kD = roller_kD.get();
        m_rollerMotor.getConfigurator().apply(IntakeConstants.ROLLER_MOTOR_CONFIG);
    }
}
