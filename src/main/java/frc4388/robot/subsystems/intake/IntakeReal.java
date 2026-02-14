package frc4388.robot.subsystems.intake;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

public class IntakeReal implements IntakeIO {

    TalonFX m_armMotor;
    TalonFX m_rollerMotor;
    DigitalInput m_armLimitSwitch;

    PositionDutyCycle armPosition = new PositionDutyCycle(0);
    VelocityDutyCycle rollerVelocity = new VelocityDutyCycle(0);

    public IntakeReal(
        DigitalInput armLimitSwitch,
        TalonFX armMotor,
        TalonFX rollerMotor
    ) {
        // m_angleMotor = angleMotor;
        // m_pitchMotor = pitchMotor;
        m_armMotor = armMotor;
        m_rollerMotor = rollerMotor;
        m_armLimitSwitch = armLimitSwitch;

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

        if(angularVelocity.baseUnitMagnitude() == 0) {
            m_rollerMotor.set(0);
            return;
        }

        // (REAL_ROT / SEC) * (MOTOR_ROT / REAL_ROT) = (MOTOR_ROT / SEC)
        AngularVelocity motorSpeed = angularVelocity.times(IntakeConstants.ROLLER_MOTOR_GEAR_RATIO);

        // m_rollerMotor.set(motorSpeed);
        // VelocityDutyCycle velocity = new VelocityDutyCycle(motorSpeed);
        m_rollerMotor.setControl(rollerVelocity.withVelocity(motorSpeed));
    }

    @Override
    public void setArmAngle(IntakeState state, Angle angle) {
        state.armTargetAngle = angle;
        // Assume that the angle is always accurate, because I think we will use a shaft encoder
        // Assume that 0 degrees = forwards. Might need an offset here

        // angle = clampAng(angle, IntakeConstants.ARM_LIMIT_RETRACTED, IntakeConstants.ARM_LIMIT_EXTENDED);

        // (REAL_ROT) * (MOTOR_ROT / REAL_ROT) = MOTOR_ROT
        Angle motorAngle = angle.times(IntakeConstants.ARM_MOTOR_GEAR_RATIO);
        
        // PositionDutyCycle posRequest = new PositionDutyCycle(motorTargetAngle);
        m_armMotor.setControl(armPosition.withPosition(motorAngle));
    }
    @Override
    public void stopArm(){
        m_armMotor.set(0);
    }
    @Override
    public void armExtend(double percentOutput){
        m_armMotor.set(percentOutput);
    }

    @Override
        public void armRetract(double percentOutput){
        m_armMotor.set(percentOutput);
    }

    @Override
    public void updateInputs(IntakeState state) {
        state.armAngle = m_armMotor.getPosition().getValue().div(IntakeConstants.ARM_MOTOR_GEAR_RATIO);
        state.armMotorCurrent = m_armMotor.getStatorCurrent().getValue();
        state.retractedLimit = m_armLimitSwitch.get();
        state.rollerVelocity = m_rollerMotor.getVelocity().getValue().div(IntakeConstants.ROLLER_MOTOR_GEAR_RATIO);
        state.rollerMotorCurrent = m_rollerMotor.getStatorCurrent().getValue();
    }

    @Override 
    public void updateGains() {

        IntakeConstants.ARM_PID.kP = IntakeConstants.arm_kP.get();
        IntakeConstants.ARM_PID.kI = IntakeConstants.arm_kI.get();
        IntakeConstants.ARM_PID.kD = IntakeConstants.arm_kD.get();
        m_armMotor.getConfigurator().apply(IntakeConstants.ARM_PID);

        IntakeConstants.ROLLER_PID.kP = IntakeConstants.roller_kP.get();
        IntakeConstants.ROLLER_PID.kI = IntakeConstants.roller_kI.get();
        IntakeConstants.ROLLER_PID.kD = IntakeConstants.roller_kD.get();
        m_rollerMotor.getConfigurator().apply(IntakeConstants.ROLLER_PID);
    }
}
