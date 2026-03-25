package frc4388.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeReal implements IntakeIO {

    TalonFX m_armMotor;
    SparkMax m_rollerMotor;
    DigitalInput m_armLimitSwitch;

    PositionDutyCycle armPosition = new PositionDutyCycle(0);
    DutyCycleOut armPercentOutput = new DutyCycleOut(0);

    public IntakeReal(
        DigitalInput armLimitSwitch,
        TalonFX armMotor,
        SparkMax rollerMotor
    ) {
        // m_angleMotor = angleMotor;
        // m_pitchMotor = pitchMotor;
        m_armMotor = armMotor;
        m_rollerMotor = rollerMotor;
        m_armLimitSwitch = armLimitSwitch;

        // Apply the configs
        m_armMotor.getConfigurator().apply(IntakeConstants.ARM_PID);
        m_armMotor.getConfigurator().apply(IntakeConstants.ARM_MOTOR_CONFIG);
        // m_rollerMotor.getConfigurator().apply(IntakeConstants.ROLLER_MOTOR_CONFIG);

        armPosition.Slot = 0;
        // rollerVelocity.Slot = 0;
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
    public void setRollerOutput(IntakeState state, double rollerOutput) {
        state.rollerTargetOutput = rollerOutput;

        m_rollerMotor.set(rollerOutput);
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
        m_armMotor.setControl(
            armPosition
                .withPosition(motorAngle)
                .withLimitReverseMotion(!m_armLimitSwitch.get())
            );

    }

    @Override
    public void testSetArmAngle(IntakeState state, Angle angle){
        state.armTargetAngle = angle;
        Angle motorAngle = angle.times(IntakeConstants.ARM_MOTOR_GEAR_RATIO);

        final TrapezoidProfile m_profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(80, 160)
        );

        // Final target of motorAngle rot, 0 rps
        // Convert the Angle to a numeric degree value before creating the profile state
        TrapezoidProfile.State m_goal = new TrapezoidProfile.State(motorAngle.in(Rotations), 0);
        TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

        // create a position closed-loop request, voltage output, slot 0 configs
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

        // calculate the next profile setpoint
        m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);

        // send the request to the device
        m_request.Position = m_setpoint.position;
        m_request.Velocity = m_setpoint.velocity;
        m_armMotor.setControl(m_request);
    }

    @Override
    public void stopArm(){
        m_armMotor.set(0);
        // m_rollerMotor.set(0);
    }

    @Override
    public void armOutput(double percentOutput){
        m_armMotor.setControl(
            armPercentOutput.withOutput(percentOutput)
                .withLimitReverseMotion(!m_armLimitSwitch.get())
            );
    }

    @Override
    public void updateInputs(IntakeState state) {
        state.armAngle = m_armMotor.getPosition().getValue().div(IntakeConstants.ARM_MOTOR_GEAR_RATIO);
        state.armMotorCurrent = m_armMotor.getStatorCurrent().getValue();
        state.rollerOutput = m_rollerMotor.get();
        state.rollerMotorCurrent = Amps.of(m_rollerMotor.getOutputCurrent());
        state.retractedLimit = !m_armLimitSwitch.get();
        
        state.armMotorVelocity = m_armMotor.getVelocity().getValue();
        state.armMotorAcceleration = m_armMotor.getAcceleration().getValue();

        if(state.retractedLimit) {
            // Set the arm motor to be zero if the limit switch is pressed
            m_armMotor.setPosition(0., 0); 
        }
    }

    @Override 
    public void updateGains() {

        IntakeConstants.ARM_PID.kP = IntakeConstants.arm_kP.get();
        IntakeConstants.ARM_PID.kI = IntakeConstants.arm_kI.get();
        IntakeConstants.ARM_PID.kD = IntakeConstants.arm_kD.get();
        m_armMotor.getConfigurator().apply(IntakeConstants.ARM_PID);

    }
}
