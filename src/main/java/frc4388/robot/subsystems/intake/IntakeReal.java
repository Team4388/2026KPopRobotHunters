package frc4388.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

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
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class IntakeReal implements IntakeIO {

    SparkMax m_armMotor;
    SparkMax m_rollerMotor;
    DutyCycleEncoder encoder;

    public IntakeReal(
        // DigitalInput armLimitSwitch,
        SparkMax armMotor,
        SparkMax rollerMotor
    ) {
        // m_angleMotor = angleMotor;
        // m_pitchMotor = pitchMotor;
        m_armMotor = armMotor;
        m_rollerMotor = rollerMotor;
        // m_armLimitSwitch = armLimitSwitch;
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
        // m_armMotor.setControl(
        //     armPosition
        //         .withPosition(motorAngle)
        //         .withLimitReverseMotion(!m_armLimitSwitch.get())
        //     );

    }

    @Override
    public void stopArm(){
        m_armMotor.set(0);
        // m_rollerMotor.set(0);
    }

    @Override
    public void armOutput(double percentOutput){
        m_armMotor.set(percentOutput);
    }

    @Override
    public void updateInputs(IntakeState state) {
        state.armAngle = Rotations.of(m_armMotor.getEncoder().getPosition()).div(IntakeConstants.ARM_MOTOR_GEAR_RATIO);
        state.armMotorVelocity = RotationsPerSecond.of(m_armMotor.getEncoder().getVelocity()).div(IntakeConstants.ARM_MOTOR_GEAR_RATIO);
        // state.armMotorAcceleration = RotationsPerSecondPerSecond.of(m_armMotor.getEncoder().ge);
        state.armMotorCurrent = Amps.of(m_armMotor.getOutputCurrent());

        state.rollerOutput = m_rollerMotor.get();
        state.rollerMotorCurrent = Amps.of(m_rollerMotor.getOutputCurrent());


        // if(state.retractedLimit) {
        //     // Set the arm motor to be zero if the limit switch is pressed
        //     m_armMotor.setPosition(0., 0); 
        // }
    }

    @Override 
    public void updateGains() {

        // IntakeConstants.ARM_PID.kP = IntakeConstants.arm_kP.get();
        // IntakeConstants.ARM_PID.kI = IntakeConstants.arm_kI.get();
        // IntakeConstants.ARM_PID.kD = IntakeConstants.arm_kD.get();
        // m_armMotor.getConfigurator().apply(IntakeConstants.ARM_PID);

    }
}
