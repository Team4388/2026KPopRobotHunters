package frc4388.robot.subsystems.climber;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    ClimberIO io;
    ClimberStateAutoLogged state = new ClimberStateAutoLogged();

    // Supplier<Pose2d> m_swervePoseSupplier;

    public Climber(
        ClimberIO io
        // Supplier<Pose2d> swervePoseSupplier
    ) {
        this.io = io;
        // this.m_swervePoseSupplier = swervePoseSupplier;
    }

    // public enum ClimberMode {
    //     Up,
    //     Down,
    // }

    // public void setMode(IntakeMode mode) {
    //     switch (mode) {
    //         case Up:
    //             io.setArmAngle(state, IntakeConstants.ARM_LIMIT_UPPER);
    //             io.setRollerVelocity(state, IntakeConstants.ROLLER_STOP);
    //             break;
    //         case Down:
    //             io.setArmAngle(state, IntakeConstants.ARM_LIMIT_LOWER);
    //             io.setRollerVelocity(state, IntakeConstants.ROLLER_MAX_VELOCITY);
    //             break;
    //     }
    // }



    @Override
    public void periodic() {
        
        

        // FaultReporter.register(this); // TODO Implement fault reporter


        Logger.processInputs("Climber", state);

        io.updateInputs(state);

    }
}


