package frc4388.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotation;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.utility.status.FaultReporter;

public class Intake extends SubsystemBase {
    IntakeIO io;
    IntakeStateAutoLogged state = new IntakeStateAutoLogged();

    Supplier<Pose2d> m_swervePoseSupplier;

    public Intake(
        IntakeIO io
        // Supplier<Pose2d> swervePoseSupplier
    ) {
        this.io = io;
        // this.m_swervePoseSupplier = swervePoseSupplier;
    }

    public enum IntakeMode {
        Up,
        Down,
    }

    public void setMode(IntakeMode mode) {
        switch (mode) {
            case Up:
                // io.setArmAngle(state, IntakeConstants.ARM_LIMIT_UPPER);
                io.setRollerVelocity(state, IntakeConstants.ROLLER_STOP);
                break;
            case Down:
                // io.setArmAngle(state, IntakeConstants.ARM_LIMIT_LOWER);
                io.setRollerVelocity(state, IntakeConstants.ROLLER_MAX_VELOCITY);
                break;
        }
    }


    // public enum FieldZone {
    //     // The robot should aim at the hub
    //     InShootZone,
    //     // The robot should aim towards the wall
    //     AimAtWall,
        
    // }

    // // Calculate what should be done based off of the position of the robot
    // // TODO: Implement field zones
    // public FieldZone getTarget(Pose2d position) {
    //     return FieldZone.InShootZone;
    // }

    @Override
    public void periodic() {
        
        

        // FaultReporter.register(this); // TODO Implement fault reporter


        Logger.processInputs("Intake", state);

        Pose2d pose = m_swervePoseSupplier.get();
        Angle robotRot = pose.getRotation().getMeasure();

        io.updateInputs(state);

    }
}


