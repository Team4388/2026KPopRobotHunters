package frc4388.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.utility.status.FaultReporter;

public class Intake extends SubsystemBase {
    public IntakeIO io;
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
        Extended,
        Retracted,
    }

    private IntakeMode mode = IntakeMode.Extended;

    public void setMode(IntakeMode mode) {
        this.mode = mode;
    }

    public IntakeMode getMode() {
        return mode;
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

        io.updateInputs(state);

        switch (mode) {
            case Extended:
                io.setArmAngle(state, Rotations.of(IntakeConstants.ARM_LIMIT_EXTENDED.get()));
                io.setRollerVelocity(state, RotationsPerSecond.of(IntakeConstants.ROLLER_ACTIVE.get()));
                break;
            case Retracted:
                io.setArmAngle(state, Rotations.of(IntakeConstants.ARM_LIMIT_RETRACTED.get()));
                io.setRollerVelocity(state, RotationsPerSecond.of(0));
                break;
        }

    }
}


