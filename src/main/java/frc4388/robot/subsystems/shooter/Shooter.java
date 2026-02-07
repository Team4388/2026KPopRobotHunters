package frc4388.robot.subsystems.shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.subsystems.intake.IntakeConstants;
import frc4388.robot.subsystems.shooter.ShooterIO.ShooterState;

public class Shooter extends SubsystemBase {
    ShooterIO io;
    ShooterStateAutoLogged state = new ShooterStateAutoLogged();

    // Supplier<Pose2d> m_swervePoseSupplier;


    public Shooter(
        ShooterIO io
        // Supplier<Pose2d> swervePoseSupplier
    ) {
        this.io = io;
        // this.m_swervePoseSupplier = swervePoseSupplier;
    }

    public enum FieldZone {
        // The robot should aim at the hub
        InShootZone,
        // The robot should aim towards the wall
        AimAtWall,
        
    }

    
    public enum ShooterMode {
        //Shooter is at speed it fires balls
        Active,
        //Shooter is at a resting velocity
        Resting,
        //Shooter is inactive (Off)
        Inactive,
    }

    public void setMode(ShooterMode mode) {
        switch (mode) {
            case Active:
                io.setMotor1Velocity(state, ShooterConstants.SHOOTER_ACTIVE_VELOCITY);
                io.setMotor2Velocity(state, ShooterConstants.SHOOTER_ACTIVE_VELOCITY);
                io.setIndexerVelocity(state, ShooterConstants.INDEXER_ACTIVE_VELOCITY);
                break;
            case Resting:
                io.setMotor1Velocity(state, ShooterConstants.SHOOTER_RESTING_VELOCITY);
                io.setMotor2Velocity(state, ShooterConstants.SHOOTER_RESTING_VELOCITY);
                io.setIndexerVelocity(state, ShooterConstants.INDEXER_INACTIVE_VELOCITY);
                break;
            case Inactive:
                io.setMotor1Velocity(state, ShooterConstants.SHOOTER_RESTING_VELOCITY);
                io.setMotor2Velocity(state, ShooterConstants.SHOOTER_RESTING_VELOCITY);
                io.setIndexerVelocity(state, ShooterConstants.INDEXER_INACTIVE_VELOCITY);
                break;
        }
    }

    // Calculate what should be done based off of the position of the robot
    // TODO: Implement field zones
    public FieldZone getTarget(Pose2d position) {
        return FieldZone.InShootZone;
    }


    @Override
    public void periodic() {
        
        

        // FaultReporter.register(this); // TODO Implement fault reporter


        Logger.processInputs("Shooter", state);

        // Pose2d pose = m_swervePoseSupplier.get();
        // Angle robotRot = pose.getRotation().getMeasure();

        io.updateInputs(state);

    }
}
