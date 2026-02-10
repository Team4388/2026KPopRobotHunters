package frc4388.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public ShooterIO io;
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
                io.setShooterVelocity(state, RotationsPerSecond.of(ShooterConstants.SHOOTER_ACTIVE_VELOCITY.get()));
                // io.setMotor2Velocity(state, ShooterConstants.SHOOTER_ACTIVE_VELOCITY);
                io.setIndexerVelocity(state, RotationsPerSecond.of(ShooterConstants.INDEXER_ACTIVE_VELOCITY.get()));
                break;
            case Resting:
                io.setShooterVelocity(state, RotationsPerSecond.of(ShooterConstants.SHOOTER_RESTING_VELOCITY.get()));
                // io.setMotor2Velocity(state, ShooterConstants.SHOOTER_RESTING_VELOCITY);
                io.setIndexerVelocity(state, RotationsPerSecond.of(0));
                break;
            case Inactive:
                io.setShooterVelocity(state, RotationsPerSecond.of(ShooterConstants.SHOOTER_RESTING_VELOCITY.get()));
                // io.setMotor2Velocity(state, ShooterConstants.SHOOTER_RESTING_VELOCITY);
                io.setIndexerVelocity(state, RotationsPerSecond.of(0));
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
