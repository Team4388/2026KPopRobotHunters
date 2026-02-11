package frc4388.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.intake.Intake;
import frc4388.robot.subsystems.intake.Intake.IntakeMode;
import frc4388.robot.subsystems.swerve.SwerveDrive;

public class Shooter extends SubsystemBase {
    public ShooterIO io;
    ShooterStateAutoLogged state = new ShooterStateAutoLogged();

    SwerveDrive m_SwerveDrive;
    Intake m_Intake;
    LED m_robotLED;
    

    // Supplier<Pose2d> m_swervePoseSupplier;


    public Shooter(
        ShooterIO io,
        SwerveDrive swerveDrive,
        Intake intake,
        LED robotLED
    ) {
        this.io = io;
        this.m_SwerveDrive = swerveDrive;
        this.m_Intake = intake;
        this.m_robotLED = robotLED;
        // this.m_swervePoseSupplier = swervePoseSupplier;
    }

    public enum FieldZone {
        // The robot should aim at the hub
        InShootZone,
        // The robot should aim towards the wall
        AimAtWall,
        
    }

    
    public enum ShooterMode {
        // Shooter is actively shooting
        Shooting,
        // Shooter is going to fire soon
        Ready,
        // Not ready to shoot
        NotReady,
    }

    private ShooterMode mode = ShooterMode.NotReady;

    // public void setMode(ShooterMode mode) {
    //     this.mode = mode;
    // }

    public void setShooterReady() {
        if(this.mode == ShooterMode.NotReady) {
            this.mode = ShooterMode.Ready;
        }
    }
    
    public void setShooterNotReady() {
        this.mode = ShooterMode.NotReady;
    }

    @AutoLogOutput
    public ShooterMode getMode() {
        return mode;
    }



    @Override
    public void periodic() {
        // FaultReporter.register(this); // TODO Implement fault reporter

        Logger.processInputs("Shooter", state);
        io.updateInputs(state);

        if(this.mode != ShooterMode.NotReady) {
            double badRobotSpeed = Math.sqrt(Math.pow(m_SwerveDrive.chassisSpeeds.vxMetersPerSecond,2) + Math.pow(m_SwerveDrive.chassisSpeeds.vyMetersPerSecond,2));
            double badAngSpeed = Math.abs(m_SwerveDrive.chassisSpeeds.omegaRadiansPerSecond);
            double badShooterVelocity = Math.abs(state.motor1Velocity.in(RotationsPerSecond) + state.motor2Velocity.in(RotationsPerSecond)) / 2;
            boolean intakeBad = m_Intake.getMode() == IntakeMode.Extended;



            // TODO: get if the robot is within the correct distance of the hub
        }

        switch (mode) {
            case Shooting:
                io.setShooterVelocity(state, RotationsPerSecond.of(ShooterConstants.SHOOTER_ACTIVE_VELOCITY.get()));
                io.setIndexerVelocity(state, RotationsPerSecond.of(ShooterConstants.INDEXER_FORWARD_VELOCITY.get()));
                break;
            case Ready:
                io.setShooterVelocity(state, RotationsPerSecond.of(ShooterConstants.SHOOTER_ACTIVE_VELOCITY.get()));
                io.setIndexerVelocity(state, RotationsPerSecond.of(ShooterConstants.INDEXER_REVERSE_VELOCITY.get()));
                break;
            case NotReady:
                io.setShooterVelocity(state, RotationsPerSecond.of(ShooterConstants.SHOOTER_RESTING_VELOCITY.get()));
                io.setIndexerVelocity(state, RotationsPerSecond.of(ShooterConstants.INDEXER_REVERSE_VELOCITY.get()));
                break;
        }

    }
}
