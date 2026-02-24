package frc4388.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.constants.Constants;
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.intake.Intake;
import frc4388.robot.subsystems.swerve.SwerveDrive;
import frc4388.utility.compute.FieldPositions;

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
        Shooting,
        Feeding,
        Idle
    }

    private ShooterMode mode = ShooterMode.Idle;
    private boolean shooterButtonReady = false;

    public void setShooterShooting() {
        this.mode = ShooterMode.Shooting;
    }

    public void setShooterFeeding() {
        this.mode = ShooterMode.Feeding;
    }
    
    public void setShooterIdle() {
        this.mode = ShooterMode.Idle;
    }


    public void setShooterShoot() {
        shooterButtonReady = true;
    }

    public void setShooterNOTShoot() {
        shooterButtonReady = false;
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


        ChassisSpeeds speed = m_SwerveDrive.chassisSpeeds;
        double XYSpeed = Math.sqrt(Math.pow(speed.vxMetersPerSecond,2) + Math.pow(speed.vyMetersPerSecond,2));
        double AngSpeed  = Math.abs(speed.omegaRadiansPerSecond * (180/Math.PI));

        Pose2d robotPose2d = m_SwerveDrive.getPose2d();
        //
        double distanceToHub = (robotPose2d.getTranslation().minus(FieldPositions.HUB_POSITION).getNorm());
        //Center of hub to cameras in inches
        Logger.recordOutput("Hub Dist", distanceToHub);



        boolean driverError = 
            // XYSpeed <= ShooterConstants.ROBOT_SPEED_TOLERANCE.get() |
            // AngSpeed <= ShooterConstants.ROBOT_ANG_SPEED_TOLERANCE.get() |
            distanceToHub <= ShooterConstants.ROBOT_MIN_HUB.get() | 
            distanceToHub >= ShooterConstants.ROBOT_MAX_HUB.get();


        double shooterSpeed = Math.abs(state.motor1Velocity.in(RotationsPerSecond) + state.motor2Velocity.in(RotationsPerSecond)) / 2;
        double shooterSpeedTarget = Math.abs(state.motor1TargetVelocity.in(RotationsPerSecond) + state.motor2TargetVelocity.in(RotationsPerSecond)) / 2;

        boolean badShooterVelocity = Math.abs(shooterSpeed - shooterSpeedTarget) > ShooterConstants.SHOOTER_SPEED_TOLERANCE.get();

        switch (mode) {
            case Shooting:
                io.setShooterVelocity(state, ShooterConstants.getTargetShooterSpeed(distanceToHub));

                int bitmask = (
                    (shooterButtonReady ? 1 : 0) +
                    (badShooterVelocity ? 2 : 0) +
                    (driverError ? 4 : 0)
                );

                switch (bitmask) {
                    case 0b000: // No errors but button is not pressed
                        io.setIndexerOutput(state, ShooterConstants.INDEXER_REVERSE_OUTPUT.get());
                        m_robotLED.setMode(Constants.LEDConstants.OPREADY);
                        break;

                    case 0b001: // No errors and shoot button is pressed
                        io.setIndexerOutput(state, ShooterConstants.INDEXER_FORWARD_OUTPUT.get());
                        m_robotLED.setMode(Constants.LEDConstants.OPREADY);
                        break;

                    case 0b010: // Bad shooter velocity, button is not pressed
                    case 0b011: // Bad shooter velocty, button is pressed
                        io.setIndexerOutput(state, ShooterConstants.INDEXER_REVERSE_OUTPUT.get());
                        m_robotLED.setMode(Constants.LEDConstants.BAD_FLYWEEL);
                        break;

                    case 0b100: // Driver error, button is not pressed
                    case 0b101: // Driver error, button is pressed
                        io.setIndexerOutput(state, ShooterConstants.INDEXER_REVERSE_OUTPUT.get());
                        m_robotLED.setMode(Constants.LEDConstants.OPREADY_BADPHYS);
                        break;

                    case 0b110: // Driver error, bad shooter vel, button is not pressed
                    case 0b111: // Driver error, bad shooter vel, button is pressed
                        io.setIndexerOutput(state, ShooterConstants.INDEXER_REVERSE_OUTPUT.get());
                        m_robotLED.setMode(Constants.LEDConstants.BAD_FLYWEEL_BADPHYS);
                        break;
                }
                break;
            case Feeding:
                io.setShooterVelocity(state, RotationsPerSecond.of(ShooterConstants.SHOOTER_FEED_VELOCITY.get()));

                int bitmask2 = (
                    (shooterButtonReady ? 1 : 0) +
                    (badShooterVelocity ? 2 : 0)
                );

                switch (bitmask2) {
                    case 0b000: // No errors but button is not pressed
                        io.setIndexerOutput(state, ShooterConstants.INDEXER_REVERSE_OUTPUT.get());
                        m_robotLED.setMode(Constants.LEDConstants.OPREADY_FEED);
                        break;

                    case 0b001: // No errors and shoot button is pressed
                        io.setIndexerOutput(state, ShooterConstants.INDEXER_FORWARD_OUTPUT.get());
                        m_robotLED.setMode(Constants.LEDConstants.OPREADY_FEED);
                        break;

                    case 0b010: // Bad shooter velocity, button is not pressed
                    case 0b011: // Bad shooter velocty, button is pressed
                        io.setIndexerOutput(state, ShooterConstants.INDEXER_REVERSE_OUTPUT.get());
                        m_robotLED.setMode(Constants.LEDConstants.BAD_FLYWEEL);
                        break;

                    // case 0b100: // Driver error, button is not pressed
                    // case 0b101: // Driver error, button is pressed
                    //     m_robotLED.setMode(Constants.LEDConstants.BAD_FLYWEEL);
                    //     io.setIndexerOutput(state, ShooterConstants.INDEXER_REVERSE_OUTPUT.get());
                    //     break;

                    // case 0b110: // Driver error, bad shooter vel, button is not pressed
                    // case 0b111: // Driver error, bad shooter vel, button is pressed
                    //     m_robotLED.setMode(Constants.LEDConstants.BAD_FLYWEEL_BADPHYS);
                    //     io.setIndexerOutput(state, ShooterConstants.INDEXER_REVERSE_OUTPUT.get());
                    //     break;
                }

                break;
            case Idle:
                io.setShooterVelocity(state, RotationsPerSecond.of(ShooterConstants.SHOOTER_RESTING_VELOCITY.get()));
                io.setIndexerOutput(state, ShooterConstants.INDEXER_REVERSE_OUTPUT.get());
                m_robotLED.setMode(Constants.LEDConstants.DEFAULT_PATTERN);
                break;
            }

    }
}
