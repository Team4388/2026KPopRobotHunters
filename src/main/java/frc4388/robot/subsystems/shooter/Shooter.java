package frc4388.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
        // Shooter is actively shooting
        Shooting,
        // Shooter is going to fire soon
        Ready,

        ShootingFeeder,
        ReadyFeeder,

        // Not ready to shoot
        NotReady,
    }

    private ShooterMode mode = ShooterMode.NotReady;
    private boolean shooterButtonReady = false;

    public void setShooterReady() {
        this.mode = ShooterMode.Ready;
    }


    public void setShooterReadyFeeder() {
        this.mode = ShooterMode.ReadyFeeder;
    }
    
    public void setShooterNotReady() {
        this.mode = ShooterMode.NotReady;
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


        if(this.mode != ShooterMode.NotReady) {
            // TODO: get if the robot is within the angle of the hub

            boolean driverError = 
                // XYSpeed <= ShooterConstants.ROBOT_SPEED_TOLERANCE.get() |
                // AngSpeed <= ShooterConstants.ROBOT_ANG_SPEED_TOLERANCE.get() |
                distanceToHub <= ShooterConstants.ROBOT_MIN_HUB.get() | 
                distanceToHub >= ShooterConstants.ROBOT_MAX_HUB.get();

            double shooterSpeed = Math.abs(state.motor1Velocity.in(RotationsPerSecond) + state.motor2Velocity.in(RotationsPerSecond)) / 2;
            double shooterSpeedTarget = Math.abs(state.motor1TargetVelocity.in(RotationsPerSecond) + state.motor2TargetVelocity.in(RotationsPerSecond)) / 2;

            boolean badShooterVelocity = Math.abs(shooterSpeed - shooterSpeedTarget) > ShooterConstants.SHOOTER_SPEED_TOLERANCE.get();
        //     boolean intakeBad = m_Intake.getMode() == IntxakeMode.Extended;


            int bitmask = (driverError ? 1 : 0) + (badShooterVelocity ? 2 : 0) + (this.mode == ShooterMode.ReadyFeeder ? 4 : 0);
            switch (bitmask) {
                case 0b000: // No Errors
                    m_robotLED.setMode(Constants.LEDConstants.OPREADY);
                    break;
                case 0b001: // No op err, yes driver err
                    m_robotLED.setMode(Constants.LEDConstants.OPREADY_BADPHYS);
                    break;

                case 0b010:
                case 0b110: // Bad flywheel, no driver err
                    m_robotLED.setMode(Constants.LEDConstants.BAD_FLYWEEL);
                    break;

                case 0b011:
                case 0b111: // Bad flywheel, yes driver err
                    m_robotLED.setMode(Constants.LEDConstants.BAD_FLYWEEL_BADPHYS);
                    break;

                case 0b100:
                    m_robotLED.setMode(Constants.LEDConstants.OPREADY_FEED);
                    break;
                case 0b101:
                    m_robotLED.setMode(Constants.LEDConstants.OPREADY_FEED_BADPHYS);
                    break;
            }

        //     // We set the shooter mode to ready if there are no errors
            mode = (
                bitmask == 0 ?
                ShooterMode.Shooting :
                ShooterMode.Ready
            );

        } else {
            m_robotLED.setMode(Constants.LEDConstants.DEFAULT_PATTERN);

        }
        
        


        switch (mode) {
            case Shooting:
                io.setShooterVelocity(state, ShooterConstants.getTargetShooterSpeed(distanceToHub));

                if(shooterButtonReady) {
                    io.setIndexerOutput(state, ShooterConstants.INDEXER_FORWARD_OUTPUT.get());
                } else {
                    io.setIndexerOutput(state, ShooterConstants.INDEXER_REVERSE_OUTPUT.get());
                }
                break;

            case Ready:
                io.setShooterVelocity(state, ShooterConstants.getTargetShooterSpeed(distanceToHub));
                io.setIndexerOutput(state, ShooterConstants.INDEXER_REVERSE_OUTPUT.get());
                break;

            case ShootingFeeder:
                io.setShooterVelocity(state, RotationsPerSecond.of(ShooterConstants.SHOOTER_FEED_VELOCITY.get()));
                
                if(shooterButtonReady) {
                    io.setIndexerOutput(state, ShooterConstants.INDEXER_FORWARD_OUTPUT.get());
                } else {
                    io.setIndexerOutput(state, ShooterConstants.INDEXER_REVERSE_OUTPUT.get());
                }
                break;

            case ReadyFeeder:
                io.setShooterVelocity(state, ShooterConstants.getTargetShooterSpeed(distanceToHub));
                io.setIndexerOutput(state, ShooterConstants.INDEXER_REVERSE_OUTPUT.get());
                break;

            case NotReady:
                io.setShooterVelocity(state, RotationsPerSecond.of(ShooterConstants.SHOOTER_RESTING_VELOCITY.get()));
                io.setIndexerOutput(state, ShooterConstants.INDEXER_REVERSE_OUTPUT.get());
                break;
        }

    }
}
