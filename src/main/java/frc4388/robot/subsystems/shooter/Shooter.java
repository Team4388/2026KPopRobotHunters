package frc4388.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.constants.Constants;
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.intake.Intake;
import frc4388.robot.subsystems.intake.Intake.IntakeMode;
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
        // Not ready to shoot
        NotReady,
    }

    private ShooterMode mode = ShooterMode.NotReady;

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

            ChassisSpeeds speed = m_SwerveDrive.chassisSpeeds;
            double XYSpeed = Math.sqrt(Math.pow(speed.vxMetersPerSecond,2) + Math.pow(speed.vyMetersPerSecond,2));
            double AngSpeed  = Math.abs(speed.omegaRadiansPerSecond * (180/Math.PI));

            Pose2d robotPose2d = m_SwerveDrive.getPose2d();
            
            double distanceToHub = robotPose2d.getTranslation().minus(FieldPositions.HUB_POSITION).getNorm();

            this.mode = ShooterMode.Shooting;
            // TODO: get if the robot is within the angle of the hub

        //     boolean driverError = 
        //         XYSpeed <= ShooterConstants.ROBOT_SPEED_TOLERANCE.get() |
        //         AngSpeed <= ShooterConstants.ROBOT_ANG_SPEED_TOLERANCE.get() |
        //         distanceToHub <= ShooterConstants.ROBOT_MIN_HUB.get() | 
        //         distanceToHub >= ShooterConstants.ROBOT_MAX_HUB.get();

        //     double shooterSpeed = Math.abs(state.motor1Velocity.in(RotationsPerSecond) + state.motor2Velocity.in(RotationsPerSecond)) / 2;

        //     boolean badShooterVelocity = shooterSpeed < ShooterConstants.SHOOTER_SPEED_TOLERANCE.get();
        //     boolean intakeBad = m_Intake.getMode() == IntakeMode.Extended;

            
        //     int bitmask = (driverError ? 1 : 0) + (badShooterVelocity ? 2 : 0) + (intakeBad ? 4 : 0);
        //     switch (bitmask) {
        //         case 0b000: // No Errors
        //             m_robotLED.setMode(Constants.LEDConstants.OPREADY);
        //             break;
        //         case 0b001: // No op err, yes driver err
        //             m_robotLED.setMode(Constants.LEDConstants.OPREADY_BADPHYS);
        //             break;
        //         case 0b010: // Bad flywheel, no driver err
        //             m_robotLED.setMode(Constants.LEDConstants.BAD_FLYWEEL);
        //             break;
        //         case 0b011: // Bad flywheel, yes driver err
        //             m_robotLED.setMode(Constants.LEDConstants.BAD_FLYWEEL_BADPHYS);
        //             break;
        //         case 0b100: // Bad intake, no driver err
        //             m_robotLED.setMode(Constants.LEDConstants.INTAKE_OUT);
        //             break;
        //         case 0b101: // Bad intake, yes driver err
        //             m_robotLED.setMode(Constants.LEDConstants.INTAKE_OUT_BADPHYS);
        //             break;
        //         case 0b110: // Bad intake and shooter (intake is more important), no driver err
        //             m_robotLED.setMode(Constants.LEDConstants.INTAKE_OUT);
        //             break;
        //         case 0b111: // Bad intake and shooter (intake is more important), yes driver err
        //             m_robotLED.setMode(Constants.LEDConstants.INTAKE_OUT_BADPHYS);
        //             break;
        //     }

        //     // We set the shooter mode to ready if there are no errors
        //     mode = (
        //         bitmask == 0 ?
        //         ShooterMode.Ready :
        //         ShooterMode.NotReady
        //     );

        // }

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
    }}
