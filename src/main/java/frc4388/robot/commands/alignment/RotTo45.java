package frc4388.robot.commands.alignment;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc4388.robot.subsystems.swerve.SwerveDrive;

public class RotTo45 extends Command {

    SwerveDrive m_SwerveDrive;
    Rotation2d targetAngle;


    public RotTo45(SwerveDrive swerveDrive) {
        m_SwerveDrive = swerveDrive;
        
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        targetAngle = new Rotation2d();
    }

    @Override
    public void execute() {
        m_SwerveDrive.driveRelativeAngle(new Translation2d(), targetAngle);
    }

    @Override
    public boolean isFinished() {
        Rotation2d curRot = m_SwerveDrive.getPose2d().getRotation();
        // TODO: Tune
        return Math.abs(curRot.getDegrees() - targetAngle.getDegrees()) < 5;
    }
    
}
