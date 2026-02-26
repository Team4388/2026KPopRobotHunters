package frc4388.robot.commands.alignment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc4388.robot.constants.Constants.AutoConstants;
import frc4388.robot.subsystems.swerve.SwerveDrive;
import frc4388.robot.subsystems.vision.Vision;
import frc4388.utility.compute.FieldPositions;
import frc4388.utility.structs.Gains;

public class AutoAlign extends Command {

    private PID rotPID = new PID(AutoConstants.ROT_GAINS, 0);
    private Pose2d targetpos;

    SwerveDrive swerveDrive;
    Vision vision;

    public AutoAlign(SwerveDrive swerveDrive, Vision vision, Pose2d targetpos) {
        this.swerveDrive = swerveDrive;
        this.vision = vision;
        this.targetpos = targetpos;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        rotPID.initialize();
        //this.targetpos = new Pose2d(FieldPositions.HUB_POSITION, new Rotation2d(0));
    }
    

    double roterr;

    double rotoutput;

    @Override
    public void execute() {


        Pose2d robotPose = vision.getPose2d();
        if (robotPose == null) return;

        double dx = targetpos.getX() - robotPose.getX();
        double dy = targetpos.getY() - robotPose.getY();

        Rotation2d desiredHeading = new Rotation2d(Math.atan2(dy, dx));

        roterr = desiredHeading.getDegrees() - robotPose.getRotation().getDegrees();
        if (roterr > 180) roterr -= 360;
        if (roterr < -180) roterr += 360;

        SmartDashboard.putNumber("PID Rot Error", roterr);

        swerveDrive.driveRelativeAngle(new Translation2d(0.0, 0.0), desiredHeading);
     }

    @Override
    public final boolean isFinished() {
        boolean finished = Math.abs(roterr) < AutoConstants.ROT_TOLERANCE;
        if (finished) {
            swerveDrive.softStop();
        }
        return finished;
    }

    private class PID {
        protected Gains  gains;
        private   double output    = 0;


        /** Creates a new PelvicInflammatoryDisease. */
        public PID(Gains gains, double tolerance) {
            this.gains     = gains;
        }

        // Called when the command is initially scheduled.
        public final void initialize() {
            output = 0;
        }

        private double prevError, cumError = 0;
        
        // Called every time the scheduler runs while the command is scheduled.
        public double update(double error) {
            cumError += error * .02; // 20 ms
            double delta = error - prevError;

            output = error * gains.kP;
            output += cumError * gains.kI;
            output += delta * gains.kD;
            output += gains.kF;

            return output;
        }
    }
    
}
