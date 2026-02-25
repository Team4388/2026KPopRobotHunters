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
import frc4388.utility.compute.TimesNegativeOne;
import frc4388.utility.structs.Gains;

public class AutoAlign extends Command {

    private PID xPID = new PID(AutoConstants.XY_GAINS, 0);
    private PID yPID = new PID(AutoConstants.XY_GAINS, 0);
    private Pose2d targetpos;

    SwerveDrive swerveDrive;
    Vision vision;
    boolean waitVelocityZero;

    public AutoAlign(SwerveDrive swerveDrive, Vision vision, boolean waitVelocityZero) {
        this.swerveDrive = swerveDrive;
        this.vision = vision;
        this.waitVelocityZero = waitVelocityZero && false;
        addRequirements(swerveDrive);
    }

    public static double tagRelativeXError = -1;
    private static void setTagRelativeXError(double val){
        tagRelativeXError = val;
    }

    @Override
    public void initialize() {
        xPID.initialize();
        yPID.initialize();
        this.targetpos = new Pose2d(FieldPositions.HUB_POSITION, new Rotation2d(0));
    }
    
    double xerr;
    double yerr;
    double roterr;

    double xoutput;
    double youtput;
    double rotoutput;

    @Override
    public void execute() {
        xerr = TimesNegativeOne.invert(targetpos.getX() - vision.getPose2d().getX(), TimesNegativeOne.XAxis);
        yerr = TimesNegativeOne.invert(targetpos.getY() - vision.getPose2d().getY(), !TimesNegativeOne.YAxis);
        
        roterr = ((targetpos.getRotation().getDegrees() - vision.getPose2d().getRotation().getDegrees()));

        if(roterr > 180){
            roterr -= 360;
        }else if(roterr < -180){
            roterr += 360;
        }

    

        SmartDashboard.putNumber("PID X Error", xerr);
        SmartDashboard.putNumber("PID Y Error", yerr);
        SmartDashboard.putNumber("PID Rot Error", roterr);

        xoutput = xPID.update(xerr);
        youtput = yPID.update(yerr);

        xoutput *= Math.abs(xerr) < AutoConstants.XY_TOLERANCE ? 0 : 1;
        youtput *= Math.abs(yerr) < AutoConstants.XY_TOLERANCE ? 0 : 1;
        


        Translation2d leftStick = new Translation2d(
            Math.max(Math.min(-youtput, 1), -1),
            Math.max(Math.min(-xoutput, 1), -1)
        );

        

        setTagRelativeXError(
            new Translation2d(xerr, yerr).getAngle()
            .rotateBy(targetpos.getRotation())
            .getCos());

        swerveDrive.driveRelativeAngle(leftStick, targetpos.getRotation());
    }

    @Override
    public final boolean isFinished() {
        boolean finished = (
            (Math.abs(xerr) < AutoConstants.XY_TOLERANCE || Math.abs(xoutput) <= AutoConstants.MIN_XY_PID_OUTPUT) && 
            (Math.abs(yerr) < AutoConstants.XY_TOLERANCE || Math.abs(youtput) <= AutoConstants.MIN_XY_PID_OUTPUT) && 
            (Math.abs(roterr) < AutoConstants.ROT_TOLERANCE) &&
            (!waitVelocityZero || swerveDrive.lastOdomSpeed < AutoConstants.VELOCITY_THRESHHOLD)
        );

        if(finished)
            swerveDrive.softStop();

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
