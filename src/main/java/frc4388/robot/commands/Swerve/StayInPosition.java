// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc4388.robot.commands.PID;
import frc4388.robot.subsystems.swerve.SwerveDrive;

public class StayInPosition extends PID {

  SwerveDrive drive;

  /** Creates a new StayInPosition. */
  public StayInPosition(SwerveDrive drive) {
    super(0.3, 0.0, 0.0, 0.0, 1);
    
    this.drive = drive;

    addRequirements(drive);
  }

  public void goToTargetPose(Pose2d targetPose){
    Pose2d currentPose = drive.getCurrentPose();
    double translationX = targetPose.getX() - currentPose.getX();
    double translationY = targetPose.getY() - currentPose.getY();
    Rotation2d deltaRotation = targetPose.getRotation().minus(currentPose.getRotation());
    Translation2d driveTranslation = new Translation2d(translationX, translationY);

    drive.driveFieldAngle(driveTranslation, deltaRotation);
  }

  @Override
  public double getError() {
    return 0;
    // return targetAngle - drive.getGyroAngle();
  }

  @Override
  public void runWithOutput(double output) {
    // drive.driveWithInput(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(output / Math.abs(getError()))));
  }

  // @Override
  //   public boolean isFinished() {
  //       Rotation2d curRot = m_SwerveDrive.getPose2d().getRotation();
  //       double ballAngleDeg = m_lidar.getLatestBallAngleDegrees();

  //       // TODO: Tune
  //       return Math.abs(curRot.getDegrees() +ballAngleDeg) < 5;
  //   }
    
}
