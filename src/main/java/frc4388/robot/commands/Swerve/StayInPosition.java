// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc4388.robot.commands.PID;
import frc4388.robot.subsystems.swerve.SwerveDrive;

public class StayInPosition extends PID {
    SwerveDrive drive;

    public StayInPosition(SwerveDrive drive) {
        super(0.3, 0.0, 0.0, 0.0, 1);
        this.drive = drive;
        addRequirements(drive);
    }

    public void goToTargetPose(Pose2d targetPose) {
        Pose2d currentPose = drive.getCurrentPose();

        double translationX = targetPose.getX() - currentPose.getX();
        double translationY = targetPose.getY() - currentPose.getY();

        double magnitude = Math.sqrt(translationX * translationX + translationY * translationY);
        if (magnitude > 1.0) {
            translationX /= magnitude;
            translationY /= magnitude;
        }

        Translation2d driveTranslation;
        if (magnitude < 0.05) {
            driveTranslation = new Translation2d();
        } else {
            driveTranslation = new Translation2d(translationX, translationY);
        }

        drive.driveFieldAngleSIP(driveTranslation, targetPose.getRotation());
    }

    @Override
    public double getError() {
        return 0;
    }

    @Override
    public void runWithOutput(double output) {}
}