package frc4388.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;

public final class FieldConstants {
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    
    public static final Translation2d BLUE_HUB_POS = new Translation2d();
    public static final Translation2d RED_HUB_POS = new Translation2d();


}
