package frc4388.robot.constants;

import java.util.Arrays;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public final class FieldConstants {
    // public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    
    public static final Translation2d BLUE_HUB_POS = new Translation2d();
    public static final Translation2d RED_HUB_POS = new Translation2d();

    // Test april tag field layout
        public static final AprilTagFieldLayout kTagLayout = new AprilTagFieldLayout(
            Arrays.asList(new AprilTag[] {
                new AprilTag(0, new Pose3d(
                    new Translation3d(0.,0.,0.26035), new Rotation3d(0.,0.,0.)
                )),
            }), 100, 100);

}
