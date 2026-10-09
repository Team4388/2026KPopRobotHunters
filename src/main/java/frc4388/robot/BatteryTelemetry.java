package frc4388.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Records the battery signals Marble needs from each robot run.
 *
 * <p>These entries are stored in the team's existing AdvantageKit WPILOG. They intentionally do
 * not identify a physical battery; that association belongs in Marble's pre-match "Use" action.
 */
public final class BatteryTelemetry {
  private final PowerDistribution powerDistribution = new PowerDistribution();

  /** Records one synchronized sample. Call once per robot packet. */
  public void periodic() {
    Logger.recordOutput("Battery/VoltageVolts", RobotController.getBatteryVoltage());
    Logger.recordOutput("Battery/TotalCurrentAmps", powerDistribution.getTotalCurrent());
    Logger.recordOutput("Battery/BrownedOut", RobotController.isBrownedOut());
    Logger.recordOutput("Battery/Enabled", DriverStation.isEnabled());
    Logger.recordOutput("Battery/MatchTimeSeconds", DriverStation.getMatchTime());
  }
}
