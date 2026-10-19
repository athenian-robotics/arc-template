package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

/** Alliance helper that latches the last known alliance to avoid "unknown -> Blue" flicker. */
public final class AllianceUtil {
  private static volatile Alliance lastKnownAlliance = Alliance.Blue;
  private static volatile boolean everHadAlliance = false;

  private AllianceUtil() {}

  /**
   * Returns the current alliance if reported by the DS/FMS, otherwise returns the last known
   * alliance (or {@code defaultAlliance} if the alliance has never been reported yet).
   */
  public static Alliance getAllianceOrDefault(Alliance defaultAlliance) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      lastKnownAlliance = alliance.get();
      everHadAlliance = true;
      return lastKnownAlliance;
    }
    return everHadAlliance ? lastKnownAlliance : defaultAlliance;
  }

  /** Returns the latched alliance, defaulting to Blue until the DS reports one. */
  public static Alliance getAlliance() {
    return getAllianceOrDefault(Alliance.Blue);
  }

  /** True when the (latched) alliance is Red. */
  public static boolean isRedAlliance() {
    return getAlliance() == Alliance.Red;
  }
}

