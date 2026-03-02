package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public final class Constants {

  public static enum AimPoints {
    RED_HUB(new Translation3d(11.938, 4.034536, 1.5748)),
    RED_OUTPOST(new Translation3d(15.75, 7.25, 0)),
    RED_FAR_SIDE(new Translation3d(15.75, 0.75, 0)),

    BLUE_HUB(new Translation3d(4.5974, 4.034536, 1.5748)),
    BLUE_OUTPOST(new Translation3d(0.75, 0.75, 0)),
    BLUE_FAR_SIDE(new Translation3d(0.75, 7.25, 0));

    public final Translation3d value;

    private AimPoints(Translation3d value) {
      this.value = value;
    }

    public static final Translation3d getAllianceHubPosition() {
      return DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red
          ? RED_HUB.value : BLUE_HUB.value;
    }

    public static final Translation3d getAllianceOutpostPosition() {
      return DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red
          ? RED_OUTPOST.value : BLUE_OUTPOST.value;
    }

    public static final Translation3d getAllianceFarSidePosition() {
      return DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red
          ? RED_FAR_SIDE.value : BLUE_FAR_SIDE.value;
    }
  }

  public static final double ROBOT_MASS = Units.lbsToKilograms(120);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms spark max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);

  // AprilTag field layout for PhotonVision (2026 Rebuilt field)
  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // CHANGED: Tank drive physical + control constants (replaces swerve config)
  public static class TankDriveConstants {
    public static final double TRACK_WIDTH_METERS = 0.546;
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double GEAR_RATIO = 6.75;

    // Feedforward gains — ESTIMATED VALUES for NEO + 6.75:1, run SysId to refine
    public static final double kS = 0.15;  // volts (static friction)
    public static final double kV = 2.6;   // volts * seconds / meter (12V / ~4.6 m/s max)
    public static final double kA = 0.3;   // volts * seconds^2 / meter

    // Velocity PID per wheel side — conservative starting values, tune after SysId
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Max auto speeds (PathPlanner also reads from its settings.json)
    public static final double MAX_AUTO_VELOCITY = 3.0;       // m/s
    public static final double MAX_AUTO_ACCELERATION = 3.0;   // m/s^2
  }

  public static class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kPoseControllerPort = 2;

    // Joystick Deadband
    public static final double DEADBAND = 0.1;

    /** Global speed limit for hardware test (e.g. 0.2 = 20% max). Set to 1.0 for full speed. */
    public static final double TEST_SPEED_MULTIPLIER = 0.2;
  }

  public static class DriveConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    public static class FL {
      public static final int kDriveMotorId = 1;
    }

    public static class BL {
      public static final int kDriveMotorId = 2;
    }

    public static class FR {
      public static final int kDriveMotorId = 3;
    }

    public static class BR {
      public static final int kDriveMotorId = 4;
    }
  }

  public static class ShooterConstants {
    public static final int kLeaderMotorId = 6;
    public static final int kFollowerMotorId = 5;
  }

  public static class TurretConstants {
    public static final int kMotorId = 7;
  }

  public static class IntakeConstants {
    public static final int kPivotMotorId = 9;
    public static final int kRollerMotorId = 10;
  }

  public static class HopperConstants {
    public static final int kHopperMotorId = 11;
  }

  public static class KickerConstants {
    public static final int kKickerMotorId = 12;
  }
}
