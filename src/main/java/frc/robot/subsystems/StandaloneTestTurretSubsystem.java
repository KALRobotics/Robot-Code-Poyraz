package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Self-contained test subsystem that bundles turret, shooter, hopper, and kicker
 * with direct open-loop control from a single Xbox controller.
 * Only requires WPILib + REVLib vendordep. No other dependencies.
 *
 * Controls:
 *   Left Bumper (hold)  — turret turns left
 *   Right Bumper (hold) — turret turns right
 *   Y (hold)            — spin up shooter
 *   Left Trigger (hold) — feed (hopper + kicker forward)
 *
 * Usage in RobotContainer:
 *   StandaloneTestTurretSubsystem test = new StandaloneTestTurretSubsystem();
 *   test.setDefaultCommand(test.manualDriveCommand(0));
 */
public class StandaloneTestTurretSubsystem extends SubsystemBase {

  // --- CAN IDs (change these to match your robot's wiring) ---
  private static final int TURRET_CAN_ID = 7;
  private static final int SHOOTER_LEADER_CAN_ID = 5;
  private static final int SHOOTER_FOLLOWER_CAN_ID = 6;
  private static final int HOPPER_CAN_ID = 11;
  private static final int KICKER_CAN_ID = 12;

  // --- Motors ---
  private final SparkMax turretMotor;
  private final SparkMax shooterLeader;
  private final SparkMax shooterFollower;
  private final SparkMax hopperMotor;
  private final SparkMax kickerMotor;

  // --- Speeds (conservative for testing — increase after verifying) ---
  private static final double TURRET_SPEED = 0.10;
  private static final double SHOOTER_SPEED = 0.40;
  private static final double FEED_SPEED = 0.50;
  private static final double TRIGGER_THRESHOLD = 0.3;

  public StandaloneTestTurretSubsystem() {
    turretMotor = new SparkMax(TURRET_CAN_ID, MotorType.kBrushless);
    shooterLeader = new SparkMax(SHOOTER_LEADER_CAN_ID, MotorType.kBrushless);
    shooterFollower = new SparkMax(SHOOTER_FOLLOWER_CAN_ID, MotorType.kBrushless);
    hopperMotor = new SparkMax(HOPPER_CAN_ID, MotorType.kBrushless);
    kickerMotor = new SparkMax(KICKER_CAN_ID, MotorType.kBrushless);

    // Turret: 40:1 reduction, inverted, low current limit
    SparkMaxConfig turretCfg = new SparkMaxConfig();
    turretCfg
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(10);
    turretMotor.configure(turretCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Shooter leader: coast so flywheel spins down naturally
    SparkMaxConfig shooterLeaderCfg = new SparkMaxConfig();
    shooterLeaderCfg
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40);
    shooterLeader.configure(shooterLeaderCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Shooter follower: follows leader, inverted (counter-rotating flywheel pair)
    SparkMaxConfig shooterFollowerCfg = new SparkMaxConfig();
    shooterFollowerCfg
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40)
        .follow(shooterLeader, true);
    shooterFollower.configure(shooterFollowerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Hopper: brake to hold game pieces in place
    SparkMaxConfig hopperCfg = new SparkMaxConfig();
    hopperCfg
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30);
    hopperMotor.configure(hopperCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Kicker: brake to hold game pieces
    SparkMaxConfig kickerCfg = new SparkMaxConfig();
    kickerCfg
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20);
    kickerMotor.configure(kickerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Returns a command that polls the given controller every cycle and
   * drives all motors directly. Set this as the default command.
   *
   * @param controllerPort the Xbox controller port (usually 0 or 1)
   */
  public Command manualDriveCommand(int controllerPort) {
    XboxController ctrl = new XboxController(controllerPort);

    return run(() -> {
      double mult = Constants.ControllerConstants.TEST_SPEED_MULTIPLIER;
      // --- Turret: bumpers ---
      if (ctrl.getRightBumper()) {
        turretMotor.set(TURRET_SPEED * mult);
      } else if (ctrl.getLeftBumper()) {
        turretMotor.set(-TURRET_SPEED * mult);
      } else {
        turretMotor.set(0);
      }

      // --- Shooter: Y button ---
      if (ctrl.getYButton()) {
        shooterLeader.set(SHOOTER_SPEED * mult);
      } else {
        shooterLeader.set(0);
      }

      // --- Feed (hopper + kicker): left trigger ---
      if (ctrl.getLeftTriggerAxis() > TRIGGER_THRESHOLD) {
        hopperMotor.set(FEED_SPEED * mult);
        kickerMotor.set(FEED_SPEED * mult);
      } else {
        hopperMotor.set(0);
        kickerMotor.set(0);
      }
    }).finallyDo(() -> {
      turretMotor.set(0);
      shooterLeader.set(0);
      hopperMotor.set(0);
      kickerMotor.set(0);
    }).withName("StandaloneTest.ManualDrive");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Test/TurretOutput", turretMotor.getAppliedOutput());
    SmartDashboard.putNumber("Test/ShooterRPM", shooterLeader.getEncoder().getVelocity());
    SmartDashboard.putNumber("Test/HopperOutput", hopperMotor.getAppliedOutput());
    SmartDashboard.putNumber("Test/KickerOutput", kickerMotor.getAppliedOutput());
  }
}
