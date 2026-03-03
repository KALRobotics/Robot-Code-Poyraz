// Copyright (c) FIRST and other WPILib contributors.
// Tek dosya: TimedRobot + tüm motorlar + kontroller. Command/Subsystem yok.

package frc.robot;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

// ==================== CAN ID'LERİ VE GÜÇ AYARLARI ====================

// Sürücü (Tank): Sol FL=1, BL=2 | Sağ FR=3, BR=4
// Shooter: Leader=6, Follower=5
// Taret: 7
// Intake: Pivot=9, Roller=10
// Hopper: 11 | Kicker: 12

// Güç / hız (duty cycle veya oran)
// Intake pivot: aşağı -0.10, yukarı 0.15
// Intake roller içeri: 0.3
// Shooter tam: 1.0
// Hopper / Kicker feed: 0.7 / 1.0
// Taret manuel: ±0.05
// Akıllı atış: tetik basıldığında shooter 1.0, 1 sn sonra hopper+kicker

// ==================== MOTOR TANIMLAMALARI ====================

public class Robot extends TimedRobot {

  // --- CAN ID'leri ---
  private static final int DRIVE_LEFT_LEADER_ID = 1;
  private static final int DRIVE_LEFT_FOLLOWER_ID = 2;
  private static final int DRIVE_RIGHT_LEADER_ID = 3;
  private static final int DRIVE_RIGHT_FOLLOWER_ID = 4;
  private static final int SHOOTER_LEADER_ID = 6;
  private static final int SHOOTER_FOLLOWER_ID = 5;
  private static final int TURRET_ID = 7;
  private static final int INTAKE_PIVOT_ID = 9;
  private static final int INTAKE_ROLLER_ID = 10;
  private static final int HOPPER_ID = 11;
  private static final int KICKER_ID = 12;

  // --- Güç / hız sabitleri ---
  private static final double DEADBAND = 0.1;
  private static final double INTAKE_PIVOT_DOWN = -0.10;
  private static final double INTAKE_PIVOT_UP = 0.15;
  private static final double INTAKE_ROLLER_SPEED = 0.3;
  private static final double SHOOTER_SPEED = 1.0;
  private static final double HOPPER_SPEED = 0.7;
  private static final double KICKER_SPEED = 1.0;
  private static final double TURRET_MANUAL_POWER = 0.05;
  private static final double SMART_SHOOT_DELAY_SEC = 1.0;

  // --- Intake pivot yazılım limitleri (derece) ---
  private static final double PIVOT_MIN_DEG = 0.0;
  private static final double PIVOT_MAX_DEG = 148.0;
  private static final int INTAKE_PIVOT_CURRENT_LIMIT_A = 20;

  // --- Kontrolcü portları ---
  private static final int DRIVER_PORT = 0;
  private static final int OPERATOR_PORT = 1;

  // --- Drive ---
  private SparkMax leftLeader;
  private SparkMax leftFollower;
  private SparkMax rightLeader;
  private SparkMax rightFollower;
  private DifferentialDrive differentialDrive;

  // --- Shooter ---
  private SparkMax shooterLeader;
  private SparkMax shooterFollower;

  // --- Taret ---
  private SparkMax turretMotor;

  // --- Intake ---
  private SparkMax intakePivotMotor;
  private SparkMax intakeRollerMotor;

  // --- Hopper & Kicker ---
  private SparkMax hopperMotor;
  private SparkMax kickerMotor;

  // --- Kontroller ---
  private XboxController driver;
  private XboxController operator;

  // --- Akıllı atış makrosu (Timer) ---
  private final Timer smartShootTimer = new Timer();

  // ==================== robotInit ====================

  @Override
  public void robotInit() {
    // --- Drive motorları ---
    leftLeader = new SparkMax(DRIVE_LEFT_LEADER_ID, MotorType.kBrushless);
    leftFollower = new SparkMax(DRIVE_LEFT_FOLLOWER_ID, MotorType.kBrushless);
    rightLeader = new SparkMax(DRIVE_RIGHT_LEADER_ID, MotorType.kBrushless);
    rightFollower = new SparkMax(DRIVE_RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    SparkMaxConfig leftLeaderCfg = new SparkMaxConfig();
    leftLeaderCfg.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    leftLeader.configure(leftLeaderCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SparkMaxConfig leftFollowerCfg = new SparkMaxConfig();
    leftFollowerCfg.idleMode(IdleMode.kBrake).smartCurrentLimit(40).follow(leftLeader);
    leftFollower.configure(leftFollowerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SparkMaxConfig rightCfg = new SparkMaxConfig();
    rightCfg.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    rightLeader.configure(rightCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(
        new SparkMaxConfig().idleMode(IdleMode.kBrake).smartCurrentLimit(40).follow(rightLeader),
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    differentialDrive = new DifferentialDrive(leftLeader, rightLeader);
    differentialDrive.setSafetyEnabled(false);

    // --- Shooter ---
    shooterLeader = new SparkMax(SHOOTER_LEADER_ID, MotorType.kBrushless);
    shooterFollower = new SparkMax(SHOOTER_FOLLOWER_ID, MotorType.kBrushless);
    SparkMaxConfig shooterCfg = new SparkMaxConfig();
    shooterCfg.smartCurrentLimit(40);
    shooterLeader.configure(shooterCfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    shooterFollower.configure(
        new SparkMaxConfig().follow(shooterLeader).smartCurrentLimit(40),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // --- Taret ---
    turretMotor = new SparkMax(TURRET_ID, MotorType.kBrushless);
    SparkMaxConfig turretCfg = new SparkMaxConfig();
    turretCfg.smartCurrentLimit(10);
    turretMotor.configure(turretCfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // --- Intake pivot (akım limiti 20A) ---
    intakePivotMotor = new SparkMax(INTAKE_PIVOT_ID, MotorType.kBrushless);
    SparkMaxConfig pivotCfg = new SparkMaxConfig();
    pivotCfg.smartCurrentLimit(INTAKE_PIVOT_CURRENT_LIMIT_A);
    intakePivotMotor.configure(pivotCfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // --- Intake roller ---
    intakeRollerMotor = new SparkMax(INTAKE_ROLLER_ID, MotorType.kBrushless);
    SparkMaxConfig rollerCfg = new SparkMaxConfig();
    rollerCfg.smartCurrentLimit(30);
    intakeRollerMotor.configure(rollerCfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // --- Hopper & Kicker ---
    hopperMotor = new SparkMax(HOPPER_ID, MotorType.kBrushless);
    kickerMotor = new SparkMax(KICKER_ID, MotorType.kBrushless);
    SparkMaxConfig feedCfg = new SparkMaxConfig();
    feedCfg.smartCurrentLimit(30);
    hopperMotor.configure(feedCfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    kickerMotor.configure(new SparkMaxConfig().smartCurrentLimit(30),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // --- Kontroller ---
    driver = new XboxController(DRIVER_PORT);
    operator = new XboxController(OPERATOR_PORT);
  }

  private static double applyDeadband(double value) {
    if (Math.abs(value) < DEADBAND) return 0.0;
    return value;
  }

  // Intake pivot: yazılım limitleri (derece). Basılıyken manuel güç, limit aşılıyorsa 0.
  private void setIntakePivotPower(double duty) {
    double angle = intakePivotMotor.getEncoder().getPosition();
    if (duty < 0 && angle >= PIVOT_MAX_DEG) duty = 0;
    if (duty > 0 && angle <= PIVOT_MIN_DEG) duty = 0;
    intakePivotMotor.set(duty);
  }

  // ==================== teleopPeriodic ====================

  @Override
  public void teleopPeriodic() {
    // ---------- SÜRÜCÜ (Port 0) ----------
    double leftY = applyDeadband(-driver.getLeftY());
    double rightY = applyDeadband(-driver.getRightY());
    differentialDrive.tankDrive(leftY, rightY);

    // D-Pad: Intake pivot (POV 180 = aşağı, POV 0 = yukarı)
    int pov = driver.getPOV();
    if (pov == 180) {
      setIntakePivotPower(INTAKE_PIVOT_DOWN);
    } else if (pov == 0) {
      setIntakePivotPower(INTAKE_PIVOT_UP);
    } else {
      intakePivotMotor.set(0);
    }

    // RB: Intake roller içeri
    if (driver.getRightBumper()) {
      intakeRollerMotor.set(INTAKE_ROLLER_SPEED);
    } else {
      intakeRollerMotor.set(0);
    }

    // Sağ Tetik (RT): Akıllı atış makrosu (Timer)
    if (driver.getRightTriggerAxis() > 0.5) {
      if (!smartShootTimer.isRunning()) {
        smartShootTimer.reset();
        smartShootTimer.start();
      }
      shooterLeader.set(SHOOTER_SPEED);
      if (smartShootTimer.get() > SMART_SHOOT_DELAY_SEC) {
        hopperMotor.set(HOPPER_SPEED);
        kickerMotor.set(KICKER_SPEED);
      } else {
        hopperMotor.set(0);
        kickerMotor.set(0);
      }
    } else {
      smartShootTimer.stop();
      smartShootTimer.reset();
      shooterLeader.set(0);
      hopperMotor.set(0);
      kickerMotor.set(0);
    }

    // ---------- OPERATÖR (Port 1) ----------
    // RT: Taret sağa
    if (operator.getRightTriggerAxis() > 0.5) {
      turretMotor.set(TURRET_MANUAL_POWER);
    } else if (operator.getLeftTriggerAxis() > 0.5) {
      turretMotor.set(-TURRET_MANUAL_POWER);
    } else {
      turretMotor.set(0);
    }

    // Back: Sıkışma çözücü (Hopper + Intake roller dışarı). Basılı değilse sürücü bloğu zaten değerleri ayarladı.
    if (operator.getBackButton()) {
      hopperMotor.set(-HOPPER_SPEED);
      intakeRollerMotor.set(-INTAKE_ROLLER_SPEED);
    }
  }

  @Override
  public void disabledInit() {
    differentialDrive.stopMotor();
    shooterLeader.set(0);
    turretMotor.set(0);
    intakePivotMotor.set(0);
    intakeRollerMotor.set(0);
    hopperMotor.set(0);
    kickerMotor.set(0);
    smartShootTimer.stop();
    smartShootTimer.reset();
  }
}
