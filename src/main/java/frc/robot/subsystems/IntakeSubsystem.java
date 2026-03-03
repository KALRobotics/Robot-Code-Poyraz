package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeSubsystem extends SubsystemBase {

  // Reduced from 1.0 for safe initial testing with NEO — scale up after verifying
  private static final double INTAKE_SPEED = 0.6;

  /** Pivot runs down for this long when button held; physical stop limits travel at bottom. */
  private static final double DEPLOY_DOWN_SECONDS = 5.0;
  /** Pivot runs up (to 0°) for this long when button released; physical stop at top. */
  private static final double RETRACT_UP_SECONDS = 5.0;

  /** Intake pivot: target angles (degrees) and software limits. */
  public static final double PIVOT_UP_DEGREES = 0.0;
  public static final double PIVOT_DOWN_DEGREES = 148.0;
  /** Gear ratio: 5, 5, 60/18 → output degrees per motor rotation = 360 / ratio. */
  private static final double PIVOT_GEAR_RATIO = 5.0 * 5.0 * (60.0 / 18.0);
  /** Position PID kP for lock (dynamic setpoint); soft to avoid oscillation. */
  private static final double PIVOT_LOCK_kP = 0.05;
  /** Smart current limit (A) when pressing on ball. */
  private static final int PIVOT_SMART_CURRENT_LIMIT = 25;
  /** Manual drive duty: down. */
  public static final double PIVOT_DOWN_DUTY = -0.20;
  /** Manual drive duty: up. */
  public static final double PIVOT_UP_DUTY = 0.30;

  private SparkMax rollerSpark = new SparkMax(Constants.IntakeConstants.kRollerMotorId, MotorType.kBrushless);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(30));

  private SmartMotorController smc = new SparkWrapper(rollerSpark, DCMotor.getNEO(1), smcConfig);

  private final FlyWheelConfig intakeConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(0.5))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(-6000))
      .withTelemetry("IntakeRoller", TelemetryVerbosity.HIGH);

  private FlyWheel intake = new FlyWheel(intakeConfig);

  // 5:1, 5:1, 60/18 — Position PID for lock (PIVOT_LOCK_kP), current limit 25 A
  private SmartMotorControllerConfig intakePivotSmartMotorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(PIVOT_LOCK_kP, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(360))
      .withFeedforward(new SimpleMotorFeedforward(0, 10, 0))
      .withTelemetry("IntakePivotMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5, 60.0 / 18.0)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withSoftLimit(Degrees.of(PIVOT_UP_DEGREES), Degrees.of(PIVOT_DOWN_DEGREES))
      .withStatorCurrentLimit(Amps.of(PIVOT_SMART_CURRENT_LIMIT))
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1));

  private SparkMax pivotMotor = new SparkMax(Constants.IntakeConstants.kPivotMotorId, MotorType.kBrushless);

  private SmartMotorController intakePivotController = new SparkWrapper(pivotMotor, DCMotor.getNEO(1),
      intakePivotSmartMotorConfig);

  private final ArmConfig intakePivotConfig = new ArmConfig(intakePivotController)
      .withSoftLimits(Degrees.of(PIVOT_UP_DEGREES), Degrees.of(PIVOT_DOWN_DEGREES))
      .withHardLimit(Degrees.of(0), Degrees.of(155))
      .withStartingPosition(Degrees.of(0))
      .withLength(Feet.of(1))
      .withMass(Pounds.of(2))
      .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH);

  private Arm intakePivot = new Arm(intakePivotConfig);

  public IntakeSubsystem() {
    // Spark Max built-in: encoder in degrees, smart current limit, software limits (0–148°)
    double degreesPerMotorRotation = 360.0 / PIVOT_GEAR_RATIO;
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig
        .smartCurrentLimit(PIVOT_SMART_CURRENT_LIMIT)
        .softLimit
        .forwardSoftLimit(PIVOT_DOWN_DEGREES)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(PIVOT_UP_DEGREES)
        .reverseSoftLimitEnabled(true);
    pivotConfig.encoder
        .positionConversionFactor(degreesPerMotorRotation)
        .velocityConversionFactor(degreesPerMotorRotation / 60.0);
    pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Command to run the intake while held.
   */
  public Command intakeCommand() {
    return intake.set(INTAKE_SPEED * Constants.ControllerConstants.TEST_SPEED_MULTIPLIER).finallyDo(() -> smc.setDutyCycle(0)).withName("Intake.Run");
  }

  /**
   * Command to eject while held.
   */
  public Command ejectCommand() {
    return intake.set(-INTAKE_SPEED * Constants.ControllerConstants.TEST_SPEED_MULTIPLIER).finallyDo(() -> smc.setDutyCycle(0)).withName("Intake.Eject");
  }

  public Command setPivotAngle(Angle angle) {
    return intakePivot.setAngle(angle).withName("IntakePivot.SetAngle");
  }

  public Command rezero() {
    return Commands.runOnce(() -> pivotMotor.getEncoder().setPosition(0), this).withName("IntakePivot.Rezero");
  }

  /**
   * While held: pivot goes down for 5s (then holds at physical stop), roller runs.
   * When you release, binding runs retractUpCommand() so pivot always goes up for 5s.
   */
  public Command deployAndRollCommand() {
    final double[] startTime = { Double.NaN };
    return Commands.run(() -> {
      if (Double.isNaN(startTime[0])) {
        startTime[0] = Timer.getFPGATimestamp();
      }
      double elapsed = Timer.getFPGATimestamp() - startTime[0];
      if (elapsed < DEPLOY_DOWN_SECONDS) {
        setIntakeDeployed();
      } else {
        holdCurrentPivotPosition();
      }
      smc.setDutyCycle(INTAKE_SPEED * Constants.ControllerConstants.TEST_SPEED_MULTIPLIER);
    }, this).finallyDo(() -> smc.setDutyCycle(0)).withName("Intake.DeployAndRoll");
  }

  /**
   * Pivot goes up (to 0°) for 5s then holds. Schedule this when bumper is released.
   */
  public Command retractUpCommand() {
    return Commands.run(this::setIntakeStow, this)
        .withTimeout(RETRACT_UP_SECONDS)
        .finallyDo(this::holdCurrentPivotPosition)
        .withName("Intake.RetractUp");
  }

  public Command backFeedAndRollCommand() {
    final double[] startTime = { Double.NaN };
    return Commands.run(() -> {
      if (Double.isNaN(startTime[0])) {
        startTime[0] = Timer.getFPGATimestamp();
      }
      double elapsed = Timer.getFPGATimestamp() - startTime[0];
      if (elapsed < DEPLOY_DOWN_SECONDS) {
        setIntakeDeployed();
      } else {
        holdCurrentPivotPosition();
      }
      smc.setDutyCycle(-INTAKE_SPEED * Constants.ControllerConstants.TEST_SPEED_MULTIPLIER);
    }, this).finallyDo(() -> smc.setDutyCycle(0)).withName("Intake.BackFeedAndRoll");
  }

  private void setIntakeStow() {
    intakePivotController.setPosition(Degrees.of(0));
  }

  private void setIntakeFeed() {
    intakePivotController.setPosition(Degrees.of(59));
  }

  /**
   * Hold pivot at its current angle (use when releasing deploy so the arm doesn't
   * try to move toward 115° which may be further down if it never reached 148°).
   */
  private void holdCurrentPivotPosition() {
    intakePivotController.setPosition(intakePivot.getAngle());
  }

  /**
   * Set pivot target angle (clamped to software limits [0°, 148°]). Uses position PID.
   */
  public void setPivotTarget(Angle angle) {
    double deg = angle.in(Degrees);
    double clamped = Math.max(PIVOT_UP_DEGREES, Math.min(PIVOT_DOWN_DEGREES, deg));
    intakePivotController.setPosition(Degrees.of(clamped));
  }

  /**
   * Hold pivot at current angle (call when releasing manual test button).
   */
  public void holdPivot() {
    holdCurrentPivotPosition();
  }

  /**
   * Current pivot angle in degrees (from encoder). Used for software limits in voltage control.
   */
  public double getPivotAngleDegrees() {
    return pivotMotor.getEncoder().getPosition();
  }

  /**
   * Manual open-loop drive: motor runs at duty. Respects software limits (0° and 148°).
   * Use lockPosition() when releasing D-Pad to hold at current angle.
   */
  public void manualDrive(double duty) {
    double a = getPivotAngleDegrees();
    if (duty < 0 && a >= PIVOT_DOWN_DEGREES) duty = 0;
    if (duty > 0 && a <= PIVOT_UP_DEGREES) duty = 0;
    pivotMotor.set(duty);
  }

  /**
   * Set current angle as position PID target (dynamic setpoint). Call when releasing D-Pad.
   */
  public void lockPosition() {
    double currentAngle = getPivotAngleDegrees();
    double clamped = Math.max(PIVOT_UP_DEGREES, Math.min(PIVOT_DOWN_DEGREES, currentAngle));
    intakePivotController.setPosition(Degrees.of(clamped));
  }

  /** @deprecated Use manualDrive(duty) and lockPosition(). */
  public void setPivotDutyCycleWithLimits(double duty) {
    manualDrive(duty);
  }

  /**
   * Retract pivot to hold angle (115°). Use when you want a fixed "hold" position;
   * deploy release now holds current position instead so the arm doesn't drive down.
   */
  public Command retractToHoldCommand() {
    return Commands.runOnce(this::setIntakeHold, this).withName("Intake.RetractToHold");
  }

  private void setIntakeHold() {
    intakePivotController.setPosition(Degrees.of(115));
  }

  private void setIntakeDeployed() {
    intakePivotController.setPosition(Degrees.of(PIVOT_DOWN_DEGREES));
  }

  @Override
  public void periodic() {
    intake.updateTelemetry();
    intakePivot.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    intake.simIterate();
    intakePivot.simIterate();
  }
}
