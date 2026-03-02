package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax leaderSpark = new SparkMax(Constants.ShooterConstants.kLeaderMotorId,
      MotorType.kBrushless);

  private final SparkMax followerSpark = new SparkMax(Constants.ShooterConstants.kFollowerMotorId,
      MotorType.kBrushless);

  private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withFollowers(Pair.of(followerSpark, true))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.00936, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.191, 0.11858, 0.0))
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  private final SmartMotorController smc = new SparkWrapper(leaderSpark, DCMotor.getNEO(2), smcConfig);

  private final FlyWheelConfig shooterConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(1))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(0))
      .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

  private final FlyWheel shooter = new FlyWheel(shooterConfig);

  /** Target RPM for match (full power); used by isAtTargetSpeed() and spinUpMatchSpeed(). */
  private static final double MATCH_TARGET_RPM = 5500;
  /** Tolerance (RPM) for considering shooter at target speed. */
  private static final double TOLERANCE_RPM = 50;

  public ShooterSubsystem() {
  }

  /**
   * Returns true when current shooter RPM is within +/- TOLERANCE_RPM of match target (5500 RPM).
   */
  public boolean isAtTargetSpeed() {
    double current = getSpeed().in(RPM);
    return Math.abs(current - MATCH_TARGET_RPM) <= TOLERANCE_RPM;
  }

  /**
   * Starts shooter at full match speed (5500 RPM). Returns a command that schedules the setpoint
   * and ends immediately (for use in sequences before WaitUntilCommand(isAtTargetSpeed)).
   */
  public Command spinUpMatchSpeed() {
    return Commands.runOnce(() -> setSpeed(RPM.of(MATCH_TARGET_RPM)).schedule(), this);
  }

  public Command setSpeed(AngularVelocity speed) {
    return shooter.setSpeed(speed);
  }

  public Command setSpeedDynamic(Supplier<AngularVelocity> speedSupplier) {
    return shooter.setSpeed(speedSupplier);
  }

  public Command spinUp() {
    return setSpeed(RPM.of(5500 * Constants.ControllerConstants.TEST_SPEED_MULTIPLIER));

    // return setSpeed(RotationsPerSecond.of(50));

    // return run(() -> {
    // // followerNova.follow(leaderNova.getID());
    // // followerNova.setInverted(true);

    // // leaderNova.setPercent(SHOOTER_SPEED);
    // // followerNova.setPercent(SHOOTER_SPEED);

    // // followerNova.setPercent(0.5);
    // });

    // return shooter.set(0.5);
    // return shooter.setSpeed(RotationsPerSecond.of(500));
  }

  public Command stop() {
    return setSpeed(RPM.of(0));
    // return run(() -> {

    // // leaderNova.setPercent(0);
    // // followerNova.setPercent(0);
    // // followerNova.setPercent(0.5);
    // });
    // return shooter.set(0);
  }

  public AngularVelocity getSpeed() {
    return shooter.getSpeed();
  }

  // public Command set(double dutyCycle) {
  // return shooter.set(dutyCycle);
  // }

  public Command sysId() {
    return shooter.sysId(Volts.of(12), Volts.of(3).per(Second), Seconds.of(7));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/LeaderVelocity", leaderSpark.getEncoder().getVelocity());
    Logger.recordOutput("Shooter/FollowerVelocity", followerSpark.getEncoder().getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    shooter.simIterate();
  }

  private Distance wheelRadius() {
    return Inches.of(4).div(2);
  }

  public LinearVelocity getTangentialVelocity() {
    // Calculate tangential velocity at the edge of the wheel and convert to
    // LinearVelocity

    return MetersPerSecond.of(getSpeed().in(RadiansPerSecond)
        * wheelRadius().in(Meters));
  }
}
