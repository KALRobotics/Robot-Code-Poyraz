package frc.robot.controls;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.TankDriveSubsystem;
import frc.robot.util.maplesim.RebuiltFuelOnFly;

public class DriverControls {

  private static Pose2d getTargetPose() {
    Pose2d hubPose = new Pose2d(
        Meter.of(11.902),
        Meter.of(4.031),
        Rotation2d.kZero);

    Logger.recordOutput("DriverControls/TargetHubPose", hubPose);

    return hubPose;
  }

  public static void configure(int port, TankDriveSubsystem drivetrain, Superstructure superstructure) {
    CommandXboxController controller = new CommandXboxController(port);
    CommandXboxController controller2 = new CommandXboxController(1);

    // Curvature drive: left stick Y = throttle, right stick X = turn rate
    // Quick-turn (turn in place) activates when throttle is near zero
    drivetrain.setDefaultCommand(
        drivetrain.curvatureDriveCommand(
            () -> {
              double input = -controller.getLeftY() + controller2.getLeftY();
              if (Math.abs(input) < ControllerConstants.DEADBAND) {
                return 0.0;
              }
              return input * 0.5 * Constants.ControllerConstants.TEST_SPEED_MULTIPLIER;
            },
            () -> {
              double input = -controller.getRightX() + controller2.getRightX();
              if (Math.abs(input) < ControllerConstants.DEADBAND) {
                return 0.0;
              }
              return input * 0.5 * Constants.ControllerConstants.TEST_SPEED_MULTIPLIER;
            },
            () -> Math.abs(controller.getLeftY()) < ControllerConstants.DEADBAND && controller2.getLeftY() < ControllerConstants.DEADBAND)
            .withName("Drive"));

    if (DriverStation.isTest()) {
      // Test mode controls
      controller.y().onTrue(Commands.runOnce(drivetrain::zeroGyro));
      // Add other test controls as needed
    } else if (Robot.isSimulation()) {
      // Fire fuel 10 times per second while button is held
      controller.back().whileTrue(
          Commands.repeatingSequence(
              fireFuel(drivetrain, superstructure),
              Commands.waitSeconds(0.1)));
    } else {
      // Teleop controls
      controller.start().onTrue(Commands.runOnce(drivetrain::zeroGyro));
      controller.rightBumper().whileTrue(
          superstructure.feedAllCommand()
              .finallyDo(() -> superstructure.stopFeedingAllCommand().schedule()));

      controller.leftBumper()
          .whileTrue(superstructure.setIntakeDeployAndRoll().withName("OperatorControls.intakeDeployed"));
      controller.leftBumper().onFalse(superstructure.retractIntakeUpCommand());

      // Smart Shooter Macro: spin up to match RPM, wait until at speed, then feed (avoids jams)
      controller.rightTrigger().whileTrue(
          Commands.sequence(
              superstructure.shooter.spinUpMatchSpeed(),
              new WaitUntilCommand(() -> superstructure.shooter.isAtTargetSpeed()),
              Commands.parallel(
                  superstructure.hopper.feedCommand(),
                  superstructure.kicker.feedCommand()))
              .finallyDo(() -> {
                superstructure.shooter.stop().schedule();
                superstructure.hopper.stopCommand().schedule();
                superstructure.kicker.stopCommand().schedule();
              })
              .withName("DriverControls.SmartShooterMacro"));


      controller.y().onTrue(superstructure.shootCommand());
      
      controller.x().whileTrue(superstructure.stopShootingCommand());

      controller.b().whileTrue(
          superstructure.backFeedAllCommand()
              .finallyDo(() -> superstructure.stopFeedingAllCommand().schedule()));

      // Intake pivot: software-limited voltage control (no PID oscillation); release = stop motor
      controller.povUp().whileTrue(
          Commands.run(() -> superstructure.intake.setPivotDutyCycleWithLimits(IntakeSubsystem.PIVOT_UP_DUTY), superstructure.intake)
              .finallyDo(() -> superstructure.intake.setPivotDutyCycleWithLimits(0))
              .withName("DriverControls.IntakePivotUp"));
      controller.povDown().whileTrue(
          Commands.run(() -> superstructure.intake.setPivotDutyCycleWithLimits(IntakeSubsystem.PIVOT_DOWN_DUTY), superstructure.intake)
              .finallyDo(() -> superstructure.intake.setPivotDutyCycleWithLimits(0))
              .withName("DriverControls.IntakePivotDown"));
    }
  }

  public static Command fireFuel(TankDriveSubsystem drivetrain, Superstructure superstructure) {
    return Commands.runOnce(() -> {
      SimulatedArena arena = SimulatedArena.getInstance();

      GamePieceProjectile fuel = new RebuiltFuelOnFly(
          drivetrain.getPose().getTranslation(),
          new Translation2d(
              superstructure.turret.turretTranslation.getX() * -1,
              superstructure.turret.turretTranslation.getY()),
          drivetrain.getRobotVelocity(),
          drivetrain.getPose().getRotation().rotateBy(superstructure.getAimRotation3d().toRotation2d()),
          superstructure.turret.turretTranslation.getMeasureZ(),

          superstructure.getTangentialVelocity().times(0.5),
          Degrees.of(60));

      // Configure callbacks to visualize the flight trajectory of the projectile
      fuel.withProjectileTrajectoryDisplayCallBack(
          // Callback for when the note will eventually hit the target (if configured)
          (pose3ds) -> Logger.recordOutput("FieldSimulation/Shooter/ProjectileSuccessfulShot",
              pose3ds.toArray(Pose3d[]::new)),
          // Callback for when the note will eventually miss the target, or if no target
          // is configured
          (pose3ds) -> Logger.recordOutput("FieldSimulation/Shooter/ProjectileUnsuccessfulShot",
              pose3ds.toArray(Pose3d[]::new)));

      arena.addGamePieceProjectile(fuel);
    });
  }
}
