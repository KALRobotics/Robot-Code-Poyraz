package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.TankDriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;

public class OperatorControls {
  public static final boolean MACOS_WEIRD_CONTROLLER = true;

  public static void configure(int port, TankDriveSubsystem drivetrain, Superstructure superstructure) {
    CommandXboxController controller = new CommandXboxController(port);

    // if (Robot.isSimulation()) {
    // controller.leftBumper().whileTrue(aimCommand(drivetrain, superstructure));
    // controller.start().whileTrue(fireAlgae(drivetrain, superstructure));

    // Commands.run(() -> {
    // double leftX = controller.getLeftX();
    // double leftY = controller.getLeftY();
    // double rightY = controller.getRightY();

    // if (MACOS_WEIRD_CONTROLLER) {
    // rightY = controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();
    // }

    // // Apply deadband
    // if (Math.abs(leftX) < Constants.ControllerConstants.DEADBAND)
    // leftX = 0;
    // if (Math.abs(leftY) < Constants.ControllerConstants.DEADBAND)
    // leftY = 0;
    // if (Math.abs(rightY) < Constants.ControllerConstants.DEADBAND)
    // rightY = 0;

    // Translation3d translation = new Translation3d(leftX, leftY, rightY);

    // if (MACOS_WEIRD_CONTROLLER) {
    // // MacOS Xbox controller mapping is weird - swap X and Y
    // translation = new Translation3d(leftY, leftX, rightY);
    // }

    // // System.out.println("Adjusting pose by: " + translation.toString());

    // var newAimPoint = superstructure.getAimPoint().plus(translation.times(0.05));
    // // new Transform3d(leftX * 0.05, leftY * 0.05, rightY * 0.05));

    // superstructure.setAimPoint(newAimPoint);
    // }).ignoringDisable(true).schedule();
    // }

    // REAL CONTROLS
    controller.start().onTrue(superstructure.rezeroIntakePivotAndTurretCommand().ignoringDisable(true));
    
    controller.rightTrigger().whileTrue(superstructure.turret.addAngle(Degrees.of(0.1)));

    controller.leftTrigger().whileTrue(superstructure.turret.addAngle(Degrees.of(-0.1)));

    controller.leftBumper()
        .whileTrue(superstructure.setIntakeDeployAndRoll().withName("OperatorControls.intakeDeployed"));
    controller.leftBumper().onFalse(superstructure.retractIntakeUpCommand());

    controller.y().onTrue(superstructure.shootCommand());
    // controller.x().whileTrue(superstructure.stopShootingCommand()); // disabled: double binding; X = ManualTest.Kicker only

    controller.a().whileTrue(
        superstructure.feedAllCommand()
            .finallyDo(() -> superstructure.stopFeedingAllCommand().schedule()));

    // controller.b().whileTrue(
    //     superstructure.backFeedAllCommand()
    //         .finallyDo(() -> superstructure.stopFeedingAllCommand().schedule())); // disabled: double binding; B = ManualTest.Shooter only

    // controller.povUp().onTrue(superstructure.setTurretForward().withName("OperatorControls.setTurretForward")); // disabled: double binding; POV Up = ManualTest.IntakePivotUp only

    controller.povLeft().onTrue(superstructure.setTurretLeft().withName("OperatorControls.setTurretLeft"));
    controller.povRight().onTrue(superstructure.setTurretRight().withName("OperatorControls.setTurretRight"));

    // controller.rightBumper().toggleOnTrue(
    //     new ShootOnTheMoveCommand(drivetrain, superstructure, () -> superstructure.getAimPoint())
    //         .ignoringDisable(true)
    //         .withName("OperatorControls.aimCommand")); // disabled: double binding; RB = ManualTest.IntakeRollers only

    // ----- Intake pivot: D-Pad Up = 0° (stow), D-Pad Down = 148° (down); release = hold current -----
    controller.povUp().whileTrue(
        Commands.run(() -> superstructure.intake.setPivotTarget(Degrees.of(IntakeSubsystem.PIVOT_UP_DEGREES)), superstructure.intake)
            .finallyDo(superstructure.intake::holdPivot)
            .withName("OperatorControls.IntakePivotUp"));
    controller.povDown().whileTrue(
        Commands.run(() -> superstructure.intake.setPivotTarget(Degrees.of(IntakeSubsystem.PIVOT_DOWN_DEGREES)), superstructure.intake)
            .finallyDo(superstructure.intake::holdPivot)
            .withName("OperatorControls.IntakePivotDown"));
    controller.rightBumper().whileTrue(
        superstructure.intake.intakeCommand().withName("ManualTest.IntakeRollers"));
    // controller.x().whileTrue(
    //     superstructure.kicker.feedCommand().withName("ManualTest.Kicker")); // disabled: B = Kicker+Shooter combined
    // controller.b().whileTrue(
    //     Commands.sequence(
    //         superstructure.shooter.spinUp(),
    //         Commands.waitForever())
    //         .finallyDo(() -> superstructure.shooter.stop().schedule())
    //         .withName("ManualTest.Shooter")); // disabled: B = Kicker+Shooter combined
    controller.b().whileTrue(
        Commands.parallel(
            superstructure.hopper.feedCommand(),
            superstructure.kicker.feedCommand(),
            Commands.sequence(
                superstructure.shooter.spinUp(),
                Commands.waitForever())
                .finallyDo(() -> superstructure.shooter.stop().schedule()))
            .finallyDo(() -> {
                superstructure.hopper.stopCommand().schedule();
                superstructure.kicker.stopCommand().schedule();
                superstructure.shooter.stop().schedule();
            })
            .withName("ManualTest.HopperKickerAndShooter"));
  }
}
