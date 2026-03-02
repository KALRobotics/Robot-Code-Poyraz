package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Tank drive subsystem with DifferentialDrivePoseEstimator (vision-fused odometry)
 * and PathPlanner AutoBuilder integration via PPLTVController.
 *
 * Left side: motors 1 (FL drive), 2 (BL drive)
 * Right side: motors 3 (FR drive), 4 (BR drive)
 */
public class TankDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader = new SparkMax(Constants.DriveConstants.FL.kDriveMotorId, MotorType.kBrushless);
  private final SparkMax leftFollower = new SparkMax(Constants.DriveConstants.BL.kDriveMotorId, MotorType.kBrushless);
  private final SparkMax rightLeader = new SparkMax(Constants.DriveConstants.FR.kDriveMotorId, MotorType.kBrushless);
  private final SparkMax rightFollower = new SparkMax(Constants.DriveConstants.BR.kDriveMotorId, MotorType.kBrushless);

  private final DifferentialDrive differentialDrive;
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  private final DifferentialDrivePoseEstimator poseEstimator;
  private final DifferentialDriveKinematics kinematics;

  // PhotonVision
  private static final boolean IS_PHOTONVISION_ENABLED = true;
  private PhotonCamera photonCamera;
  private PhotonPoseEstimator photonPoseEstimator;
  private double distanceToHub = 0.0;

  private final SimpleMotorFeedforward feedforward;
  private final PIDController leftPID;
  private final PIDController rightPID;

  //MAP
  private final Field2d m_field = new Field2d();

  public TankDriveSubsystem() {

    SmartDashboard.putData("Field", m_field);

    // Encoder conversion: rotations → meters, RPM → m/s
    double wheelDiameterMeters = Units.inchesToMeters(Constants.TankDriveConstants.WHEEL_DIAMETER_INCHES);
    double gearRatio = Constants.TankDriveConstants.GEAR_RATIO;
    double encoderPositionFactor = (Math.PI * wheelDiameterMeters) / gearRatio;
    double encoderVelocityFactor = encoderPositionFactor / 60.0;

    // REVLib 2026: configure all SparkMax settings via SparkMaxConfig objects
    SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
    leftLeaderConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40);
    leftLeaderConfig.encoder
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor);

    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    leftFollowerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .follow(leftLeader);

    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    rightLeaderConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40);
    rightLeaderConfig.encoder
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor);

    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .follow(rightLeader);

    leftLeader.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    differentialDrive = new DifferentialDrive(leftLeader, rightLeader);
    differentialDrive.setSafetyEnabled(false);

    gyro.reset();

    leftLeader.getEncoder().setPosition(0);
    rightLeader.getEncoder().setPosition(0);

    kinematics = new DifferentialDriveKinematics(Constants.TankDriveConstants.TRACK_WIDTH_METERS);

    feedforward = new SimpleMotorFeedforward(
        Constants.TankDriveConstants.kS,
        Constants.TankDriveConstants.kV,
        Constants.TankDriveConstants.kA);

    leftPID = new PIDController(
        Constants.TankDriveConstants.kP,
        Constants.TankDriveConstants.kI,
        Constants.TankDriveConstants.kD);
    rightPID = new PIDController(
        Constants.TankDriveConstants.kP,
        Constants.TankDriveConstants.kI,
        Constants.TankDriveConstants.kD);

    poseEstimator = new DifferentialDrivePoseEstimator(
        kinematics,
        gyro.getRotation2d(),
        0.0,
        0.0,
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    // PhotonVision 2026: 2-arg constructor (fieldLayout, robotToCamera transform)
    if (IS_PHOTONVISION_ENABLED) {
      try {
        photonCamera = new PhotonCamera("pi kamera");
        var robotToCamera = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-11.0),
                Units.inchesToMeters(-9.0),
                Units.inchesToMeters(12.75)),
            new Rotation3d(0, Units.degreesToRadians(21), Units.degreesToRadians(180)));

        photonPoseEstimator = new PhotonPoseEstimator(
            Constants.fieldLayout,
            robotToCamera);
      } catch (Exception e) {
        System.err.println("Failed to initialize PhotonVision: " + e.getMessage());
        photonCamera = null;
        photonPoseEstimator = null;
      }
    }

    // PathPlanner AutoBuilder: LTV controller for differential drive
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      System.err.println("Failed to load PathPlanner RobotConfig: " + e.getMessage());
      e.printStackTrace();
      config = null;
    }

    if (config != null) {
      AutoBuilder.configure(
          this::getPose,
          this::resetPose,
          this::getRobotRelativeSpeeds,
          (speeds, feedforwards) -> driveRobotRelative(speeds),
          new PPLTVController(0.02),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
          },
          this);
    }
  }

  @Override
  public void periodic() {
    poseEstimator.update(
        gyro.getRotation2d(),
        leftLeader.getEncoder().getPosition(),
        rightLeader.getEncoder().getPosition());

    // PhotonVision 2026: get latest result, then pass to estimator
    if (IS_PHOTONVISION_ENABLED && photonCamera != null && photonPoseEstimator != null) {
      var latestResult = photonCamera.getLatestResult();
      Optional<EstimatedRobotPose> visionEstimate = photonPoseEstimator.update(latestResult);

      visionEstimate.ifPresent(estimate -> {
        if (estimate.targetsUsed.size() > 0) {
          poseEstimator.addVisionMeasurement(
              estimate.estimatedPose.toPose2d(),
              estimate.timestampSeconds);

          Logger.recordOutput("PhotonVision/TagCount", estimate.targetsUsed.size());
          Logger.recordOutput("FieldSimulation/PhotonPose", estimate.estimatedPose);

          Translation3d hubPos = Constants.AimPoints.RED_HUB.value;
          try {
            hubPos = Constants.AimPoints.getAllianceHubPosition();
          } catch (Exception ignored) {
          }
          distanceToHub = estimate.estimatedPose.toPose2d()
              .getTranslation()
              .getDistance(new Translation2d(hubPos.getX(), hubPos.getY()));
          Logger.recordOutput("FieldSimulation/hubDiff", distanceToHub);
        }
      });
    }

    Logger.recordOutput("TankDrive/Pose", getPose());
    Logger.recordOutput("TankDrive/LeftVelocity", leftLeader.getEncoder().getVelocity());
    Logger.recordOutput("TankDrive/RightVelocity", rightLeader.getEncoder().getVelocity());
    Logger.recordOutput("TankDrive/GyroAngle", gyro.getAngle());

    m_field.setRobotPose(poseEstimator.getEstimatedPosition());
  }

  @Override
  public void simulationPeriodic() {
  }

  // ---------- PathPlanner required interface ----------

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(
        gyro.getRotation2d(),
        leftLeader.getEncoder().getPosition(),
        rightLeader.getEncoder().getPosition(),
        pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    setWheelSpeeds(wheelSpeeds);
  }

  // ---------- Teleop drive commands ----------

  public Command arcadeDriveCommand(DoubleSupplier forward, DoubleSupplier rotation) {
    return run(() -> differentialDrive.arcadeDrive(forward.getAsDouble(), rotation.getAsDouble()))
        .finallyDo(() -> differentialDrive.stopMotor());
  }

  public Command curvatureDriveCommand(DoubleSupplier throttle, DoubleSupplier turn, BooleanSupplier quickTurn) {
    return run(() -> differentialDrive.curvatureDrive(throttle.getAsDouble(), turn.getAsDouble(), quickTurn.getAsBoolean()))
        .finallyDo(() -> differentialDrive.stopMotor());
  }

  public Command tankDriveCommand(DoubleSupplier left, DoubleSupplier right) {
    return run(() -> differentialDrive.tankDrive(left.getAsDouble(), right.getAsDouble()))
        .finallyDo(() -> differentialDrive.stopMotor());
  }

  public Command driveForward() {
    return run(() -> differentialDrive.arcadeDrive(0.5, 0))
        .finallyDo(() -> differentialDrive.stopMotor());
  }

  public Command driveBackwards() {
    return run(() -> differentialDrive.arcadeDrive(-0.5, 0))
        .finallyDo(() -> differentialDrive.stopMotor());
  }

  // ---------- Motor output ----------

  public void setWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
    double leftFF = feedforward.calculate(speeds.leftMetersPerSecond);
    double rightFF = feedforward.calculate(speeds.rightMetersPerSecond);

    double leftFeedback = leftPID.calculate(
        leftLeader.getEncoder().getVelocity(), speeds.leftMetersPerSecond);
    double rightFeedback = rightPID.calculate(
        rightLeader.getEncoder().getVelocity(), speeds.rightMetersPerSecond);

    leftLeader.setVoltage(leftFF + leftFeedback);
    rightLeader.setVoltage(rightFF + rightFeedback);
  }

  // ---------- Accessors ----------

  public Pose3d getPose3d() {
    return new Pose3d(getPose());
  }

  public void resetOdometry(Pose2d pose) {
    resetPose(pose);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftLeader.getEncoder().getVelocity(),
        rightLeader.getEncoder().getVelocity());
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void zeroGyro() {
    gyro.reset();
    resetPose(new Pose2d(getPose().getTranslation(), Rotation2d.kZero));
  }

  public double getDistanceToHub() {
    return IS_PHOTONVISION_ENABLED ? distanceToHub : 0.0;
  }

  public ChassisSpeeds getRobotVelocity() {
    return getRobotRelativeSpeeds();
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getHeading());
  }

  public Rotation2d getPitch() {
    return Rotation2d.kZero;
  }

  public void setMotorBrake(boolean brake) {
    IdleMode mode = brake ? IdleMode.kBrake : IdleMode.kCoast;
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.idleMode(mode);
    leftLeader.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    leftFollower.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightLeader.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightFollower.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
