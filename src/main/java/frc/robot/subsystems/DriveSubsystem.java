// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.constants.DriveConstants;
import frc.robot.utility.ctre.CTREConvert;
import frc.robot.wrappers.motorcontrollers.CCompanyTalonSRX;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final CCompanyTalonSRX m_leftMotors = new CCompanyTalonSRX(DriveConstants.kLeftFrontMotorPort,
      DriveConstants.kLeftInvertMotor, DriveConstants.kLeftInvertSensorPhase);
  private final CCompanyTalonSRX m_leftFollowMotor = new CCompanyTalonSRX(DriveConstants.kLeftRearMotorPort,
      DriveConstants.kLeftInvertMotor, DriveConstants.kLeftInvertSensorPhase);

  // The motors on the right side of the drive.
  private final CCompanyTalonSRX m_rightMotors = new CCompanyTalonSRX(DriveConstants.kRightFrontMotorPort,
      DriveConstants.kRightInvertMotor, DriveConstants.kRightInvertSensorPhase);
  private final CCompanyTalonSRX m_rightFollowMotor = new CCompanyTalonSRX(DriveConstants.kRightRearMotorPort,
      DriveConstants.kRightInvertMotor, DriveConstants.kRightInvertSensorPhase);

  // Object for simulated inputs into Talon.
  TalonSRXSimCollection m_leftDriveSim = m_leftMotors.getSimCollection();
  TalonSRXSimCollection m_rightDriveSim = m_rightMotors.getSimCollection();

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP); // NavX

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // These classes help us simulate our drivetrain
  public DifferentialDrivetrainSim m_drivetrainSimulator;
  // The Field2d class shows the field in the sim GUI
  private Field2d m_fieldSim;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_leftFollowMotor.follow(m_leftMotors);
    m_leftFollowMotor.setInverted(InvertType.FollowMaster);
    m_rightFollowMotor.follow(m_rightMotors);
    m_rightFollowMotor.setInverted(InvertType.FollowMaster);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    if (RobotBase.isSimulation()) { // If our robot is simulated
      m_leftMotors.setInverted(InvertType.None);
      m_leftMotors.setSensorPhase(false);
      m_rightMotors.setInverted(InvertType.None);
      m_rightMotors.setSensorPhase(false);

      // This class simulates our drivetrain's motion around the field.
      m_drivetrainSimulator = new DifferentialDrivetrainSim(DriveConstants.kDrivetrainPlant,
          DriveConstants.kDriveGearbox, DriveConstants.kDriveGearing, DriveConstants.kTrackwidthMeters,
          DriveConstants.kWheelDiameterMeters / 2.0, VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

      // the Field2d class lets us visualize our robot in the simulation GUI.
      m_fieldSim = new Field2d();
      SmartDashboard.putData("Field", m_fieldSim);
    }
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()),
        CTREConvert.nativeUnitsToDistanceMeters(m_leftMotors.getSelectedSensorPosition()),
        CTREConvert.nativeUnitsToDistanceMeters(m_rightMotors.getSelectedSensorPosition()));
    m_fieldSim.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and
    // gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    m_drivetrainSimulator.setInputs(m_leftMotors.getMotorOutputVoltage(), -m_rightMotors.getMotorOutputVoltage());
    m_drivetrainSimulator.update(0.020);

    // Update all of our sensors.
    m_leftDriveSim
        .setQuadratureRawPosition(CTREConvert.distanceToNativeUnits(m_drivetrainSimulator.getLeftPositionMeters()));
    m_leftDriveSim.setQuadratureVelocity(
        CTREConvert.velocityToNativeUnits(m_drivetrainSimulator.getLeftVelocityMetersPerSecond()));
    m_rightDriveSim
        .setQuadratureRawPosition(CTREConvert.distanceToNativeUnits(m_drivetrainSimulator.getRightPositionMeters()));
    m_rightDriveSim.setQuadratureVelocity(
        CTREConvert.velocityToNativeUnits(m_drivetrainSimulator.getRightVelocityMetersPerSecond()));

    // Update NavX gyro (per
    // https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java/)
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  /**
   * Returns the current being drawn by the drivetrain. This works in SIMULATION
   * ONLY! If you want it to work elsewhere, use the code in
   * {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
   *
   * @return The drawn current in Amps.
   */
  public double getDrawnCurrentAmps() {
    return m_drivetrainSimulator.getCurrentDrawAmps();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        CTREConvert.nativeUnitsToVelocityMetersPerSecond(m_leftMotors.getSelectedSensorVelocity()),
        CTREConvert.nativeUnitsToVelocityMetersPerSecond(m_rightMotors.getSelectedSensorVelocity()));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_drivetrainSimulator.setPose(pose);
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    var batteryVoltage = RobotController.getBatteryVoltage();
    if (Math.max(Math.abs(leftVolts), Math.abs(rightVolts)) > batteryVoltage) {
      leftVolts *= batteryVoltage / 12.0;
      rightVolts *= batteryVoltage / 12.0;
    }
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftMotors.setSelectedSensorPosition(0);
    m_rightMotors.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  // public double getAverageEncoderDistance() {
  // return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  // }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    m_gyro.zeroYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
