// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.numbers.N2;

/** Add your docs here. */
public final class DriveConstants {
        public static final int kLeftFrontMotorPort = 15;
        public static final int kLeftRearMotorPort = 14;
        public static final int kRightFrontMotorPort = 13;
        public static final int kRightRearMotorPort = 12;

        public static final boolean kLeftInvertMotor = false;
        public static final boolean kLeftInvertSensorPhase = true;

        public static final boolean kRightInvertMotor = true;
        public static final boolean kRightInvertSensorPhase = true;

        public static final double kTrackwidthMeters = 0.65;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                        kTrackwidthMeters);

        public static final int kEncoderCPR = 2048;
        public static final double kWheelDiameterMeters = (Units.inchesToMeters(4.58));
        public static final double kEncoderDistancePerPulse =
                        // Assumes the encoders are directly mounted on the wheel shafts
                        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        public static final boolean kGyroReversed = true;

        public static final double ksVolts = 0.0732;
        public static final double kvVoltSecondsPerMeter = 3.28;
        public static final double kaVoltSecondsSquaredPerMeter = 0.292;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // These two values are "angular" kV and kA
        public static final double kvVoltSecondsPerRadian = 1.5;
        public static final double kaVoltSecondsSquaredPerRadian = 0.3;

        public static final LinearSystem<N2, N2, N2> kDrivetrainPlant = LinearSystemId.identifyDrivetrainSystem(
                        kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter, kvVoltSecondsPerRadian,
                        kaVoltSecondsSquaredPerRadian);

        public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(2);
        public static final double kDriveGearing = 10.52;

        public static final double kDistancePerRevolution = (kWheelDiameterMeters * Math.PI)
                        / (kDriveGearing * kEncoderCPR);

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 0.00182; // 0.1;

}
