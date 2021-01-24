// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility.ctre;

import frc.robot.constants.DriveConstants;

/** Add your docs here. */
public class CTREConvert {

    /**
     * Convert distance in meters to CTRE native units
     * 
     * @param positionMeters in meters
     * @return native unit sensor counts
     */
    public static int distanceToNativeUnits(double positionMeters) {
        double wheelRotations = positionMeters / (Math.PI * DriveConstants.kWheelDiameterMeters);
        double motorRotations = wheelRotations * DriveConstants.kDriveGearing;
        int sensorCounts = (int) (motorRotations * DriveConstants.kEncoderCPR);
        return sensorCounts;
    }

    /**
     * Convert velocity from meters per second to sensor counts per 100ms
     * 
     * @param velocityMetersPerSecond
     * @return native unit sensor count per 100ms
     */
    public static int velocityToNativeUnits(double velocityMetersPerSecond) {
        double wheelRotationsPerSecond = velocityMetersPerSecond / (Math.PI * DriveConstants.kWheelDiameterMeters);
        double motorRotationsPerSecond = wheelRotationsPerSecond * DriveConstants.kDriveGearing;
        double motorRotationsPer100ms = motorRotationsPerSecond / 10;
        int sensorCountsPer100ms = (int) (motorRotationsPer100ms * DriveConstants.kEncoderCPR);
        return sensorCountsPer100ms;
    }

    /**
     * Convert CTRE native units to position in meters
     * 
     * @param sensorCounts from getSelectedSensorPosition()
     * @return position in meters
     */
    public static double nativeUnitsToDistanceMeters(double sensorCounts) {
        double motorRotations = (double) sensorCounts / DriveConstants.kEncoderCPR;
        double wheelRotations = motorRotations / DriveConstants.kDriveGearing;
        double positionMeters = wheelRotations * (Math.PI * DriveConstants.kWheelDiameterMeters);
        return positionMeters;
    }

    /**
     * Convert CTRE native units to velocity in meters per second
     * 
     * @param sensorCountsPer100ms from getSelectedSensorVelocity()
     * @return velocity in meters per second
     */
    public static double nativeUnitsToVelocityMetersPerSecond(double sensorCountsPer100ms) {
        double motorRotationsPer100ms = sensorCountsPer100ms / DriveConstants.kEncoderCPR;
        double motorRotationsPerSecond = motorRotationsPer100ms * 10;
        double wheelRotationsPersSecond = motorRotationsPerSecond / DriveConstants.kDriveGearing;
        double velocityMetersPerSecond = wheelRotationsPersSecond * Math.PI * DriveConstants.kWheelDiameterMeters;
        return velocityMetersPerSecond;
    }
}
