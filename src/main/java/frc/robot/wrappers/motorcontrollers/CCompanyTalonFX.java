// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrappers.motorcontrollers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * This class is an encapsulation of WPI_TalonSRX that add a couple constructors
 * for forcing common settings. It's heavily inspired from team 6559 Nomads. It
 * takes inspiration for the lazy feature from team 254.
 */


public class CCompanyTalonFX extends WPI_TalonFX {
    /** This decides if the talon should operate in lazy mode. */
    protected boolean lazy = false;

    protected double lastPower = Double.NaN;
    protected ControlMode lastMode = null;

    /**
     * Constructs a TalonSRX, reverts it to factory default, and sets brake mode.
     * 
     * @param port The CAN ID of this Talon
     */
    public CCompanyTalonFX(int port) {
        super(port);
        configFactoryDefault();
        setNeutralMode(NeutralMode.Coast);
        setSafetyEnabled(false);
    }

    /**
     * Constructs a TalonSRX, reverts it to factory default, and sets brake mode and
     * inversion status.
     * 
     * @param port     The CAN ID of this Talon.
     * @param inverted invert direction of speed controller. True for inverted,
     *                 false if not.
     */
    public CCompanyTalonFX(int port, boolean inverted) {
        this(port);
        setInverted(inverted);
    }

    /**
     * Constructs a TalonSRX, reverts it to factory default, sets brake mode and
     * inversion status, and slaves it to a specified CCompanyTalonFX.
     * 
     * @param port     The CAN ID of this Talon.
     * @param inverted invert direction of speed controller. True for inverted,
     *                 false if not.
     * @param master   The CCompanyTalonFX to follow.
     */
    public CCompanyTalonFX(int port, boolean inverted, CCompanyTalonSRX master) {
        this(port, inverted);
        follow(master);
    }

    /**
     * Constructs a TalonSRX, reverts it to factory default, sets brake mode and
     * inversion status, and slaves it to a specified CCompanyTalonFX.
     * 
     * @param port     The CAN ID of this Talon.
     * @param inverted invert direction of speed controller. True for inverted,
     *                 false if not.
     * @param phase    True for inverted, false if not.
     */
    public CCompanyTalonFX(int port, boolean inverted, boolean phase) {
        this(port, inverted);
        setSensorPhase(phase);

    }

    /**
     * Constructs a TalonSRX, reverts it to factory default, sets brake mode and
     * inversion status, and slaves it to a specified CCompanyTalonFX.
     * 
     * @param port     The CAN ID of this Talon.
     * @param inverted invert direction of speed controller. True for inverted,
     *                 false if not.
     * @param phase    True for inverted, false if not.
     * @param master   The CCompanyTalonSRX to follow.
     */
    public CCompanyTalonFX(int port, boolean inverted, boolean phase, CCompanyTalonSRX master) {
        this(port, inverted, phase);
        follow(master);
    }

    /**
     * Constructs a TalonSRX, reverts it to factory default, sets brake mode and
     * inversion status, and slaves it to a specified CCompanyTalonFX.
     * 
     * @param port     The CAN ID of this Talon.
     * @param inverted invert direction of speed controller. True for inverted,
     *                 false if not.
     * @param phase    True for inverted, false if not.
     * @param master   The CCompanyTalonFX to follow.
     */
    public CCompanyTalonFX(int port, boolean inverted, boolean phase, CCompanyTalonFX master) {
        this(port, inverted, phase);
        follow(master);
    }

    /**
     * Check if the motor controller is lazy
     * 
     * @return Whether the motor controller is lazy
     */
    public boolean isLazy() {
        return lazy;
    }

    /**
     * Set the lazy mode
     * 
     * @param isLazy A boolean for the lazy mode, where true is lazy on
     */
    public void setLazy(boolean isLazy) {
        lazy = isLazy;
    }

    /**
     * Sets the appropriate output on the talon, depending on the mode. If in lazy
     * mode, it will not call set unless the value or mode is different.
     * 
     * @param mode  The output mode to apply. In PercentOutput, the output is
     *              between -1.0 and 1.0, with 0.0 as stopped. In Current mode,
     *              output value is in amperes. In Velocity mode, output value is in
     *              position change / 100ms. In Position mode, output value is in
     *              encoder ticks or an analog value, depending on the sensor. In
     *              Follower mode, the output value is the integer device ID of the
     *              talon to duplicate.
     *
     * @param value The setpoint value, as described above.
     *
     *
     *              Standard Driving Example:
     *              _talonLeft.set(ControlMode.PercentOutput, leftJoy);
     *              _talonRght.set(ControlMode.PercentOutput, rghtJoy);
     */
    @Override
    public void set(ControlMode mode, double value) {
        if (lazy) {
            if (value != lastPower || mode != lastMode) {
                lastPower = value;
                lastMode = mode;
                super.set(mode, value);
            }
        } else {
            super.set(mode, value);
        }
    }
}
