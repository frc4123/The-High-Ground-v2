// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CanIdConstants;

public class ShooterSubsystem extends SubsystemBase {
    // TODO make this a pid subsystem

    private final TalonSRX motorMaster = new TalonSRX(CanIdConstants.SHOOTER_MASTER_ID);
    private final TalonSRX motorSlave = new TalonSRX(CanIdConstants.SHOOTER_SLAVE_ID);

    /** Creates a new ShooterSubsystem. */
    public ShooterSubsystem() {
        // if motors are on the same side, set to follow, if not, comment out follow, and uncomment
        // shooterSlave.set(ControlMode.PercentOutput, -speed);
        motorSlave.follow(motorMaster);
    }

    /**
     * Sets the shooter velocity. Value is clamped between -1.0 and 1.0.
     *
     * @param velo the velocity to set the motor to.
     */
    public void setShooterVelo(double velo) {
        motorMaster.set(ControlMode.PercentOutput, velo);
        // shooterSlave.set(ControlMode.PercentOutput, -speed);
    }
}
