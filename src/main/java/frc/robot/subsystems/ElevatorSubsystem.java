// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CanIdConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private final VictorSPX motor = new VictorSPX(CanIdConstants.ELEVATOR_MOTOR_ID);

    /** Creates a new ElevatorSubsystem */
    public ElevatorSubsystem() {
        motor.configOpenloopRamp(1);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Sets the elevator velocity. Value is clamped between -1.0 and 1.0.
     *
     * @param velo the velocity to set the motor to.
     */
    public void setElevatorVelo(double velo) {
        if (velo > 1.0) {
            velo = 1.0;
        } else if (velo < -1.0) {
            velo = -1.0;
        }
        motor.set(ControlMode.PercentOutput, velo);
    }
}
