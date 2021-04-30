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

    private final VictorSPX elevatorMotor = new VictorSPX(CanIdConstants.ELEVATOR_MOTOR_ID);

    public ElevatorSubsystem() {
        elevatorMotor.configOpenloopRamp(1);
        elevatorMotor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Sets the elevator velocity. Value is calmped between -1.0 and 1.0.
     *
     * @param speed the velocity to set the motor to.
     */
    public void setElevatorVelocity(double speed) {
        if (speed > 1.0) {
            speed = 1.0;
        } else if (speed < -1.0) {
            speed = -1.0;
        }
        elevatorMotor.set(ControlMode.PercentOutput, speed);
    }
}
