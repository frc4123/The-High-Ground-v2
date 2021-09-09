// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
    TalonSRX motor = new TalonSRX(Constants.CanIdConstants.INDEX_MOTOR_ID);

    /** Creates a new IndexerSubsystem. */
    public IndexerSubsystem() {
        motor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Sets the velocity of the indexer motor.
     * 
     * @param velo the velocity to set the indexer motor
     */
    public void setIndexVelo(double velo) {
        motor.set(ControlMode.PercentOutput, velo);
    }
}
