// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdConstants;

public class IntakeWheelsSubsystem extends SubsystemBase {
  
  private final TalonSRX motor = new TalonSRX(CanIdConstants.INTAKE_WHEELS_ID);

  /** Creates a new IntakeWheels. */
  public IntakeWheelsSubsystem() {
    motor.configOpenloopRamp(1);
    motor.setNeutralMode(NeutralMode.Brake);
  }

  public void setIntakeWheelsVelo(double velo){
    motor.set(ControlMode.PercentOutput, velo);
  }
}
