// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  SparkMax r_motor;
  SparkMax l_motor;
  SparkMaxConfig config_r;
  SparkMaxConfig config_l;
  public DriveTrain() {
    r_motor = new SparkMax(0, MotorType.kBrushless);
    l_motor = new SparkMax(1, MotorType.kBrushless);

    config_r
      .inverted(false)
      .idleMode(IdleMode.kBrake);
    
    config_l
      .inverted(true)
      .idleMode(IdleMode.kBrake);

    r_motor.configure(config_r, 
    ResetMode.kResetSafeParameters, //when configure is run it will reset some saved data excluding certain things
    PersistMode.kNoPersistParameters);// when powercycled it will not store data in volatile storage

    l_motor.configure(config_l,
    ResetMode.kResetSafeParameters, //when configure is run it will reset some saved data excluding certain things
    PersistMode.kNoPersistParameters);// when powercycled it will not store data in volatile storage
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
