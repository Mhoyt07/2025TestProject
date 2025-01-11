// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraNT extends SubsystemBase {
  /** Creates a new CameraNT. */
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  double pipeline;
  double x;
  double y;
  double area;
  public CameraNT() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
  }

  public double[] get_data() {
    x = tx.getDouble(0.0);
    y = tx.getDouble(0.0);
    area = ta.getDouble(0.0);
    return new double[] {x, y, area};
  }

  public void set_pipeline(int pipe_num) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipe_num);
    pipeline = pipe_num;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Limelight X" , x);
    SmartDashboard.putNumber("Limelight Y", y);
    SmartDashboard.putNumber("Limelight Area", area);
    SmartDashboard.putNumber("Pipeline", pipeline);
  }
}
