// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  SparkMax r_motor;
  SparkMax l_motor;
  SparkMaxConfig config_r;
  SparkMaxConfig config_l;
  RelativeEncoder r_encoder;
  RelativeEncoder l_encoder;
  double rx;
  double ly;
  DifferentialDrivePoseEstimator pose_estimator;
  Pigeon2 gyro;
  DifferentialDriveKinematics kinematics;
  LimelightHelpers.PoseEstimate mt2;
  boolean do_reject_update;
  Field2d field;
  public DriveTrain() {
    r_motor = new SparkMax(0, MotorType.kBrushless);
    l_motor = new SparkMax(1, MotorType.kBrushless);

    r_encoder = r_motor.getEncoder();
    l_encoder = l_motor.getEncoder();

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

    gyro = new Pigeon2(0);

    field = new Field2d();

    kinematics = new DifferentialDriveKinematics(Constants.drive.width);
    //the new Pose2d() might need a diffedrent rotation 2d
    //creates position estimator 
    pose_estimator = new DifferentialDrivePoseEstimator(kinematics, get_rotation_2d(), 0, 0, new Pose2d());
  }

  public void arcade(Joystick l, Joystick r) {
    rx = r.getX();
    ly = -l.getY();
    r_motor.set(ly-rx);
    l_motor.set(ly+rx);
  }

  //gets gyro angle
  public Rotation2d get_rotation_2d() {
    return gyro.getRotation2d();
  }

  public double get_yaw_rate() {
    return gyro.getAngularVelocityZWorld().getValueAsDouble();
  }

  //gets encoder values
  public double[] get_distances() {
    return new double[] {l_encoder.getPosition(), r_encoder.getPosition()};
  }

  public Pose2d get_pose() {
    return pose_estimator.getEstimatedPosition();
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //update pose estimator
    pose_estimator.update(get_rotation_2d(), get_distances()[0], get_distances()[1]);
    //can be done with network tables?
    LimelightHelpers.SetRobotOrientation("limelight", pose_estimator.getEstimatedPosition().getRotation().getDegrees(), get_yaw_rate(), 0, 0, 0, 0);
    mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if (Math.abs(get_yaw_rate()) > 720) {
        do_reject_update = true;
      } else if (mt2.tagCount == 0) {
        do_reject_update = true;
      } else {
        do_reject_update = false;
      } if (!do_reject_update) {
        pose_estimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        pose_estimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
      }
    field.setRobotPose(get_pose());
    SmartDashboard.putData("field", field);
  }
}
