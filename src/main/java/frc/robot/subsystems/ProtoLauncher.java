// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ProtoConstants.*;

public class ProtoLauncher extends SubsystemBase {
  /* private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller; */
//private final TalonSRX feederRoller; //Rollers on back of prototype launcher
//private final TalonSRX intakeRoller; //Orange star at center of launcher
private final SparkFlex launcherRoller; //The launcher wheel
private final SparkFlex launcherRoller2;
private final SparkMax feederRoller; // REV Neo that feeds the launcher
private final Servo deflector;
private final RelativeEncoder launcherEncoder;

  /** Creates a new CANBallSubsystem. */
  public ProtoLauncher() {
    // create brushed motors for each of the motors on the launcher mechanism
    /* intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushed);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushed); */

 // intakeRoller = new TalonSRX(PROTO_INTAKE_MOTOR_ID);
 // feederRoller = new TalonSRX(PROTO_FEEDER_MOTOR_ID);
  launcherRoller = new SparkFlex(PROTO_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
  deflector = new Servo(0);
  launcherEncoder = launcherRoller.getEncoder();

  launcherRoller2 = new SparkFlex(PROTO_LAUNCHER_MOTOR2_ID, MotorType.kBrushless);
  feederRoller = new SparkMax(PROTO_FEEDER_MOTOR_ID, MotorType.kBrushless);
  SparkBaseConfig LauncherRoller2Config = new SparkFlexConfig();
  LauncherRoller2Config.follow(launcherRoller);
  launcherRoller2.configure(LauncherRoller2Config, com.revrobotics.ResetMode.kResetSafeParameters
  , com.revrobotics.PersistMode.kNoPersistParameters);
    
    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Proto feeder roller value", PROTO_FEEDER_POWER);
    SmartDashboard.putNumber("Proto intake roller value",PROTO_INTAKE_POWER);
    SmartDashboard.putNumber("Proto launch roller value", PROTO_LAUNCHER_POWER);
    
  }

  /* A method to set the voltage of the intake roller
  public void setIntakeRoller(double voltage) {
    intakeLauncherRoller.setVoltage(voltage);
    intakeRoller.set(TalonSRXControlMode.PercentOutput, voltage);
  }

  // A method to set the voltage of the intake roller
  public void setFeederRoller(double voltage) {
    feederRoller.setVoltage(voltage);
    feederRoller.set(TalonSRXControlMode.PercentOutput, voltage);
  }*/

// A method to set the power of the prototype launcher wheel
public void setLauncherRoller (double voltage) {
  //voltage is percent voltage, not voltage value
  launcherRoller.set(voltage);
}

public void setFeederRoller(double voltage){
  feederRoller.set(voltage);
}

public void setDeflector(double Angle){
  deflector.set(Angle);
}
  // A method to stop the rollers
  public void stop() {
   // feederRoller.set(TalonSRXControlMode.PercentOutput,0);
   // intakeRoller.set(TalonSRXControlMode.PercentOutput,0);
    launcherRoller.set(0);
    feederRoller.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Launcher Speed (RPM)", launcherEncoder.getVelocity());
  }
}
