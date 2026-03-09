// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ProtoConstants.*;

public class ClimberSubsystem extends SubsystemBase {
  /* private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller; */
//private final TalonSRX feederRoller; //Rollers on back of prototype launcher
//private final TalonSRX intakeRoller; //Orange star at center of launcher
private final DoubleSolenoid climber; 

  /** Creates a new CANBallSubsystem. */
  public ClimberSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    /* intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushed);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushed); */
    climber = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
 // intakeRoller = new TalonSRX(PROTO_INTAKE_MOTOR_ID);
 // feederRoller = new TalonSRX(PROTO_FEEDER_MOTOR_ID);
    
    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
   
    
  }

  // A method to set the voltage of the intake roller
  

  /*/ A method to set the voltage of the intake roller
  public void setFeederRoller(double voltage) {
    feederRoller.setVoltage(voltage);
    feederRoller.set(TalonSRXControlMode.PercentOutput, voltage);
  }*/

public void extend(boolean extendState){
  if (extendState){
    climber.set(DoubleSolenoid.Value.kForward);
    
  } else{
    climber.set(DoubleSolenoid.Value.kReverse);
  }
}

  // A method to stop the rollers
  public void stop() { 
   // feederRoller.set(TalonSRXControlMode.PercentOutput,0);
   // intakeRoller.set(TalonSRXControlMode.PercentOutput,0);
    climber.set(DoubleSolenoid.Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
