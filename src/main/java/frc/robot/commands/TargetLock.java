// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TargetLock extends Command {
  /** Creates a new Drive. */
  DriveSubsystem driveSubsystem;
  boolean clockwise;
  long lockTime;//time on target

  public TargetLock(DriveSubsystem driveSystem, boolean clockwise) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
    this.clockwise = clockwise;
    driveSubsystem = driveSystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lockTime = System.currentTimeMillis();//initialize lock time to start time
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Setting the values here instead of in initialize feeds the watchdog on the
  // arcade drive object
  @Override
  public void execute() {
    // Basic targeting data
double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

double txnc = LimelightHelpers.getTXNC("");  // Horizontal offset from principal pixel/point to target in degrees
double tync = LimelightHelpers.getTYNC("");  // Vertical offset from principal pixel/point to target in degrees
    
if (hasTarget)
{

  double maxZ = 0.03;
  double maxX = 0.03;

  //positive is counter-clockwise, negative is clockwise
  double zSpeed = -tx/(50);
  if(Math.abs(tx)<5){
    zSpeed = 0;
  }
  double xSpeed = -ty/(30);
  if(Math.abs(ty)<5){
    xSpeed=0;
  }

  if (zSpeed > maxZ)
  {
    zSpeed = maxZ;
  }
  if (zSpeed < -maxZ)
  {
    zSpeed = -maxZ;
  }
  if (xSpeed > maxX)
  {
    xSpeed = maxX;
  }
  if (xSpeed < -maxX)
  {
    xSpeed = -maxX;
  }
  //driveSubsystem.driveArcade(xSpeed,zSpeed);//Choose center coordinates
   driveSubsystem.drive(xSpeed,0,zSpeed,false);
   //for troubleshooting
   SmartDashboard.putNumber("xspeed", xSpeed);
   SmartDashboard.putNumber("zSpeed", zSpeed);
} else {
  if(clockwise){
    //driveSubsystem.driveArcade(0,-0.15); // Robot is rotating slowly if theres no target
     driveSubsystem.drive(0,0,-0.03,false);
  } else {
    //driveSubsystem.driveArcade(0,0.15); // Robot is rotating slowly if theres no target
     driveSubsystem.drive(0,0,0.03,false);
  }
  
}

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0,0,0,false);//stop moving 
    //driveSubsystem.driveArcade(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
//    return false;
double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?
    if((Math.abs(tx)<2 && Math.abs(ty)<2 && hasTarget)){
      return (System.currentTimeMillis()-lockTime) > 500;//ensure stabilized target
    }
    else{
      lockTime = System.currentTimeMillis();//reset time to current time
    }
  return false;

  }
}
