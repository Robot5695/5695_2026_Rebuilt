// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ProtoLauncher;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EjectFuel extends Command {
  IntakeSubsystem intakeSubsystem;
  ProtoLauncher protoLauncher;
  /** Creates a new EjectFuel. */
  public EjectFuel(IntakeSubsystem intakeSubsystem, ProtoLauncher protoLauncher) {
    this.intakeSubsystem = intakeSubsystem;
    this.protoLauncher = protoLauncher;
    addRequirements(intakeSubsystem, protoLauncher);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.setIntakeRoller(-0.5);
    protoLauncher.setFeederRoller(-0.5);
    protoLauncher.setIndexRoller(-0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeRoller(0);
    protoLauncher.setFeederRoller(0);
    protoLauncher.setIndexRoller(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
