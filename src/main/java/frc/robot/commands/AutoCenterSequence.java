// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ProtoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ProtoLauncher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCenterSequence extends SequentialCommandGroup {
  /** Creates a new LaunchSequence. */
  public AutoCenterSequence(DriveSubsystem driveSubsystem, ProtoLauncher fuelSubsystem) {
    TrajectoryConfig configslow = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);
        
         var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

      Trajectory back_up = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(Math.PI)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(-1, 0) ,new Translation2d(-2, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(-2.44, 0, new Rotation2d(Math.PI)),
        configslow);

           SwerveControllerCommand back_up_command = new SwerveControllerCommand(
        back_up,
        driveSubsystem::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        driveSubsystem::setModuleStates,
        driveSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
       // new TargetLock(driveSubsystem, false),
       // new BackUp(driveSubsystem).withTimeout(ProtoConstants.AUTO_BACKUP_SECONDS),
      //back up
        
        //back_up_command.andThen(() -> driveSubsystem.drive(0, 0, 0, false)),
        new RunCommand(()->driveSubsystem.drive(-0.1, 0, 0, false), driveSubsystem).withTimeout(0.5).andThen(() -> driveSubsystem.drive(0, 0, 0, false)),
        new ProtoSpinUp(fuelSubsystem).withTimeout(ProtoConstants.PROTO_SPIN_UP),
        new ProtoLaunch(fuelSubsystem).withTimeout(ProtoConstants.AUTO_LAUNCH_SECONDS)
        );

        //center start
        //move back until target detected
        //rotate+shoot
        //navigate to climbing tower side via QR codes
        //raise the climber
        //move to prepare to climb
        //retract the climber
        //
  }
}
