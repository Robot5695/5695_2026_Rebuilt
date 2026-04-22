// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ProtoLauncher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightSideAuto extends SequentialCommandGroup {
  /** Creates a new NeutralPickupScore. */
  public RightSideAuto(DriveSubsystem m_robotDrive, ProtoLauncher fuelSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // Create config for trajectory
    TrajectoryConfig configslow = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

        TrajectoryConfig configfast = new TrajectoryConfig(
        AutoConstants.kMaxReturnSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

         var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);


    // Motion from left trench to neutral zone
    Trajectory left_trench_to_neutral = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(2, 0),new Translation2d(4, 1)/* ,
        new Translation2d(1,0),
        new Translation2d(0,0),
        new Translation2d(-1,0),
        new Translation2d(-1,-3) */),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(4, 2, new Rotation2d(Math.PI/4)),
        configslow);

        // Motion from neutral zone to left trench
    Trajectory neutral_to_left_trench = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(4, 1, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1.5, -0.8) ,new Translation2d(-2, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(-1.6, 0.7, new Rotation2d(Math.PI/4)),
        configfast);

        
   
    SwerveControllerCommand left_trench_to_neutral_command = new SwerveControllerCommand(
        left_trench_to_neutral,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

 SwerveControllerCommand neutral_to_left_trench_command = new SwerveControllerCommand(
        neutral_to_left_trench,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    //m_robotDrive.resetOdometry(left_trench_to_neutral.getInitialPose());
    
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //deploy intake
      new Intake(intakeSubsystem).withTimeout(0.5),
      
      //drive to neutral zone balls
      left_trench_to_neutral_command,
      neutral_to_left_trench_command.andThen(() -> m_robotDrive.drive(0, 0, 0, false)),
      
      //target lock
      //new TargetLock(m_robotDrive,false),
      //spin
      new ProtoLaunchSequence(fuelSubsystem).withTimeout(10)
      //launch
    );
  }


}
