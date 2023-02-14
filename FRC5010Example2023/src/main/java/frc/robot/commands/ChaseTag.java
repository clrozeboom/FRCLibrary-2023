// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.drive.GenericDrivetrain;
import frc.robot.FRC5010.drive.swerve.SwerveDrivetrain;


public class ChaseTag extends CommandBase {
  private SwerveDrivetrain swerveSubsystem;
  private final GenericPID pidTranslation = new GenericPID(1, 0, 0);
  private final GenericPID thetaTranslation = new GenericPID(.25, 0, 0);

  private final TrapezoidProfile.Constraints xConstraints;  
  private final TrapezoidProfile.Constraints yConstraints;  
  private final TrapezoidProfile.Constraints thetaConstraints; 
 
  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;


  private final Pose3d tagToGoal = new Pose3d(
    new Translation3d(1.5, 0.0, 0.0),
    new Rotation3d(0.0, 0.0, 0));

    
 
  private Supplier<Pose2d> poseProvider; 

  public ChaseTag(SwerveDrivetrain swerveSubsystem, Supplier<Pose2d> poseProvider) {
    xConstraints = new TrapezoidProfile.Constraints(swerveSubsystem.getSwerveConstants().getkPhysicalMaxSpeedMetersPerSecond(), swerveSubsystem.getSwerveConstants().getkTeleDriveMaxAccelerationUnitsPerSecond()); 
    yConstraints = new TrapezoidProfile.Constraints(swerveSubsystem.getSwerveConstants().getkPhysicalMaxSpeedMetersPerSecond(), swerveSubsystem.getSwerveConstants().getkTeleDriveMaxAccelerationUnitsPerSecond()); 
    thetaConstraints = new TrapezoidProfile.Constraints(swerveSubsystem.getSwerveConstants().getkPhysicalMaxAngularSpeedRadiansPerSecond(), swerveSubsystem.getSwerveConstants().getkTeleDriveMaxAngularAccelerationUnitsPerSecond()); 

    xController = new ProfiledPIDController(pidTranslation.getkP(), pidTranslation.getkI(), pidTranslation.getkD(), xConstraints);
    yController = new ProfiledPIDController(pidTranslation.getkP(), pidTranslation.getkI(), pidTranslation.getkD(), yConstraints);
    thetaController = new ProfiledPIDController(thetaTranslation.getkP(), thetaTranslation.getkI(), thetaTranslation.getkD(), thetaConstraints);
  
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = (SwerveDrivetrain) swerveSubsystem;
    this.poseProvider = poseProvider;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    thetaController.setTolerance(Units.degreesToRadians(3));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    addRequirements(swerveSubsystem);
    
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var robotPose = poseProvider.get(); 

    thetaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());

    xController.setGoal(tagToGoal.getX());
    yController.setGoal(tagToGoal.getY());
    thetaController.setGoal(tagToGoal.getRotation().getAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var robotPose2d = poseProvider.get();
    
    //System.out.println(robotPose2d);
    var robotPose = 
        new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0.0, 
            new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
    
    //System.out.println(robotPose);
    var xSpeed = xController.calculate(robotPose.getX()) * swerveSubsystem.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond() * 0.5; 
    var ySpeed = yController.calculate(robotPose.getY()) * swerveSubsystem.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond() * 0.5; 
    var thetaSpeed = thetaController.calculate(robotPose2d.getRotation().getRadians()) * swerveSubsystem.getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond();


    if (xController.atGoal()){
      xSpeed = 0;
    }

    if (yController.atGoal()){
      ySpeed = 0;
    }

    if (thetaController.atGoal()){
      thetaSpeed = 0;
    }
            
      // // Transform the tag's pose to set our goal
      
      // System.out.println(goalPose); 

      // // Drive
      xController.setGoal(tagToGoal.getX());
      yController.setGoal(tagToGoal.getY());
      thetaController.setGoal(0);

      System.out.println("xSpeed: " + xController.calculate(robotPose.getX()) + "\nySpeed: " + yController.calculate(robotPose.getY()) + 
      "\nTheta: " + thetaController.calculate(robotPose2d.getRotation().getRadians())); 

      
      //System.out.println(thetaSpeed);
    // swerveSubsystem.drive(new ChassisSpeeds(-xSpeed, -ySpeed, -thetaSpeed));
    //   System.out.println("xSpeed: " + xController.calculate(robotPose.getX()) + "\nySpeed: " + yController.calculate(robotPose.getY()) + 
    //   "\nTheta: " + thetaController.calculate(robotPose2d.getRotation().getRadians())); 

    // swerveSubsystem.drive(new ChassisSpeeds(
    // xController.calculate(robotPose.getX()) * swerveSubsystem.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond(), 
    // yController.calculate(robotPose.getY()) * swerveSubsystem.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond()
    // ,0));

    swerveSubsystem.drive(new ChassisSpeeds(0, 0, -thetaController.calculate(robotPose2d.getRotation().getRadians())));
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}