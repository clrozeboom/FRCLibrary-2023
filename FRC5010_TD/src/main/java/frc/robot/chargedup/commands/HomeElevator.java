// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup.commands;

import frc.robot.FRC5010.constants.GenericCommand;
import frc.robot.chargedup.ElevatorLevel;
import frc.robot.chargedup.ElevatorSubsystem;
import frc.robot.chargedup.PivotSubsystem;

public class HomeElevator extends GenericCommand {
  ElevatorSubsystem elevatorSubsystem;
  PivotSubsystem pivot;

  /** Creates a new HomeElevator. */
  public HomeElevator(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivot) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.pivot = pivot;
    addRequirements(elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {
    ElevatorSubsystem.setElevatorTargetLevel(ElevatorLevel.ground);
    elevatorSubsystem.initializeRunToTarget(ElevatorLevel.ground.getExtensionPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.elevatorSubsystem.runExtendToTarget(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void stop(boolean interrupted) {
    this.elevatorSubsystem.stopAndHoldExtend();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.elevatorSubsystem.isElevatorIn();
  }
}
