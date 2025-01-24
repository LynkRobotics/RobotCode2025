// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class EjectCommand extends LoggedCommandBase {
  private final IntakeSubsystem intake;
  private final IndexSubsystem index;
  private final ShooterSubsystem shooter;

  public EjectCommand(IntakeSubsystem intake, IndexSubsystem index, ShooterSubsystem shooter) {
    super();
    addRequirements(intake, index, shooter);
    this.intake = intake;
    this.index = index;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    intake.eject();
    index.eject();
    shooter.eject();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    intake.stop();
    index.stop();
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}