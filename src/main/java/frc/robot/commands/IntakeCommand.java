// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.TempState;

public class IntakeCommand extends Command {
  private final IntakeSubsystem intake;
  private final IndexSubsystem index;
  private final GenericHID controller;

  public IntakeCommand(IntakeSubsystem intake, IndexSubsystem index, GenericHID controller) {
    addRequirements(intake, index);
    this.intake = intake;
    this.index = index;
    this.controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.intake();
    index.index();
    LEDSubsystem.setTempState(TempState.INTAKING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    index.stop();

    LEDSubsystem.clearTempState();

    if (!interrupted) {
      CommandScheduler.getInstance().schedule(
        Commands.startEnd(
          () -> { controller.setRumble(RumbleType.kLeftRumble, 1.0); controller.setRumble(RumbleType.kRightRumble, 1.0); },
          () -> { controller.setRumble(RumbleType.kLeftRumble, 0.0); controller.setRumble(RumbleType.kRightRumble, 0.0); })
        .raceWith(Commands.waitSeconds(0.5)));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return index.haveNote();
  }
}