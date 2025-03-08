// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.wpilibj2.command.Command;

// Wraps a command with logging
public class LoggedCommandBase extends Command {
    public LoggedCommandBase() {
        super();
    }

    @Override
    public void initialize() {
        LoggedCommands.logInit(this);
    }

    @Override
    public void end(boolean interrupted) {
        LoggedCommands.logFinish(this, interrupted);
    }
}