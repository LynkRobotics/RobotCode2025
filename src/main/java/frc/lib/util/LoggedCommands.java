// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

// Provide a Commands-like interface, but producing LoggedCommands
public class LoggedCommands {
    public static final String key = "Robot/Status";

    public static void logInit(Command command) {
        DogLog.log(key, "Running " + command.getName());
    }

    public static void logFinish(Command command, boolean interrupted) {
        DogLog.log(key, "Finished " + command.getName() + (interrupted ? " (interrupted)" : ""));
    }

    public static Command loggedCommand(Command command) {
        Command loggedCommand = new WrapperCommand(command) {
            @Override
            public void initialize() {
                logInit(command);
                super.initialize();
            }

            @Override
            public void end(boolean interrupted) {
                logFinish(command, interrupted);
                super.end(interrupted);
            }
        };

        loggedCommand.setName(command.getName() + " (Logged)");
        return loggedCommand;
    }

    public static Command logWithName(String name, Command command) {
        command.setName(name);
        return loggedCommand(command);
    }

    /* Custom convenience commands */

    // Use DogLog to log a message to the same key as the logged commands
    // Note that the command itself is *not* logged, since the message itself serves
    // as the log entry
    public static Command log(String message) {
        return Commands.runOnce(() -> {
            DogLog.log(key, message);
        });
    }

    // Use DogLog to log a message to the same key as the logged commands
    // In this variant, the message is constructed at the time of invocation by the
    // provided Supplier
    // Note that the command itself is *not* logged, since the message itself serves
    // as the log entry
    public static Command log(Supplier<String> supplier) {
        return Commands.runOnce(() -> {
            DogLog.log(key, supplier.get());
        });
    }

    public static Command proxy(Command command) {
        return logWithName("Proxy: " + command.getName(), command.asProxy());
    }

    /* The following map to the static utilities from the standard Commands class */

    public static Command none(String name) {
        return logWithName(name, Commands.none());
    }

    public static Command idle(String name, Subsystem... requirements) {
        return logWithName(name, Commands.idle(requirements));
    }

    public static Command runOnce(String name, Runnable action, Subsystem... requirements) {
        return logWithName(name, Commands.runOnce(action, requirements));
    }

    public static Command run(String name, Runnable action, Subsystem... requirements) {
        return logWithName(name, Commands.run(action, requirements));
    }

    public static Command startEnd(String name, Runnable start, Runnable end, Subsystem... requirements) {
        return logWithName(name, Commands.startEnd(start, end, requirements));
    }

    public static Command runEnd(String name, Runnable run, Runnable end, Subsystem... requirements) {
        return logWithName(name, Commands.runEnd(run, end, requirements));
    }

    public static Command startRun(String name, Runnable start, Runnable run, Subsystem... requirements) {
        return logWithName(name, Commands.startRun(start, run, requirements));
    }

    public static Command print(String name, String message) {
        return logWithName(name, Commands.print(message));
    }

    public static Command waitSeconds(String name, double seconds) {
        return logWithName(name, Commands.waitSeconds(seconds));
    }

    public static Command waitUntil(String name, BooleanSupplier condition) {
        return logWithName(name, Commands.waitUntil(condition));
    }

    public static Command either(String name, Command onTrue, Command onFalse, BooleanSupplier selector) {
        return logWithName(name, Commands.either(onTrue, onFalse, selector));
    }

    public static <K> Command select(String name, Map<K, Command> commands, Supplier<? extends K> selector) {
        return logWithName(name, Commands.select(commands, selector));
    }

    public static Command defer(String name, Supplier<Command> supplier, Set<Subsystem> requirements) {
        return logWithName(name, Commands.defer(supplier, requirements));
    }

    public static Command deferredProxy(String name, Supplier<Command> supplier) {
        return logWithName(name, Commands.deferredProxy(supplier));
    }

    public static Command sequence(String name, Command... commands) {
        return logWithName(name, Commands.sequence(commands));
    }

    public static Command repeatingSequence(String name, Command... commands) {
        return logWithName(name, Commands.repeatingSequence(commands));
    }

    public static Command parallel(String name, Command... commands) {
        return logWithName(name, Commands.parallel(commands));
    }

    public static Command race(String name, Command... commands) {
        return logWithName(name, Commands.race(commands));
    }

    public static Command deadline(String name, Command deadline, Command... otherCommands) {
        return logWithName(name, Commands.deadline(deadline, otherCommands));
    }

    private LoggedCommands() {
        throw new UnsupportedOperationException("This is a utility class");
    }
}