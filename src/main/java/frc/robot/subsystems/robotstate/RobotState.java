package frc.robot.subsystems.robotstate;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotState extends SubsystemBase {
    public static final RobotState instance = new RobotState();

    static {
    }

    public static boolean haveAlgae() {
        return false; // TODO
    }

    public static boolean haveCoral() {
        return false; // TODO
    }

    public static boolean coralReady() {
        return false; // TODO
    }

    @Override
    public void periodic() {
    }
}