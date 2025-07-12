package frc.robot;

import frc.robot.autos.Autos;
import frc.robot.subsystems.robotstate.RobotState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.controls.Controls;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pose.Pose;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Subsystems */
    @SuppressWarnings ("unused")
    private final RobotState s_RobotState;
    @SuppressWarnings ("unused")
    private final Swerve s_Swerve;
    @SuppressWarnings ("unused")
    private final LED s_LED;
    @SuppressWarnings ("unused")
    private final Vision s_Vision;
    @SuppressWarnings ("unused")
    private final Pose s_Pose;
    @SuppressWarnings ("unused")
    private final Elevator s_Elevator;
    @SuppressWarnings ("unused")
    private final EndEffector s_EndEffector;
    @SuppressWarnings ("unused")
    private final Index s_Index;
    @SuppressWarnings ("unused")
    private final Climber s_Climber;
    @SuppressWarnings ("unused")
    private final Controls s_Controls;
    @SuppressWarnings ("unused")
    private final Autos s_Autos;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Initial Subsystems
        s_Swerve = new Swerve();
        s_Vision = new Vision();
        s_Pose = new Pose();
        s_RobotState = new RobotState();
        s_Elevator = new Elevator();
        s_EndEffector = new EndEffector();
        s_LED = new LED();
        s_Index = new Index();
        s_Climber = new Climber();
        s_Controls = new Controls();
        s_Autos = new Autos();
    }
}