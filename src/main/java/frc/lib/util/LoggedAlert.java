package frc.lib.util;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.util.Elastic.Notification;
import frc.lib.util.Elastic.Notification.NotificationLevel;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;

public class LoggedAlert {
    /**
     * Creates a new Logged Error Notification, sent to the dashboard and DogLog
     *
     * @param subsystem the subsystem of the logged error notification
     * @param title the title of the logged error notification
     * @param msg the message of the logged error notification
     */
    public static void Error(String subsystem, String title, String msg) {
        Elastic.sendNotification(new Notification(NotificationLevel.ERROR, title, msg, Constants.errorTime));
        String formattedError = new String(subsystem + ": " + title + ": " + msg); //Constant Formatting
        DogLog.logFault(formattedError, AlertType.kError); //Errors only
        DogLog.logFault(formattedError); //All Faults
        LEDSubsystem.triggerError(); //TODO: i dont really like subsystem calls in util classes
    }

    /**
     * Creates a new Logged Warning Notification, sent to the dashboard and DogLog
     *
     * @param subsystem the subsystem of the logged warning notification
     * @param title the title of the logged warning notification
     * @param msg the message of the logged warning notification
     */
    public static void Warning(String subsystem, String title, String msg) {
        Elastic.sendNotification(new Notification(NotificationLevel.WARNING, title, msg, Constants.warningTime));
        String formattedWarning = new String(subsystem + ": " + title + ": " + msg); //Constant formatting
        DogLog.logFault(formattedWarning, AlertType.kWarning); //Warnings only
        DogLog.logFault(formattedWarning); //All Faults
        LEDSubsystem.triggerWarning();
    }

    /**
     * Creates a new Logged Info Notification, sent to the dashboard and DogLog
     *
     * @param subsystem the subsystem of the logged info notification
     * @param title the title of the logged info notification
     * @param msg the message of the logged info notification
     */
    public static void Info(String subsystem, String title, String msg) {
        Elastic.sendNotification(new Notification(NotificationLevel.INFO, title, msg, Constants.infoTime));
        String formattedInfo = new String(subsystem + ": " + title + ": " + msg); //Constant formatting
        DogLog.logFault(formattedInfo, AlertType.kInfo); //Info(s) only
        DogLog.logFault(formattedInfo); //All Faults
        LEDSubsystem.triggerInfo();
    }
}