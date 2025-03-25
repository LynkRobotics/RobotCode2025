package frc.lib.util;

import dev.doglog.DogLog;
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
        DogLog.log("Fault/Error", subsystem + ": " + title + " " + msg);
        
        DogLog.log("Fault/All Faults", subsystem + ": " + "Error: " + title + " " + msg);
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
        DogLog.log("Fault/Warning", subsystem + ": " + title + " " + msg);
        DogLog.log("Fault/All Faults", subsystem + ": " + "Warning: " + title + " " + msg);
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
        Elastic.sendNotification(new Notification(NotificationLevel.INFO, title, msg));
        DogLog.log("Fault/Info", subsystem + ": " + title + " " + msg);
        DogLog.log("Fault/All Faults",  subsystem + ": " + "Info: " + title + " " + msg);
    }
}