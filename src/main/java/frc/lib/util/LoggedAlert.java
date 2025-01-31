package frc.lib.util;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.util.Elastic.Notification;
import frc.lib.util.Elastic.Notification.NotificationLevel;

public class LoggedAlert {
    /**
     * Creates a new Logged Error Notification, sent to the dashboard and DogLog
     *
     * @param subsystem the subsystem of the logged error notification
     * @param title the title of the logged error notification
     * @param msg the message of the logged error notification
     */
    public static void Error(String subsystem, String title, String msg){
        Elastic.sendNotification(new Notification(NotificationLevel.ERROR, title, msg));
        DogLog.logFault(subsystem + "/" + msg, AlertType.kError);
    }

    /**
     * Creates a new Logged Warning Notification, sent to the dashboard and DogLog
     *
     * @param subsystem the subsystem of the logged warning notification
     * @param title the title of the logged warning notification
     * @param msg the message of the logged warning notification
     */
    public static void Warning(String subsystem, String title, String msg){
        Elastic.sendNotification(new Notification(NotificationLevel.WARNING, title, msg));
        DogLog.logFault(subsystem + "/" + msg, AlertType.kWarning);
    }

    /**
     * Creates a new Logged Info Notification, sent to the dashboard and DogLog
     *
     * @param subsystem the subsystem of the logged info notification
     * @param title the title of the logged info notification
     * @param msg the message of the logged info notification
     */
    public static void Info(String subsystem, String title, String msg){
        Elastic.sendNotification(new Notification(NotificationLevel.INFO, title, msg));
        DogLog.logFault(subsystem + "/" + msg, AlertType.kInfo);
    }
}
