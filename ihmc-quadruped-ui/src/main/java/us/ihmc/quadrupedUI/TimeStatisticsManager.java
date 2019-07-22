package us.ihmc.quadrupedUI;

import java.text.DecimalFormat;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.RobotConfigurationData;
import javafx.animation.AnimationTimer;
import javafx.scene.control.Label;
import us.ihmc.commons.Conversions;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.messager.MessagerAPIFactory.Topic;

public class TimeStatisticsManager extends AnimationTimer
{
   private final Label timeSinceLastUpdateLabel;
   private final Label lastControllerTimeLabel;

   private final DecimalFormat format = new DecimalFormat("0.000");
   private final AtomicReference<RobotConfigurationData> robotConfigurationDataReference;
   private final AtomicReference<String> newTimeToDisplay = new AtomicReference<>(null);

   public TimeStatisticsManager(Label timeSinceLastUpdateLabel, Label lastControllerTimeLabel, JavaFXMessager messager,
                                Topic<RobotConfigurationData> robotConfigurationDataTopic)
   {
      this.timeSinceLastUpdateLabel = timeSinceLastUpdateLabel;
      this.lastControllerTimeLabel = lastControllerTimeLabel;

      robotConfigurationDataReference = messager.createInput(robotConfigurationDataTopic, null);

      messager.registerTopicListener(robotConfigurationDataTopic, m ->
      {
         if (m != null && newTimeToDisplay.get() == null)
         {
            newTimeToDisplay.set(format.format(Conversions.nanosecondsToSeconds(m.getMonotonicTime())));
         }
      });
   }

   private long lastUpdateTimestamp = -1L;

   @Override
   public void handle(long now)
   {
      RobotConfigurationData robotConfigurationData = robotConfigurationDataReference.getAndSet(null);

      if (robotConfigurationData != null)
      {
         lastUpdateTimestamp = now;
         String timeFormatted = format.format(Conversions.nanosecondsToSeconds(robotConfigurationData.getMonotonicTime()));
         lastControllerTimeLabel.setText(timeFormatted);
      }

      if (lastUpdateTimestamp > 0L)
      {
         String timeFormatted = format.format(Conversions.nanosecondsToSeconds(now - lastUpdateTimestamp));
         timeSinceLastUpdateLabel.setText(timeFormatted);
      }
   }
}
