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
   private final Label label;
   private final AtomicReference<String> newTimeToDisplay = new AtomicReference<>(null);
   private final DecimalFormat format = new DecimalFormat("0.000");
   

   public TimeStatisticsManager(Label label, JavaFXMessager messager, Topic<RobotConfigurationData> robotConfigurationDataTopic)
   {
      this.label = label;
      messager.registerTopicListener(robotConfigurationDataTopic, m ->
      {
         if (m != null && newTimeToDisplay.get() == null)
         {
            newTimeToDisplay.set("Controller time: " + format.format(Conversions.nanosecondsToSeconds(m.getMonotonicTime())));
         }
      });
   }

   @Override
   public void handle(long now)
   {
      if (newTimeToDisplay.get() != null)
      {
         label.setText(newTimeToDisplay.getAndSet(null));
      }
   }
}
