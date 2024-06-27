package us.ihmc.rdx.ui.graphics.ros2;

import imgui.ImGui;
import imgui.type.ImFloat;
import perception_msgs.msg.dds.DetectionManagerSettingsMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.rdx.ui.graphics.RDXVisualizer;
import us.ihmc.tools.thread.Throttler;

/*
 *  FIXME: It doesn't make sense to have a visualizer for settings.
 *  This is only meant to be a short-term solution
 *  We should really make a better system to house settings for detections.
 */
public class RDXDetectionManagerSettings extends RDXVisualizer
{
   private static final double MESSAGE_PUBLISH_PERIOD = 2; // publish messages every 2 seconds

   private final ROS2PublishSubscribeAPI ros2;

   private final ImFloat matchDistanceThreshold = new ImFloat(1.0f);
   private final ImFloat acceptanceConfidence = new ImFloat(0.6f);
   private final ImFloat stabilityConfidence = new ImFloat(0.4f);
   private final ImFloat stabilityFrequency = new ImFloat(10.0f);
   private final ImFloat historyDuration = new ImFloat(1.0f);

   private final Throttler messagePublishThrottler = new Throttler().setPeriod(MESSAGE_PUBLISH_PERIOD);
   private final Notification parametersChanged = new Notification();
   private final DetectionManagerSettingsMessage settingsMessage = new DetectionManagerSettingsMessage();

   public RDXDetectionManagerSettings(String title, ROS2PublishSubscribeAPI ros2)
   {
      super(title);

      this.ros2 = ros2;
   }

   @Override
   public void renderImGuiWidgets()
   {
      setActive(true); // Detection Manager is always active. That's part of the reason why this visualizer doesn't make sense.

      if (ImGui.sliderFloat("Match Distance", matchDistanceThreshold.getData(), 0.0f, 10.0f))
         parametersChanged.set();
      if (ImGui.sliderFloat("Acceptance Confidence", acceptanceConfidence.getData(), 0.0f, 1.0f))
         parametersChanged.set();
      if (ImGui.sliderFloat("Stability Confidence", stabilityConfidence.getData(), 0.0f, 1.0f))
         parametersChanged.set();
      if (ImGui.sliderFloat("Stability Frequency", stabilityFrequency.getData(), 0.0f, 30.0f))
         parametersChanged.set();
      if (ImGui.sliderFloat("History Duration", historyDuration.getData(), 0.0f, 30.0f))
         parametersChanged.set();
   }

   @Override
   public void update()
   {
      super.update();

      if (parametersChanged.poll() || messagePublishThrottler.run())
      {
         settingsMessage.setMatchDistanceThreshold(matchDistanceThreshold.get());
         settingsMessage.setAcceptanceAverageConfidence(acceptanceConfidence.get());
         settingsMessage.setStabilityAverageConfidence(stabilityConfidence.get());
         settingsMessage.setStabilityFrequency(stabilityFrequency.get());
         settingsMessage.setDetectionHistoryDuration(historyDuration.get());

         ros2.publish(PerceptionAPI.DETECTION_MANAGER_SETTINGS, settingsMessage);
      }
   }
}
