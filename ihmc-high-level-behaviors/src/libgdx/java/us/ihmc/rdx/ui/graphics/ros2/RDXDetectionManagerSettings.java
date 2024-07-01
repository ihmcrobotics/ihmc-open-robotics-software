package us.ihmc.rdx.ui.graphics.ros2;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.detections.DetectionManagerSettings;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySet;
import us.ihmc.rdx.ui.graphics.RDXVisualizer;

/*
 *  FIXME: It doesn't make sense to have a visualizer for settings.
 *  This is only meant to be a short-term solution
 *  We should really make a better system to house settings for detections.
 */
public class RDXDetectionManagerSettings extends RDXVisualizer
{

   private final DetectionManagerSettings settings = new DetectionManagerSettings();
   private final ImGuiRemoteROS2StoredPropertySet remoteStoredPropertySet;

   public RDXDetectionManagerSettings(String title, ROS2PublishSubscribeAPI ros2)
   {
      super(title);

      remoteStoredPropertySet = new ImGuiRemoteROS2StoredPropertySet(ros2, settings, PerceptionAPI.DETECTION_MANAGER_SETTINGS);
   }

   @Override
   public void renderImGuiWidgets()
   {
      setActive(true); // Detection Manager is always active. That's part of the reason why this visualizer doesn't make sense.

      remoteStoredPropertySet.renderImGuiWidgetsWithUpdateButton();
   }
}
