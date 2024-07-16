package us.ihmc.rdx.ui.graphics.ros2;

import imgui.ImGui;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import perception_msgs.msg.dds.YOLOv8ParametersMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.detections.yolo.YOLOv8Model;
import us.ihmc.perception.detections.yolo.YOLOv8Tools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.ui.graphics.RDXVisualizer;
import us.ihmc.tools.thread.Throttler;

import java.util.HashSet;
import java.util.Set;
import java.util.TreeSet;

/*
 *  FIXME: It doesn't make sense to have a visualizer for settings.
 *  This is only meant to be a short-term solution
 *  We should really make a better system to house settings for detections.
 */
public class RDXYOLOv8Settings extends RDXVisualizer
{
   private static final String[] AVAILABLE_SENSORS = {"ZED", "D455"};
   private static final double MESSAGE_PUBLISH_PERIOD = 2; // publish messages every 2 seconds

   private final ROS2PublishSubscribeAPI ros2;
   private final YOLOv8ParametersMessage parametersMessage = new YOLOv8ParametersMessage();

   private final ImFloat confidenceThreshold = new ImFloat(0.8f);
   private final ImFloat nmsThreshold = new ImFloat(0.1f);
   private final ImFloat maskThreshold = new ImFloat(0.0f);
   private final ImFloat outlierRejectionThreshold = new ImFloat(1.0f);
   private final ImInt selectedSensor = new ImInt(0); // 0 = ZED, 1 = Realsense

   private final Set<YOLOv8Model> availableModels = new TreeSet<>((modelA, modelB) -> modelA.getModelName().compareToIgnoreCase(modelB.getModelName()));
   private final Set<YOLOv8Model> modelsToLoad = new HashSet<>();

   private final ROS2Heartbeat demandYOLOv8ICPZed;
   private final ROS2Heartbeat demandYOLOv8ICPRealsense;

   private final Throttler messagePublishThrottler = new Throttler().setPeriod(MESSAGE_PUBLISH_PERIOD);
   private final Notification parametersChanged = new Notification();

   public RDXYOLOv8Settings(String title, ROS2PublishSubscribeAPI ros2)
   {
      super(title);

      this.ros2 = ros2;

      demandYOLOv8ICPZed = new ROS2Heartbeat(ros2, PerceptionAPI.REQUEST_YOLO_ZED);
      demandYOLOv8ICPRealsense = new ROS2Heartbeat(ros2, PerceptionAPI.REQUEST_YOLO_REALSENSE);

      availableModels.addAll(YOLOv8Tools.getAvailableYOLOModels());
      modelsToLoad.addAll(availableModels);
   }

   @Override
   public void updateHeartbeat()
   {
      demandYOLOv8ICPZed.setAlive(isActive() && selectedSensor.get() == 0);
      demandYOLOv8ICPRealsense.setAlive(isActive() && selectedSensor.get() == 1);
   }

   @Override
   public void renderImGuiWidgets()
   {
      ImGui.combo("Sensor Selection", selectedSensor, AVAILABLE_SENSORS);

      if (ImGui.sliderFloat("confidenceThreshold", confidenceThreshold.getData(), 0.0f, 1.0f))
         parametersChanged.set();
      if (ImGui.sliderFloat("nmsThreshold", nmsThreshold.getData(), 0.0f, 1.0f))
         parametersChanged.set();
      if (ImGui.sliderFloat("maskThreshold", maskThreshold.getData(), -1.0f, 1.0f))
         parametersChanged.set();
      if (ImGui.sliderFloat("outlierThreshold", outlierRejectionThreshold.getData(), 0.0f, 10.0f))
         parametersChanged.set();

      if (ImGui.collapsingHeader("Models to Load"))
      {
         if (ImGui.button("Select All"))
         {
            modelsToLoad.addAll(availableModels);
            parametersChanged.set();
         }
         ImGui.sameLine();
         if (ImGui.button("Unselect All"))
         {
            modelsToLoad.clear();
            parametersChanged.set();
         }

         for (YOLOv8Model yoloModel : availableModels)
         {
            if (ImGuiTools.smallCheckbox(yoloModel.getModelName(), modelsToLoad.contains(yoloModel)))
            {
               if (modelsToLoad.contains(yoloModel))
                  modelsToLoad.remove(yoloModel);
               else
                  modelsToLoad.add(yoloModel);

               parametersChanged.set();
            }
         }
      }
   }

   @Override
   public void update()
   {
      super.update();

      // Publish a settings message if user changed a setting or enough time has passed since last publication
      if (parametersChanged.poll() || messagePublishThrottler.run())
      {
         parametersMessage.setConfidenceThreshold(confidenceThreshold.get());
         parametersMessage.setNonMaximumSuppressionThreshold(nmsThreshold.get());
         parametersMessage.setSegmentationThreshold(maskThreshold.get());
         parametersMessage.setOutlierThreshold(outlierRejectionThreshold.get());

         parametersMessage.getModelsToLoad().clear();
         for (YOLOv8Model yoloModel : modelsToLoad)
            parametersMessage.getModelsToLoad().add(yoloModel.getModelName());

         ros2.publish(PerceptionAPI.YOLO_PARAMETERS, parametersMessage);
      }
   }

   @Override
   public void destroy()
   {
      super.destroy();

      demandYOLOv8ICPZed.destroy();
      demandYOLOv8ICPRealsense.destroy();
   }
}