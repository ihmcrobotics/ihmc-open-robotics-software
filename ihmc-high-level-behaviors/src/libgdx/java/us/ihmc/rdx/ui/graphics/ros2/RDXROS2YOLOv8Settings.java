package us.ihmc.rdx.ui.graphics.ros2;

import imgui.ImGui;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import perception_msgs.msg.dds.YOLOv8ParametersMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.rdx.ui.graphics.RDXVisualizer;

public class RDXROS2YOLOv8Settings extends RDXVisualizer
{
   private static final String[] AVAILABLE_SENSORS = {"ZED", "D455"};

   private final ROS2PublishSubscribeAPI ros2;

   private final ImFloat confidenceThreshold = new ImFloat(0.3f);
   private final ImFloat nmsThreshold = new ImFloat(0.1f);
   private final ImFloat maskThreshold = new ImFloat(0.0f);
   private final ImFloat candidateAcceptanceThreshold = new ImFloat(0.6f);
   private final ImInt selectedSensor = new ImInt(0); // 0 = ZED, 1 = Realsense

   private final ROS2Heartbeat demandYOLOv8ICPZed;
   private final ROS2Heartbeat demandYOLOv8ICPRealsense;

   public RDXROS2YOLOv8Settings(String title, ROS2PublishSubscribeAPI ros2)
   {
      super(title);

      this.ros2 = ros2;

      demandYOLOv8ICPZed = new ROS2Heartbeat(ros2, PerceptionAPI.REQUEST_YOLO_ZED);
      demandYOLOv8ICPRealsense = new ROS2Heartbeat(ros2, PerceptionAPI.REQUEST_YOLO_REALSENSE);
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();

      ImGui.combo("Sensor Selection", selectedSensor, AVAILABLE_SENSORS);

      demandYOLOv8ICPZed.setAlive(isActive() && selectedSensor.get() == 0);
      demandYOLOv8ICPRealsense.setAlive(isActive() && selectedSensor.get() == 1);

      boolean parameterChanged = false;
      if (ImGui.sliderFloat("confidenceThreshold", confidenceThreshold.getData(), 0.0f, 1.0f))
         parameterChanged = true;
      if (ImGui.sliderFloat("nmsThreshold", nmsThreshold.getData(), 0.0f, 1.0f))
         parameterChanged = true;
      if (ImGui.sliderFloat("maskThreshold", maskThreshold.getData(), -1.0f, 1.0f))
         parameterChanged = true;
      if (ImGui.sliderFloat("candidateAcceptanceThreshold", candidateAcceptanceThreshold.getData(), 0.0f, 1.0f))
         parameterChanged = true;

      if (parameterChanged)
      {
         YOLOv8ParametersMessage message = new YOLOv8ParametersMessage();
         message.setConfidenceThreshold(confidenceThreshold.get());
         message.setNonMaximumSuppressionThreshold(nmsThreshold.get());
         message.setSegmentationThreshold(maskThreshold.get());
         message.setCandidateAcceptanceThreshold(candidateAcceptanceThreshold.get());
         ros2.publish(PerceptionAPI.YOLO_PARAMETERS, message);
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