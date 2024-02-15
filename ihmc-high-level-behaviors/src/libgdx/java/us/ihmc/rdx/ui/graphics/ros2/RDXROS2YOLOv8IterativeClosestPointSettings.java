package us.ihmc.rdx.ui.graphics.ros2;

import imgui.ImGui;
import imgui.type.ImFloat;
import perception_msgs.msg.dds.YOLOv8ParametersMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.ui.graphics.RDXVisualizer;

public class RDXROS2YOLOv8IterativeClosestPointSettings extends RDXVisualizer
{
   private final ROS2PublishSubscribeAPI ros2;

   private final ImFloat confidenceThreshold = new ImFloat(0.3f);
   private final ImFloat nmsThreshold = new ImFloat(0.1f);
   private final ImFloat maskThreshold = new ImFloat(0.0f);
   private final ImFloat candidateAcceptanceThreshold = new ImFloat(0.6f);
   private final ImFloat candidateMovementThreshold = new ImFloat(3000.0f);

   public RDXROS2YOLOv8IterativeClosestPointSettings(String title, ROS2PublishSubscribeAPI ros2)
   {
      super(title);

      this.ros2 = ros2;
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();

      boolean parameterChanged = false;
      if (ImGui.sliderFloat("confidenceThreshold", confidenceThreshold.getData(), 0.0f, 1.0f))
         parameterChanged = true;
      if (ImGui.sliderFloat("nmsThreshold", nmsThreshold.getData(), 0.0f, 1.0f))
         parameterChanged = true;
      if (ImGui.sliderFloat("maskThreshold", maskThreshold.getData(), -1.0f, 1.0f))
         parameterChanged = true;
      if (ImGui.sliderFloat("candidateAcceptanceThreshold", candidateAcceptanceThreshold.getData(), 0.0f, 1.0f))
         parameterChanged = true;
      if (ImGuiTools.volatileInputFloat("candidateMovementThreshold", candidateMovementThreshold, 500.0f))
      {
         candidateMovementThreshold.set((float) MathTools.clamp(candidateMovementThreshold.get(), 0.0f, 50000.0f));
         parameterChanged = true;
      }

      if (parameterChanged)
      {
         YOLOv8ParametersMessage message = new YOLOv8ParametersMessage();
         message.setConfidenceThreshold(confidenceThreshold.get());
         message.setNonMaximumSuppressionThreshold(nmsThreshold.get());
         message.setSegmentationThreshold(maskThreshold.get());
         message.setCandidateAcceptanceThreshold(candidateAcceptanceThreshold.get());
         message.setCandidateMovementThreshold(candidateMovementThreshold.get());
         ros2.publish(PerceptionAPI.YOLO_PARAMETERS, message);
      }
   }
}
