package us.ihmc.rdx.ui.graphics.ros2.yolo;

import imgui.ImGui;
import imgui.ImGuiStyle;
import perception_msgs.msg.dds.YOLOv8ModelInfo;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.detections.yolo.SyncedYOLOv8ExecutorParameters;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.ros2.ROS2Node;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class RDXROS2YOLOv8Settings
{
   private final SyncedYOLOv8ExecutorParameters parameters;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final List<RDXROS2YOLOv8ModelSettings> rdxModelSettings = new ArrayList<>();

   public RDXROS2YOLOv8Settings(ROS2Node ros2Node, ROS2PeerClockOffsetEstimator ros2ClockOffsetEstimator)
   {
      CRDTInfo crdtInfo = new CRDTInfo(ROS2ActorDesignation.OPERATOR, ros2ClockOffsetEstimator);
      parameters = new SyncedYOLOv8ExecutorParameters(ros2Node, crdtInfo);
   }

   public void update()
   {
      parameters.checkModifiedAndUpdate();

      if (rdxModelSettings.size() != parameters.getModelParameters().size())
      {
         rdxModelSettings.clear();
         parameters.getModelParameters().values().forEach(modelParameters -> rdxModelSettings.add(new RDXROS2YOLOv8ModelSettings(modelParameters)));
      }

      rdxModelSettings.forEach(modelSettings -> modelSettings.update(parameters));
   }


   public void renderSettings()
   {
      ImGui.beginDisabled(rdxModelSettings.isEmpty());

      ImGuiStyle style = new ImGuiStyle();
      float indent = ImGui.getFrameHeight() + style.getItemInnerSpacingX() + 1.0f;

      // Render each model's settings
      for (int i = 0; i < rdxModelSettings.size(); ++i)
      {
         RDXROS2YOLOv8ModelSettings settings = rdxModelSettings.get(i);
         String modelName = settings.getModelName();
         boolean enabled = parameters.getModelsToRun().getValue().contains(modelName);

         // Render checkbox for enabling/disabling the model
         if (ImGui.checkbox(labels.getHidden("enable" + i), enabled))
         {
            if (enabled)
               parameters.getModelsToRun().remove(modelName);
            else
               parameters.getModelsToRun().add(modelName);
         }

         ImGuiTools.previousWidgetTooltip("Enable/Disable");
         ImGui.sameLine();

         // Render the model's settings
         if (ImGui.collapsingHeader(modelName))
         {
            ImGui.indent(indent);
            settings.renderSettings();
            ImGui.unindent(indent);
         }
      }

      ImGui.endDisabled();
   }


   public boolean anyModelEnabled()
   {
      return !parameters.getModelsToRun().getValue().isEmpty();
   }

   public void enableAllModels()
   {
      parameters.getModelsToRun()
                .addAll(parameters.getAvailableModels().getReadOnly().stream().map(YOLOv8ModelInfo::getModelNameAsString).collect(Collectors.toSet()));
   }

   public void disableAllModels()
   {
      parameters.getModelsToRun().getValueAndModify().clear();
   }

   public void destroy()
   {
      parameters.close();
   }
}