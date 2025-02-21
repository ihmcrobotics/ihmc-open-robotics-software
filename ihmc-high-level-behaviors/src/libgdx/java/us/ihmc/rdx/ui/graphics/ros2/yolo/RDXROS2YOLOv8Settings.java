package us.ihmc.rdx.ui.graphics.ros2.yolo;

import imgui.ImGui;
import imgui.ImGuiStyle;
import imgui.type.ImBoolean;
import perception_msgs.msg.dds.YOLOv8AvailableModels;
import perception_msgs.msg.dds.YOLOv8ExecutorSettings;
import perception_msgs.msg.dds.YOLOv8ModelInfo;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Subscription;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class RDXROS2YOLOv8Settings
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final List<RDXROS2YOLOv8ModelSettings> modelSettings = new ArrayList<>();
   private final List<ImBoolean> modelEnables = new ArrayList<>();

   private final YOLOv8ExecutorSettings settingsMessage = new YOLOv8ExecutorSettings();
   private final ROS2Publisher<YOLOv8ExecutorSettings> settingsPublisher;
   private final Notification parametersChanged = new Notification();

   private final AtomicBoolean initialized = new AtomicBoolean(false);
   private boolean enableModelsOnInitialize = false;

   public RDXROS2YOLOv8Settings(ROS2Node ros2Node)
   {
      settingsPublisher = ros2Node.createPublisher(PerceptionAPI.YOLO_SETTINGS);

      // Subscribe to available models message
      ROS2Subscription<YOLOv8AvailableModels> availableModelsSubscription = ros2Node.createSubscription2(PerceptionAPI.YOLO_AVAILABLE_MODELS, this::initialize);

      // Start requesting available models
      ROS2Publisher<YOLOv8AvailableModels> availableModelsRequestPublisher = ros2Node.createPublisher(PerceptionAPI.YOLO_AVAILABLE_MODELS);
      Throttler availableModelsRequestThrottler = new Throttler().setPeriod(1.0);
      YOLOv8AvailableModels requestMessage = new YOLOv8AvailableModels();
      requestMessage.setRequest(true);
      ThreadTools.startAsDaemon(() ->
      {
         while (!initialized.get())
         {
            availableModelsRequestThrottler.waitAndRun();
            availableModelsRequestPublisher.publish(requestMessage);
         }

         availableModelsRequestPublisher.remove();
         availableModelsSubscription.remove();
      }, "AvailableYOLOModelRequester");
   }

   private void initialize(YOLOv8AvailableModels availableModelsMessage)
   {
      if (availableModelsMessage.getRequest() || initialized.getAndSet(true))
         return;

      for (YOLOv8ModelInfo modelInfo : availableModelsMessage.getAvailableYoloModels())
      {
         modelSettings.add(new RDXROS2YOLOv8ModelSettings(modelInfo.getModelNameAsString(), modelInfo.getDetectableObjectClasses().toStringArray()));
         modelEnables.add(new ImBoolean(enableModelsOnInitialize));
      }

      parametersChanged.set();
   }

   public void renderSettings()
   {
      ImGui.beginDisabled(!initialized.get());

      ImGuiStyle style = new ImGuiStyle();
      float indent = ImGui.getFrameHeight() + style.getItemInnerSpacingX() + 1.0f;

      // Render each model's settings
      for (int i = 0; i < modelSettings.size(); ++i)
      {
         RDXROS2YOLOv8ModelSettings settings = modelSettings.get(i);
         ImBoolean enable = modelEnables.get(i);

         // Render checkbox for enabling/disabling the model
         if (ImGui.checkbox(labels.getHidden("enable" + i), enable))
            parametersChanged.set();

         ImGuiTools.previousWidgetTooltip("Enable/Disable");
         ImGui.sameLine();

         // Render the model's settings
         if (ImGui.collapsingHeader(settings.getModelName()))
         {
            ImGui.indent(indent);
            if (settings.renderSettings())
               parametersChanged.set();
            ImGui.unindent(indent);
         }
      }

      ImGui.endDisabled();
   }

   public void publishSettingsMessageIfChanged()
   {
      // If the user adjusted parameters, publish the settings message
      if (parametersChanged.poll())
      {
         settingsMessage.getModelsToRun().clear();
         settingsMessage.getModelSettings().clear();
         for (int i = 0; i < modelSettings.size(); ++i)
         {
            if (modelEnables.get(i).get())
               settingsMessage.getModelsToRun().add(modelSettings.get(i).getModelName());

            settingsMessage.getModelSettings().add().set(modelSettings.get(i).getSettingsMessage());
         }

         settingsPublisher.publish(settingsMessage);
      }
   }

   public boolean anyModelEnabled()
   {
      return modelEnables.stream().anyMatch(ImBoolean::get);
   }

   public void enableAllModels()
   {
      enableModelsOnInitialize = true;
      for (ImBoolean enabled : modelEnables)
         enabled.set(true);

      parametersChanged.set();
   }

   public void disableAllModels()
   {
      enableModelsOnInitialize = false;
      for (ImBoolean enabled : modelEnables)
         enabled.set(false);

      parametersChanged.set();
   }

   public void destroy()
   {
      settingsPublisher.remove();
   }
}