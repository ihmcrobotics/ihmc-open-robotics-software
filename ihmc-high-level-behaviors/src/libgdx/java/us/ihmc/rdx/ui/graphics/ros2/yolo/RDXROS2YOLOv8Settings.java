package us.ihmc.rdx.ui.graphics.ros2.yolo;

import imgui.ImGui;
import imgui.ImGuiStyle;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import perception_msgs.msg.dds.YOLOv8AvailableModels;
import perception_msgs.msg.dds.YOLOv8ExecutorSettings;
import perception_msgs.msg.dds.YOLOv8ModelInfo;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.graphics.RDXVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Subscription;

import java.util.ArrayList;
import java.util.List;

/*
 *  FIXME: It doesn't make sense to have a visualizer for settings.
 *  This is only meant to be a short-term solution
 *  We should really make a better system to house settings for detections.
 */
public class RDXROS2YOLOv8Settings extends RDXVisualizer
{
   private static final String[] AVAILABLE_SENSORS = {"ZED", "D455"};

   private final ROS2Subscription<YOLOv8AvailableModels> availableModelsSubscriber;
   private final ROS2Publisher<YOLOv8ExecutorSettings> settingsPublisher;
   private final YOLOv8ExecutorSettings settingsMessage = new YOLOv8ExecutorSettings();

   private final ROS2Heartbeat demandYOLOv8ZED;
   private final ROS2Heartbeat demandYOLOv8D455;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final ImInt selectedSensor = new ImInt(0); // 0 = ZED, 1 = Realsense
   private final List<RDXROS2YOLOv8ModelSettings> modelSettings = new ArrayList<>();
   private final List<ImBoolean> modelEnables = new ArrayList<>();

   private final Notification parametersChanged = new Notification();

   private boolean initialized = false;

   public RDXROS2YOLOv8Settings(String title, ROS2Node ros2Node)
   {
      super(title);

      settingsPublisher = ros2Node.createPublisher(PerceptionAPI.YOLO_SETTINGS);

      demandYOLOv8ZED = new ROS2Heartbeat(ros2Node, PerceptionAPI.REQUEST_YOLO_ZED);
      demandYOLOv8D455 = new ROS2Heartbeat(ros2Node, PerceptionAPI.REQUEST_YOLO_REALSENSE);

      // Subscribe to available models message
      availableModelsSubscriber = ros2Node.createSubscription2(PerceptionAPI.YOLO_AVAILABLE_MODELS, this::initialize);

      // Start requesting available models
      ROS2Publisher<YOLOv8AvailableModels> availableModelsRequestPublisher = ros2Node.createPublisher(PerceptionAPI.YOLO_AVAILABLE_MODELS);
      Throttler availableModelsRequestThrottler = new Throttler().setPeriod(1.0);
      YOLOv8AvailableModels requestMessage = new YOLOv8AvailableModels();
      requestMessage.setRequest(true);
      ThreadTools.startAsDaemon(() ->
      {
         while (!initialized)
         {
            availableModelsRequestThrottler.waitAndRun();
            availableModelsRequestPublisher.publish(requestMessage);
         }
         availableModelsRequestPublisher.remove();
      }, "AvailableYOLOModelRequester");
   }

   private void initialize(YOLOv8AvailableModels availableModelsMessage)
   {
      if (availableModelsMessage.getRequest())
         return;

      for (YOLOv8ModelInfo modelInfo : availableModelsMessage.getAvailableYoloModels())
      {
         modelSettings.add(new RDXROS2YOLOv8ModelSettings(modelInfo.getModelNameAsString(), modelInfo.getDetectableObjectClasses().toStringArray(), labels));
         modelEnables.add(new ImBoolean(false));
      }

      availableModelsSubscriber.remove();
      initialized = true;
   }

   @Override
   public void updateHeartbeat()
   {
      demandYOLOv8ZED.setAlive(isActive() && selectedSensor.get() == 0);
      demandYOLOv8D455.setAlive(isActive() && selectedSensor.get() == 1);
   }

   @Override
   public void renderImGuiWidgets()
   {
      ImGui.beginDisabled(initialized);

      ImGuiStyle style = new ImGuiStyle();
      float indent = ImGui.getFrameHeight() + style.getItemInnerSpacingX() + 1.0f;
      ImGui.indent(indent);

      ImGui.combo(labels.get("Sensor Selection"), selectedSensor, AVAILABLE_SENSORS);

      // Render each model's settings
      for (int i = 0; i < modelSettings.size(); ++i)
      {
         RDXROS2YOLOv8ModelSettings settings = modelSettings.get(i);
         ImBoolean enable = modelEnables.get(i);

         // Render checkbox for enabling/disabling the model
         if (ImGui.checkbox(labels.getHidden("enable" + settings.getModelName()), enable))
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

      ImGui.unindent(indent);
      ImGui.endDisabled();
   }

   @Override
   public void update()
   {
      super.update();

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

   @Override
   public void destroy()
   {
      super.destroy();

      demandYOLOv8ZED.destroy();
      demandYOLOv8D455.destroy();

      settingsPublisher.remove();
   }
}