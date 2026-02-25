package us.ihmc.rdx.ui.graphics.ros2.yolo;

import imgui.ImGui;
import imgui.type.ImInt;
import perception_msgs.msg.dds.YOLOv8ExecutorParameters;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.detections.yolo.SyncedYOLOv8ExecutorParameters;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

public class RDXROS2YOLOv8Settings
{
   private final SyncedYOLOv8ExecutorParameters parameters;

   private String[] availableModels = new String[]{"None"};
   private final ImInt selectedModel = new ImInt();

   private final List<RDXROS2YOLOv8ModelSettings> rdxModelSettings = new ArrayList<>();

   private final YOLOv8ExecutorParameters parametersMessage = new YOLOv8ExecutorParameters();
   private final ROS2Publisher<YOLOv8ExecutorParameters> parametersPublisher;

   public RDXROS2YOLOv8Settings(ROS2Node ros2Node, ROS2PeerClockOffsetEstimator ros2ClockOffsetEstimator)
   {
      CRDTInfo crdtInfo = new CRDTInfo(ROS2ActorDesignation.OPERATOR, ros2ClockOffsetEstimator);
      parameters = new SyncedYOLOv8ExecutorParameters(crdtInfo);
      parameters.requestSendFullData();

      parametersPublisher = ros2Node.createPublisher(PerceptionAPI.YOLO_PARAMETERS);
      ros2Node.createSubscription2(PerceptionAPI.YOLO_PARAMETERS, message ->
      {
         parameters.fromMessage(message);
         parameters.confirmReceivedFullData();
      });

      parameters.toMessage(parametersMessage);
      parametersPublisher.publish(parametersMessage);
   }

   public void update()
   {
      parameters.checkModified();
      if (rdxModelSettings.size() != parameters.getAvailableModels().getSize())
      {
         rdxModelSettings.clear();
         availableModels = new String[parameters.getAvailableModels().getSize() + 1];
         availableModels[0] = "None";

         AtomicInteger counter = new AtomicInteger(1);
         parameters.getAvailableModels().getReadOnly().forEach(modelInfo ->
         {
            availableModels[counter.getAndIncrement()] = modelInfo.getModelNameAsString();
            rdxModelSettings.add(new RDXROS2YOLOv8ModelSettings(modelInfo, parameters.getModelParameters()));
         });
      }

      if (parameters.isModified())
      {
         int index = 0;
         for (int i = 0; i < availableModels.length; ++i)
         {
            if (availableModels[i].equals(parameters.getModelToRun().getValue()))
            {
               index = i;
               break;
            }
         }
         selectedModel.set(index);
      }

      if (selectedModel.get() != 0)
         rdxModelSettings.get(selectedModel.get() - 1).update();

      if (parameters.pollNeedSendFullData() || parameters.getModelParameters().pollNeedSendFullData())
      {
         parameters.toMessage(parametersMessage);
         parametersPublisher.publish(parametersMessage);
      }
   }

   public void renderSettings()
   {
      if (ImGui.combo("Model to run", selectedModel, availableModels))
         parameters.getModelToRun().setValue(selectedModel.get() == 0 ? null : availableModels[selectedModel.get()]);

      if (selectedModel.get() != 0)
      {
         // Render the selected model's settings
         RDXROS2YOLOv8ModelSettings settings = rdxModelSettings.get(selectedModel.get() - 1);
         settings.renderSettings();
      }
   }

   public void destroy()
   {
      parametersPublisher.remove();
   }
}