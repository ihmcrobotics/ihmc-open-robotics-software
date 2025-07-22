package us.ihmc.rdx.ui.lerobot;

import behavior_msgs.msg.dds.LerobotInferenceOperationMessage;
import imgui.ImGui;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.lerobot.LeRobotInferenceUpdateThread;
import us.ihmc.rdx.imgui.ImGuiAveragedFrequencyText;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;

import static us.ihmc.perception.lerobot.LeRobotInferenceUpdateThread.LEROBOT_UI;

/**
 * UI for remotely operating {@link LeRobotInferenceUpdateThread}.
 */
public class RDXLeRobotOperation
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final Throttler commandThrottler = new Throttler().setFrequency(30.0);
   private final LatestTimestampModifiable latestTimestampModifiable;
   private final CRDTBidirectionalBoolean running;
   private final CRDTBidirectionalBoolean controlRobot;
   private double pythonStatusFrequency = 0.0;
   private long receivedActions = 0L;
   private final TypedNotification<LerobotInferenceOperationMessage> statusSubscription;
   private final ROS2Publisher<LerobotInferenceOperationMessage> commandPublisher;
   private final ImGuiAveragedFrequencyText commsFrequencyText = new ImGuiAveragedFrequencyText();

   public RDXLeRobotOperation(ROS2Node ros2Node, ROS2PeerClockOffsetEstimator peerClockEstimator)
   {
      latestTimestampModifiable = new LatestTimestampModifiable(new CRDTInfo(ROS2ActorDesignation.OPERATOR, peerClockEstimator));
      running = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);
      controlRobot = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);

      statusSubscription = ROS2Tools.createNotificationSubscription(ros2Node, LEROBOT_UI.getTopic(ROS2ActorDesignation.OPERATOR.getIncomingQualifier()));
      commandPublisher = ros2Node.createPublisher(LEROBOT_UI.getTopic(ROS2ActorDesignation.OPERATOR.getOutgoingQualifier()));
   }

   public void renderImGuiWidgets()
   {
      if (statusSubscription.poll())
      {
         commsFrequencyText.ping();
         LerobotInferenceOperationMessage status = statusSubscription.read();
         latestTimestampModifiable.fromMessage(status.getLatestTimestampModifiable());
         running.fromMessage(status.getRunning());
         controlRobot.fromMessage(status.getControlRobot());
         pythonStatusFrequency = status.getPythonStatusFrequency();
         receivedActions = status.getReceivedActions();
      }

      ImGui.text("LeRobot: Comms: %s   Thread: %3d Hz   Actions: %d".formatted(commsFrequencyText.getText(), (int) pythonStatusFrequency, receivedActions));
      if (ImGui.checkbox(labels.get("Run inference & preview"), running.getValue()))
         running.setValue(!running.getValue());
      if (ImGui.checkbox(labels.get("Control robot"), controlRobot.getValue()))
         controlRobot.setValue(!controlRobot.getValue());

      ImGui.separator();

      if (commandThrottler.run())
      {
         LerobotInferenceOperationMessage command = new LerobotInferenceOperationMessage();
         latestTimestampModifiable.toMessage(command.getLatestTimestampModifiable());
         command.setRunning(running.toMessage());
         command.setControlRobot(controlRobot.toMessage());
         commandPublisher.publish(command);
      }
   }
}
