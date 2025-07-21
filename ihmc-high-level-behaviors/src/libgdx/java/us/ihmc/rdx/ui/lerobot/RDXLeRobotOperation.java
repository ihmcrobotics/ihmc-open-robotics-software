package us.ihmc.rdx.ui.lerobot;

import behavior_msgs.msg.dds.LerobotInferenceOperationMessage;
import imgui.ImGui;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.lerobot.LeRobotInferenceUpdateThread;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

import static us.ihmc.perception.lerobot.LeRobotInferenceUpdateThread.LEROBOT_UI;

public class RDXLeRobotOperation
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final CommunicationHelper communicationHelper;
   private final TypedNotification<LerobotInferenceOperationMessage> statusSubscription;
   private final Throttler commandThrottler = new Throttler().setFrequency(LeRobotInferenceUpdateThread.HZ);
   private final LatestTimestampModifiable latestTimestampModifiable;
   private final CRDTBidirectionalBoolean running;

   public RDXLeRobotOperation(CommunicationHelper communicationHelper, ROS2PeerClockOffsetEstimator peerClockEstimator)
   {
      this.communicationHelper = communicationHelper;

      statusSubscription = communicationHelper.subscribeViaTypedNotification(LEROBOT_UI.getTopic(ROS2ActorDesignation.OPERATOR.getIncomingQualifier()));

      latestTimestampModifiable = new LatestTimestampModifiable(new CRDTInfo(ROS2ActorDesignation.OPERATOR, peerClockEstimator));
      running = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);
   }

   public void renderImGuiWidgets()
   {
      if (statusSubscription.poll())
      {
         LerobotInferenceOperationMessage status = statusSubscription.read();
         latestTimestampModifiable.fromMessage(status.getLatestTimestampModifiable());
         running.fromMessage(status.getRunning());
      }

      if (ImGui.checkbox(labels.get("Run Lerobot inference"), running.getValue()))
      {
         running.setValue(!running.getValue());
      }

      if (commandThrottler.run())
      {
         LerobotInferenceOperationMessage command = new LerobotInferenceOperationMessage();
         latestTimestampModifiable.toMessage(command.getLatestTimestampModifiable());
         command.setRunning(running.toMessage());
         communicationHelper.publish(LEROBOT_UI.getTopic(ROS2ActorDesignation.OPERATOR.getOutgoingQualifier()), command);
      }
   }
}
