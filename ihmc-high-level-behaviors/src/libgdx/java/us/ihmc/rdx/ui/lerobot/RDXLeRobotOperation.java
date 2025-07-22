package us.ihmc.rdx.ui.lerobot;

import behavior_msgs.msg.dds.LerobotInferenceOperationMessage;
import imgui.ImGui;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTStatusDouble;
import us.ihmc.communication.crdt.CRDTStatusLong;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.rdx.imgui.ImGuiAveragedFrequencyText;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

import static us.ihmc.perception.lerobot.LeRobotInferenceUpdateThread.LEROBOT_UI;

public class RDXLeRobotOperation
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final CommunicationHelper communicationHelper;
   private final TypedNotification<LerobotInferenceOperationMessage> statusSubscription;
   private final Throttler commandThrottler = new Throttler().setFrequency(30.0);
   private final LatestTimestampModifiable latestTimestampModifiable;
   private final CRDTStatusLong sequenceID;
   private final CRDTBidirectionalBoolean running;
   private final CRDTBidirectionalBoolean controlRobot;
   private final CRDTStatusDouble pythonStatusFrequency;
   private final CRDTStatusLong receivedActions;
   private final ImGuiAveragedFrequencyText commsFrequencyText = new ImGuiAveragedFrequencyText();

   public RDXLeRobotOperation(CommunicationHelper communicationHelper, ROS2PeerClockOffsetEstimator peerClockEstimator)
   {
      this.communicationHelper = communicationHelper;

      statusSubscription = communicationHelper.subscribeViaTypedNotification(LEROBOT_UI.getTopic(ROS2ActorDesignation.OPERATOR.getIncomingQualifier()));

      latestTimestampModifiable = new LatestTimestampModifiable(new CRDTInfo(ROS2ActorDesignation.OPERATOR, peerClockEstimator));
      sequenceID = new CRDTStatusLong(ROS2ActorDesignation.ROBOT, latestTimestampModifiable.getCRDTInfo(), 0L);
      running = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);
      controlRobot = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);
      pythonStatusFrequency = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, latestTimestampModifiable.getCRDTInfo(), 0.0);
      receivedActions = new CRDTStatusLong(ROS2ActorDesignation.ROBOT, latestTimestampModifiable.getCRDTInfo(), 0L);
      // TODO Implement rest of sync data comms
   }

   public void renderImGuiWidgets()
   {
      if (statusSubscription.poll())
      {
         commsFrequencyText.ping();
         LerobotInferenceOperationMessage status = statusSubscription.read();
         latestTimestampModifiable.fromMessage(status.getLatestTimestampModifiable());
         running.fromMessage(status.getRunning());
      }

      ImGui.text("LeRobot: Comms: %s   Thread: %3d Hz   Actions: %d".formatted(commsFrequencyText.getText(),
                                                                           (int) pythonStatusFrequency.getValue(),
                                                                           receivedActions.getValue()));
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
         communicationHelper.publish(LEROBOT_UI.getTopic(ROS2ActorDesignation.OPERATOR.getOutgoingQualifier()), command);
      }
   }
}
