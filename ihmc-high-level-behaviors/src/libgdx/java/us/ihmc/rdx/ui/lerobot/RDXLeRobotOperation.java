package us.ihmc.rdx.ui.lerobot;

import behavior_msgs.msg.dds.LerobotInferenceOperationMessage;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.lerobot.LeRobotInferenceUpdateThread;
import us.ihmc.rdx.imgui.ImGuiAveragedFrequencyText;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;

import static us.ihmc.lerobot.LeRobotInferenceUpdateThread.OPERATOR_UI;

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
   private String statusMessage = "Not yet connected to robot";
   private final TypedNotification<LerobotInferenceOperationMessage> statusSubscription;
   private final ROS2Publisher<LerobotInferenceOperationMessage> commandPublisher;
   private final ImGuiAveragedFrequencyText commsFrequencyText = new ImGuiAveragedFrequencyText();
   private final SideDependentList<RDXReferenceFrameGraphic> actionHandPoseGraphics = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> actionForearmPoseGraphics = new SideDependentList<>();

   public RDXLeRobotOperation(ROS2Node ros2Node, ROS2PeerClockOffsetEstimator peerClockEstimator)
   {
      latestTimestampModifiable = new LatestTimestampModifiable(new CRDTInfo(ROS2ActorDesignation.OPERATOR, peerClockEstimator));
      running = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);
      controlRobot = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);

      statusSubscription = ROS2Tools.createNotificationSubscription(ros2Node, OPERATOR_UI.getTopic(ROS2ActorDesignation.OPERATOR.getIncomingQualifier()));
      commandPublisher = ros2Node.createPublisher(OPERATOR_UI.getTopic(ROS2ActorDesignation.OPERATOR.getOutgoingQualifier()));
   }

   public void create(RDXBaseUI baseUI)
   {
      for (RobotSide side : RobotSide.values)
      {
         actionHandPoseGraphics.put(side, new RDXReferenceFrameGraphic(0.15, Color.ORANGE));
         actionForearmPoseGraphics.put(side, new RDXReferenceFrameGraphic(0.15, Color.OLIVE));
      }

      baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);
   }

   public void update()
   {

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
         for (RobotSide side : RobotSide.values)
         {
            actionHandPoseGraphics.get(side).setPoseInWorldFrame(status.getActionHandPoses()[side.ordinal()]);
            actionForearmPoseGraphics.get(side).setPoseInWorldFrame(status.getActionForearmPoses()[side.ordinal()]);
         }
         pythonStatusFrequency = status.getPythonStatusFrequency();
         statusMessage = status.getPythonStatusMessageAsString();
         receivedActions = status.getReceivedActions();
      }

      ImGui.text("LeRobot: Thread: %s   Python: %3d Hz   Actions: %d".formatted(commsFrequencyText.getText(), (int) pythonStatusFrequency, receivedActions));
      ImGui.text("Python status: " + statusMessage);
      if (ImGui.checkbox(labels.get("Run inference"), running.getValue()))
         running.setValue(!running.getValue());
      ImGui.beginDisabled(running.getValue());
      if (ImGui.checkbox(labels.get("Control robot"), controlRobot.getValue()))
         controlRobot.setValue(!controlRobot.getValue());
      ImGuiTools.previousWidgetTooltip("Whether to control the robot when you run inference.");
      ImGui.endDisabled();
      ImGui.sameLine();
      ImGui.textColored(ImGuiTools.DARK_RED, "Press SPACE to stop.");

      if (ImGui.isKeyPressed(ImGuiTools.getSpaceKey()))
      {
         controlRobot.setValue(false);
         running.setValue(false);
      }

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

   private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (running.getValue())
         for (RobotSide side : RobotSide.values)
         {
            actionHandPoseGraphics.get(side).getRenderables(renderables, pool);
            actionForearmPoseGraphics.get(side).getRenderables(renderables, pool);
         }
   }
}
