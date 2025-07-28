package us.ihmc.rdx.ui.lerobot;

import behavior_msgs.msg.dds.LerobotInferenceOperationMessage;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import std_msgs.msg.dds.Float32MultiArray;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInitialConfigurationMessage;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory;
import us.ihmc.idl.IDLSequence;
import us.ihmc.perception.lerobot.LeRobotInferenceManager;
import us.ihmc.perception.lerobot.LeRobotInferenceUpdateThread;
import us.ihmc.rdx.imgui.ImGuiAveragedFrequencyText;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
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
   private final TypedNotification<Float32MultiArray> actionHandPosesSubscription;
   private double pythonStatusFrequency = 0.0;
   private long receivedActions = 0L;
   private final TypedNotification<LerobotInferenceOperationMessage> statusSubscription;
   private final ROS2Publisher<LerobotInferenceOperationMessage> commandPublisher;
   private final ImGuiAveragedFrequencyText commsFrequencyText = new ImGuiAveragedFrequencyText();

   private final ImBoolean isKSTEnabled = new ImBoolean(false);
   private final ImBoolean controlArmsOnly = new ImBoolean(false);

   private final ROS2Publisher<KinematicsStreamingToolboxConfigurationMessage> kstConfigPublisher;
   private final ROS2Publisher<KinematicsStreamingToolboxInitialConfigurationMessage> kstInitialConfigPublisher;
   private final ROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;
   private final KinematicsStreamingToolboxParameters kstParameters;
   private final ROS2SyncedRobotModel syncedRobotModel;

   private final SideDependentList<RDXReferenceFrameGraphic> actionHandPoseGraphics = new SideDependentList<>();
   private final Pose3D handPose = new Pose3D();

   public RDXLeRobotOperation(ROS2Node ros2Node, ROS2PeerClockOffsetEstimator peerClockEstimator, ROS2SyncedRobotModel syncedRobot)
   {
      this.syncedRobotModel = syncedRobot;
      this.kstParameters = new KinematicsStreamingToolboxParameters();
      this.kstConfigPublisher = ros2Node.createPublisher(KinematicsStreamingToolboxModule.getInputStreamingConfigurationTopic(syncedRobot.getRobotModel()
                                                                                                                                              .getSimpleRobotName()));
      this.kstInitialConfigPublisher = ros2Node.createPublisher(KinematicsStreamingToolboxModule.getInputStreamingInitialConfigurationTopic(syncedRobot.getRobotModel()
                                                                                                                                                            .getSimpleRobotName()));
      this.toolboxStatePublisher = ros2Node.createPublisher(KinematicsStreamingToolboxModule.getInputStateTopic(syncedRobot.getRobotModel()
                                                                                                                                .getSimpleRobotName()));
      latestTimestampModifiable = new LatestTimestampModifiable(new CRDTInfo(ROS2ActorDesignation.OPERATOR, peerClockEstimator));
      running = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);
      controlRobot = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);

      statusSubscription = ROS2Tools.createNotificationSubscription(ros2Node, LEROBOT_UI.getTopic(ROS2ActorDesignation.OPERATOR.getIncomingQualifier()));
      commandPublisher = ros2Node.createPublisher(LEROBOT_UI.getTopic(ROS2ActorDesignation.OPERATOR.getOutgoingQualifier()));

      actionHandPosesSubscription = ROS2Tools.createNotificationSubscription(ros2Node, LeRobotInferenceManager.ACTION_HAND_POSES);
   }

   public void create(RDXBaseUI baseUI)
   {
      for (RobotSide side : RobotSide.values)
         actionHandPoseGraphics.put(side, new RDXReferenceFrameGraphic(0.15));

      baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);
   }

   public void update()
   {
      if (actionHandPosesSubscription.poll())
      {
         // msg.data is [xL, yL, zL, qxL, qyL, qzL, qsL, xR, … ]
         IDLSequence.Float data = actionHandPosesSubscription.read().getData();
         int index = 0;
         for (RobotSide side : RobotSide.values)
         {
            handPose.getPosition().set(data.get(index++), data.get(index++), data.get(index++));
            handPose.getOrientation().set(data.get(index++), data.get(index++), data.get(index++), data.get(index++));
            actionHandPoseGraphics.get(side).setPoseInWorldFrame(handPose);
         }
      }

//      if(receivedActions == 1)
//      {
//         isKSTEnabled.set(true);
//         setKSTEnabled(isKSTEnabled.get());
//         reinitializeToolboxConfiguration();
//         controlArmsOnly.set(true);
//         KinematicsStreamingToolboxConfigurationMessage config = kstParameters.getDefaultConfiguration();
//         config.setLockPelvis(controlArmsOnly.get());
//         config.setLockChest(controlArmsOnly.get());
//         kstConfigPublisher.publish(config);
//      }
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

      ImGui.text("LeRobot: Thread: %s   Python: %3d Hz   Actions: %d".formatted(commsFrequencyText.getText(), (int) pythonStatusFrequency, receivedActions));
      if (ImGui.checkbox(labels.get("Run inference & preview"), running.getValue()))
         running.setValue(!running.getValue());
      if (ImGui.checkbox(labels.get("Control robot"), controlRobot.getValue()))
         controlRobot.setValue(!controlRobot.getValue());
      if (ImGui.checkbox(labels.get("Enable Kinematic Streaming"), isKSTEnabled))
      {
         setKSTEnabled(isKSTEnabled.get());
      }
      if (ImGui.checkbox(labels.get("Control Arms Only"), controlArmsOnly))
      {
         KinematicsStreamingToolboxConfigurationMessage config = kstParameters.getDefaultConfiguration();
         config.setLockPelvis(controlArmsOnly.get());
         config.setLockChest(controlArmsOnly.get());
         kstConfigPublisher.publish(config);
      }
      if (ImGui.button(labels.get("Reinitialize Toolbox Configuration")))
      {
         reinitializeToolboxConfiguration();
      }
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

   private void setKSTEnabled(boolean enabled)
   {
      if (enabled)
      {
         wakeUpToolbox();
      }
      else
      {
         sleepToolbox();
         controlArmsOnly.set(false);
      }
      isKSTEnabled.set(enabled);
   }

   private void wakeUpToolbox()
   {
      ToolboxStateMessage msg = new ToolboxStateMessage();
      msg.setRequestedToolboxState(ToolboxState.WAKE_UP.toByte());
      toolboxStatePublisher.publish(msg);
   }

   private void sleepToolbox()
   {
      ToolboxStateMessage msg = new ToolboxStateMessage();
      msg.setRequestedToolboxState(ToolboxState.SLEEP.toByte());
      toolboxStatePublisher.publish(msg);
   }

   private void reinitializeToolboxConfiguration()
   {
      sleepToolbox();
      KinematicsStreamingToolboxInitialConfigurationMessage initMsg = KinematicsToolboxMessageFactory.initialConfigurationFromFullRobotModel(syncedRobotModel.getFullRobotModel());
      kstInitialConfigPublisher.publish(initMsg);
      ToolboxStateMessage reinitMsg = new ToolboxStateMessage();
      reinitMsg.setRequestedToolboxState(ToolboxState.REINITIALIZE.toByte());
      toolboxStatePublisher.publish(reinitMsg);
      wakeUpToolbox();
   }

   private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (running.getValue())
         for (RobotSide side : RobotSide.values)
            actionHandPoseGraphics.get(side).getRenderables(renderables, pool);
   }
}
