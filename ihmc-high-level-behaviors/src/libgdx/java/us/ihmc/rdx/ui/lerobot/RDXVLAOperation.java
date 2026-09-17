package us.ihmc.rdx.ui.lerobot;

import static us.ihmc.lerobot.VLAUpdateThread.UI;

import behavior_msgs.VLAOperationMessage;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.ArmTrajectoryMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import toolbox_msgs.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.KinematicsToolboxRigidBodyMessage;
import toolbox_msgs.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ToolboxAPIs;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.lerobot.VLAUpdateThread;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiAveragedFrequencyText;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.Timer;

/**
 * UI for remotely operating {@link VLAUpdateThread}.
 */
public class RDXVLAOperation
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final Throttler commandThrottler = new Throttler().setFrequency(30.0);
   private final LatestTimestampModifiable latestTimestampModifiable;
   private final CRDTBidirectionalBoolean running;
   private final CRDTBidirectionalBoolean controlRobot;
   private final ROS2Publisher<ArmTrajectoryMessage> armTrajectoryPublisher;
   private String statusMessage = "Not yet connected to robot";
   private final TypedNotification<VLAOperationMessage> statusSubscription;
   private final VLAOperationMessage statusCopy = new VLAOperationMessage();
   private final ROS2Publisher<VLAOperationMessage> commandPublisher;
   private final ImGuiAveragedFrequencyText commsFrequencyText = new ImGuiAveragedFrequencyText();
   private final SideDependentList<RDXReferenceFrameGraphic> actionHandPoseGraphics = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> actionForearmPoseGraphics = new SideDependentList<>();

   private final ROS2SyncedRobotModel syncedRobot;
   private final FullHumanoidRobotModel fullRobotModel;
   private final ROS2Publisher<KinematicsStreamingToolboxInputMessage> kstInputPublisher;
   private final ROS2Publisher<ToolboxStateMessage> kstStatePublisher;
   private final ImBoolean generatingEpisodes = new ImBoolean(false);
   private final ImDouble circleSpeed = new ImDouble(0.5);
   private final Stopwatch stopwatch = new Stopwatch();
   private final Throttler throttler = new Throttler().setFrequency(50.0);
   private final FramePose3D handPose = new FramePose3D();
   private final FramePose3D forearmPose = new FramePose3D();
   private final Timer timer = new Timer();
   private int numberOfEpisodes = 0;

   public RDXVLAOperation(ROS2Node ros2Node, ROS2PeerClockOffsetEstimator peerClockEstimator, ROS2SyncedRobotModel syncedRobot)
   {
      latestTimestampModifiable = new LatestTimestampModifiable(new CRDTInfo(ROS2ActorDesignation.OPERATOR, peerClockEstimator));
      running = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);
      controlRobot = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);

      var statusTopic = UI.getTopic(ROS2ActorDesignation.OPERATOR.getIncomingQualifier());
      TypedNotification<VLAOperationMessage> typedNotification = new TypedNotification<>();
      ros2Node.createSubscriptionSampler(statusTopic, sample ->
      {
         statusCopy.set(sample);
         typedNotification.set(statusCopy);
      });
      statusSubscription = typedNotification;
      commandPublisher = ros2Node.createPublisher(UI.getTopic(ROS2ActorDesignation.OPERATOR.getOutgoingQualifier()));

      this.syncedRobot = syncedRobot;
      fullRobotModel = syncedRobot.getRobotModel().createFullRobotModel();
      String robotName = syncedRobot.getRobotModel().getSimpleRobotName();
      kstInputPublisher = ros2Node.createPublisher(ToolboxAPIs.getIKStreamingInputTopic(robotName));
      kstStatePublisher = ros2Node.createPublisher(ToolboxAPIs.getIKStreamingStateTopic(robotName));
      armTrajectoryPublisher = ros2Node.createPublisher(HumanoidControllerAPI.getTopic(ArmTrajectoryMessage.class, robotName));
   }

   public void create(RDXBaseUI baseUI)
   {
      for (RobotSide side : RobotSide.values)
      {
         actionHandPoseGraphics.put(side, new RDXReferenceFrameGraphic(0.15, Color.ORANGE));
         actionForearmPoseGraphics.put(side, new RDXReferenceFrameGraphic(0.15, Color.OLIVE));
      }

      baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);
      baseUI.getImGuiPanelManager().addPanel("VLA Operation", this::renderImGuiWidgets);
   }

   public void update()
   {

   }

   public void renderImGuiWidgets()
   {
      if (statusSubscription.poll())
      {
         commsFrequencyText.ping();
         VLAOperationMessage status = statusSubscription.read();
         latestTimestampModifiable.fromMessage(status.getLatestTimestampModifiable());
         running.fromMessage(status.getRunning());
         controlRobot.fromMessage(status.getControlRobot());
         for (RobotSide side : RobotSide.values)
         {
            actionHandPoseGraphics.get(side).setPoseInWorldFrame(status.getActionHandPoses()[side.ordinal()].getPose());
            actionForearmPoseGraphics.get(side).setPoseInWorldFrame(status.getActionForearmPoses()[side.ordinal()].getPose());
         }
         statusMessage = status.getStatusMessageAsString();
      }

      ImGui.text("Update Thread: %s".formatted(commsFrequencyText.getText()));
      ImGui.text("Python status: %s".formatted(statusMessage));
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

      if (commandThrottler.run())
      {
         VLAOperationMessage command = new VLAOperationMessage();
         latestTimestampModifiable.toMessage(command.getLatestTimestampModifiable());
         command.setRunning(running.toMessage());
         command.setControlRobot(controlRobot.toMessage());
         commandPublisher.publish(command);
      }

      ImGuiTools.separatorText("Test Episode Generation");

      RobotSide side = RobotSide.RIGHT;
      ReferenceFrame cameraFrame = syncedRobot.getReferenceFrames().getExperimentalCameraFrame();

      if (ImGui.button("Goto Initial Configuration"))
      {
         double[] jointAngles = { -1.176, -0.186, 1.121, 0.446, 0.757, 0.059, 0.152 };
         ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(side, 2.0, jointAngles);
         armTrajectoryPublisher.publish(armTrajectoryMessage);
      }

      if (ImGui.button("Print hand/forearm pose"))
      {
         handPose.setToZero(syncedRobot.getReferenceFrames().getHandFrame(side));
         handPose.changeFrame(cameraFrame);
         LogTools.info("Hand pose: {}", handPose);
         LogTools.info("Hand: yaw: %.2f pitch: %.2f roll: %.2f".formatted(handPose.getOrientation().getYaw(),
                                                                          handPose.getOrientation().getPitch(),
                                                                          handPose.getOrientation().getRoll()));
         forearmPose.setToZero(syncedRobot.getFullRobotModel().getForearm(side).getParentJoint().getFrameAfterJoint());
         forearmPose.changeFrame(cameraFrame);
         LogTools.info("Forearm: yaw: %.2f pitch: %.2f roll: %.2f".formatted(forearmPose.getOrientation().getYaw(),
                                                                             forearmPose.getOrientation().getPitch(),
                                                                             forearmPose.getOrientation().getRoll()));
      }

      boolean changed = ImGui.checkbox(labels.get("Generate episodes"), generatingEpisodes);

      ImGui.text("Number of episodes: %d".formatted(numberOfEpisodes));

      if (generatingEpisodes.get())
      {
         if (changed)
         {
            ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
            toolboxStateMessage.setRequestedToolboxState(ToolboxState.WAKE_UP.toByte());
            kstStatePublisher.publish(toolboxStateMessage);

            stopwatch.reset();
         }

         boolean waiting = timer.isRunning(1.0);

         if (waiting)
            stopwatch.reset();

         if (!waiting && throttler.run())
         {
            double t = stopwatch.totalElapsed();
            double rps = 0.25;
            double period = 1.0 / rps;

            KinematicsStreamingToolboxInputMessage ikInputMessage = new KinematicsStreamingToolboxInputMessage();
            if (t < period)
            {
               handPose.setToZero(cameraFrame);
               double planeDistance = 0.486;
               double radius = 0.15;
               double x = planeDistance;
               double cos = Math.cos(t * 2.0 * Math.PI / period);
               double y = radius * cos - 0.15;
               double sin = Math.sin(t * 2.0 * Math.PI / period);
               double z = radius * sin - 0.0;
               handPose.getPosition().set(x, y, z);
               handPose.getOrientation().setYawPitchRoll(1.0, 0.0, -0.80);
               handPose.changeFrame(ReferenceFrame.getWorldFrame());

               forearmPose.setToZero(cameraFrame);
               double yaw = 1.11;
               double pitch = -0.15;
               double roll = -2.40;
               forearmPose.getOrientation().setYawPitchRoll(yaw, pitch, roll);
               forearmPose.changeFrame(ReferenceFrame.getWorldFrame());

               ikInputMessage.setDemonstrationTaskId(0);
               ikInputMessage.setStreamToController(controlRobot.getValue());
               ikInputMessage.setTimestamp(Conversions.secondsToNanoseconds(t));
               KinematicsToolboxRigidBodyMessage rigidBodyMessage = new KinematicsToolboxRigidBodyMessage();
               rigidBodyMessage.setEndEffectorHashCode(fullRobotModel.getHand(side).hashCode());
               rigidBodyMessage.getDesiredPositionInWorld().set(handPose.getTranslation());
               rigidBodyMessage.getDesiredOrientationInWorld().set(handPose.getRotation());
               rigidBodyMessage.getAngularWeightMatrix().setXWeight(0.02);
               rigidBodyMessage.getAngularWeightMatrix().setYWeight(0.02);
               rigidBodyMessage.getAngularWeightMatrix().setZWeight(0.02);
               ikInputMessage.getInputs().add().set(rigidBodyMessage);

               rigidBodyMessage = new KinematicsToolboxRigidBodyMessage();
               rigidBodyMessage.setEndEffectorHashCode(fullRobotModel.getForearm(side).hashCode());
               rigidBodyMessage.getDesiredPositionInWorld().set(forearmPose.getTranslation());
               rigidBodyMessage.getLinearSelectionMatrix().setXSelected(false); // Disable position tracking for forearm
               rigidBodyMessage.getLinearSelectionMatrix().setYSelected(false);
               rigidBodyMessage.getLinearSelectionMatrix().setZSelected(false);
               rigidBodyMessage.getDesiredOrientationInWorld().set(forearmPose.getRotation());
               rigidBodyMessage.getAngularWeightMatrix().setXWeight(0.01);
               rigidBodyMessage.getAngularWeightMatrix().setYWeight(0.01);
               rigidBodyMessage.getAngularWeightMatrix().setZWeight(0.001);
      //            ikInputMessage.getInputs().add().set(rigidBodyMessage);
            }
            else
            {
               numberOfEpisodes++;
               timer.reset();
               ikInputMessage.setDemonstrationTaskId(-1);
            }
            kstInputPublisher.publish(ikInputMessage);
         }
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
