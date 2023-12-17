package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.communication.ros2.ROS2ControllerPublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.motionRetargeting.VRTrackedSegmentType;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.imgui.ImGuiFrequencyPlot;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public class RDXIKStreamer
{
   private static final double FRAME_AXIS_GRAPHICS_LENGTH = 0.2;
   private final DRCRobotModel robotModel;
   private final ROS2ControllerPublishSubscribeAPI ros2ControllerHelper;
   private final SideDependentList<RDXInteractableHand> interactableHands;
   private RDXMultiBodyGraphic ghostRobotGraphic;
   private FullHumanoidRobotModel ghostFullRobotModel;
   private OneDoFJointBasics[] ghostOneDoFJointsExcludingHands;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean enabled = new ImBoolean(false);
   private IHMCROS2Input<KinematicsToolboxOutputStatus> status;
   private final double streamPeriod = UnitConversions.hertzToSeconds(10.0);
   private final Throttler toolboxInputStreamRateLimiter = new Throttler();
   private final FramePose3D controlFramePose = new FramePose3D();
   private final ImGuiFrequencyPlot statusFrequencyPlot = new ImGuiFrequencyPlot();
   private final ImGuiFrequencyPlot outputFrequencyPlot = new ImGuiFrequencyPlot();
   private final ImBoolean wakeUpThreadRunning = new ImBoolean(false);
   private final SideDependentList<MutableReferenceFrame> handDesiredControlFrames = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> controlFrameGraphics = new SideDependentList<>();
   private final ImBoolean showReferenceFrameGraphics = new ImBoolean(true);
   private final ImBoolean streamToController = new ImBoolean(false);
   private final Throttler messageThrottler = new Throttler();
   private KinematicsStreamingToolboxModule toolbox;
   private ReferenceFrame pelvisFrame;
   private RigidBodyTransform pelvisTransformToWorld = new RigidBodyTransform();
   private ReferenceFrame chestFrame;
   private RigidBodyTransform chestTransformToWorld = new RigidBodyTransform();
   private final ROS2SyncedRobotModel syncedRobot;
   private FramePose3D tempFramePose = new FramePose3D();

   public RDXIKStreamer(DRCRobotModel robotModel,
                        ROS2ControllerPublishSubscribeAPI ros2ControllerHelper,
                        SideDependentList<RDXInteractableHand> interactableHands,
                        ROS2SyncedRobotModel syncedRobot)
   {
      this.robotModel = robotModel;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.interactableHands = interactableHands;
      this.syncedRobot = syncedRobot;
   }

   public void create(RDXVRContext vrContext)
   {
      RobotDefinition ghostRobotDefinition = new RobotDefinition(robotModel.getRobotDefinition());
      MaterialDefinition material = new MaterialDefinition(ColorDefinitions.parse("0xDEE934").derive(0.0, 1.0, 1.0, 0.5));
      RobotDefinition.forEachRigidBodyDefinition(ghostRobotDefinition.getRootBodyDefinition(),
                                                 body -> body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(material)));

      ghostFullRobotModel = robotModel.createFullRobotModel();
      ghostOneDoFJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(ghostFullRobotModel);
      ghostRobotGraphic = new RDXMultiBodyGraphic(robotModel.getSimpleRobotName() + " (IK Preview Ghost)");
      ghostRobotGraphic.loadRobotModelAndGraphics(ghostRobotDefinition, ghostFullRobotModel.getElevator());
      ghostRobotGraphic.setActive(true);
      ghostRobotGraphic.create();

      for (RobotSide side : RobotSide.values)
      {
         controlFrameGraphics.put(side, new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
      }

      status = ros2ControllerHelper.subscribe(KinematicsStreamingToolboxModule.getOutputStatusTopic(robotModel.getSimpleRobotName()));

      // FIXME This should be a parameter of the toolbox itself.
      OneDoFJointDefinition spinePitchDefinition = (OneDoFJointDefinition) robotModel.getRobotDefinition().getJointDefinition("SPINE_Y");
      spinePitchDefinition.setPositionLowerLimit(0.0);

      for (RobotSide side : RobotSide.values)
      {
         String jointName = side.getUpperCaseName() + "_ELBOW_Y";
         OneDoFJointDefinition elbowDefinition = (OneDoFJointDefinition) robotModel.getRobotDefinition().getJointDefinition(jointName);
         double limit = 0.25;
         elbowDefinition.setPositionUpperLimit(-limit);
      }

      KinematicsStreamingToolboxParameters parameters = new KinematicsStreamingToolboxParameters();
      parameters.setDefault();
      parameters.setPublishingPeriod(0.030); // Publishing period in seconds.
      parameters.setDefaultChestMessageAngularWeight(0.15, 0.15, 0.02);
      parameters.setDefaultLinearRateLimit(5.0);
      parameters.setDefaultAngularRateLimit(30.0);
      parameters.getDefaultConfiguration().setEnableLeftHandTaskspace(false);
      parameters.getDefaultConfiguration().setEnableRightHandTaskspace(false);
      parameters.getDefaultConfiguration().setEnableNeckJointspace(false);
      parameters.setUseStreamingPublisher(true);

      boolean startYoVariableServer = false;
      DomainFactory.PubSubImplementation pubSubImplementation = DomainFactory.PubSubImplementation.FAST_RTPS;
      toolbox = new KinematicsStreamingToolboxModule(robotModel, parameters, startYoVariableServer, pubSubImplementation);
      ((KinematicsStreamingToolboxController) toolbox.getToolboxController()).setInitialRobotConfigurationNamedMap(createInitialConfiguration(robotModel));
   }

   private Map<String, Double> createInitialConfiguration(DRCRobotModel robotModel)
   {
      Map<String, Double> initialConfigurationMap = new HashMap<>();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      RobotInitialSetup<HumanoidFloatingRootJointRobot> defaultRobotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);
      FullHumanoidRobotModel robot = robotModel.createFullRobotModel();
      HumanoidJointNameMap jointMap = robotModel.getJointMap();
      defaultRobotInitialSetup.initializeFullRobotModel(robot);

      for (OneDoFJointBasics joint : fullRobotModel.getOneDoFJoints())
      {
         String jointName = joint.getName();
         double q_priv = robot.getOneDoFJointByName(jointName).getQ();
         initialConfigurationMap.put(jointName, q_priv);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         initialConfigurationMap.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH), 0.5);
         initialConfigurationMap.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfRightSide(0.0));
         initialConfigurationMap.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), robotSide.negateIfRightSide(-0.5));
         initialConfigurationMap.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), -1.0);

         initialConfigurationMap.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), -0.58);
         initialConfigurationMap.put(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), 0.55 + 0.672);
         initialConfigurationMap.put(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH), -0.64);
      }

      return initialConfigurationMap;
   }

   private KinematicsToolboxRigidBodyMessage createRigidBodyMessage(RigidBodyBasics segment,
                                                                    ReferenceFrame desiredControlFrame,
                                                                    double positionWeight,
                                                                    double orientationWeight)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(segment.hashCode());

      controlFramePose.setToZero(desiredControlFrame);
      controlFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      message.getDesiredOrientationInWorld().set(controlFramePose.getOrientation());
      message.getDesiredPositionInWorld().set(controlFramePose.getPosition());

      if (!Double.isNaN(positionWeight))
      {
         if (positionWeight == 0)
         {
            message.getLinearSelectionMatrix().setXSelected(false);
            message.getLinearSelectionMatrix().setYSelected(false);
            message.getLinearSelectionMatrix().setZSelected(false);
         }
         else
         {
            message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(positionWeight));
         }
      }
      if (!Double.isNaN(orientationWeight))
      {
         if (orientationWeight == 0)
         {
            message.getAngularSelectionMatrix().setXSelected(false);
            message.getAngularSelectionMatrix().setYSelected(false);
            message.getAngularSelectionMatrix().setZSelected(false);
         }
         else
         {
            message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(orientationWeight));
         }
      }
      return message;
   }

   public void update(boolean ikStreamingModeEnabled)
   {
      // Safety features!
      if (!ikStreamingModeEnabled)
         streamToController.set(false);
      setEnabled(ikStreamingModeEnabled);
      if (!enabled.get())
         streamToController.set(false);

      if (status.getMessageNotification().poll())
      {
         KinematicsToolboxOutputStatus latestStatus = status.getMessageNotification().read();
         statusFrequencyPlot.recordEvent();
         if (latestStatus.getJointNameHash() == -1)
         {
            if (latestStatus.getCurrentToolboxState() == KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_INITIALIZE_FAILURE_MISSING_RCD
                && messageThrottler.run(1.0))
               LogTools.warn("Status update: Toolbox failed initialization, missing RobotConfigurationData.");
            else if (latestStatus.getCurrentToolboxState() == KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_INITIALIZE_SUCCESSFUL)
               LogTools.info("Status update: Toolbox initialized successfully.");
         }
         else
         {
            // update IK ghost robot
            ghostFullRobotModel.getRootJoint().setJointPosition(latestStatus.getDesiredRootPosition());
            ghostFullRobotModel.getRootJoint().setJointOrientation(latestStatus.getDesiredRootOrientation());
            for (int i = 0; i < ghostOneDoFJointsExcludingHands.length; i++)
            {
               ghostOneDoFJointsExcludingHands[i].setQ(latestStatus.getDesiredJointAngles().get(i));
            }
            ghostFullRobotModel.getElevator().updateFramesRecursively();
         }
      }
      ghostRobotGraphic.update();

      if (enabled.get() && toolboxInputStreamRateLimiter.run(streamPeriod))
      {
         KinematicsStreamingToolboxInputMessage toolboxInputMessage = new KinematicsStreamingToolboxInputMessage();

         for (RobotSide side : RobotSide.values)
         {
            if (!interactableHands.get(side).isDeleted())
            {
               ReferenceFrame controlFrame = interactableHands.get(side).getControlReferenceFrame();
               controlFrameGraphics.get(side).setToReferenceFrame(controlFrame);

               MutableReferenceFrame handDesiredControlFrame = new MutableReferenceFrame(controlFrame);
               {
                  handDesiredControlFrame.getTransformToParent().appendTranslation(new Point3D(0.0, 0.0, 0.2));
               }
               handDesiredControlFrame.getReferenceFrame().update();

               VRTrackedSegmentType trackedSegmentHand = side == RobotSide.LEFT ? VRTrackedSegmentType.LEFT_HAND : VRTrackedSegmentType.RIGHT_HAND;

               KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(ghostFullRobotModel.getHand(side),
                                                                                  handDesiredControlFrame.getReferenceFrame(),
                                                                                  trackedSegmentHand.getPositionWeight(),
                                                                                  trackedSegmentHand.getOrientationWeight());
               toolboxInputMessage.getInputs().add().set(message);
            }

            if (pelvisFrame == null)
            {
               pelvisTransformToWorld.set(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame().getTransformToWorldFrame());
               pelvisFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), pelvisTransformToWorld);
            }

            KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
            message.setEndEffectorHashCode(ghostFullRobotModel.getPelvis().hashCode());
            tempFramePose.setToZero(pelvisFrame);
            tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
            message.getDesiredPositionInWorld().set(tempFramePose.getPosition());
            message.getDesiredOrientationInWorld().set(tempFramePose.getOrientation());
            message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(50));
            message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(50));

            toolboxInputMessage.getInputs().add().set(message);

            if (chestFrame == null)
            {
               chestTransformToWorld.set(syncedRobot.getFullRobotModel().getChest().getBodyFixedFrame().getTransformToWorldFrame());
               chestFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), chestTransformToWorld);
            }

            message = new KinematicsToolboxRigidBodyMessage();
            message.setEndEffectorHashCode(ghostFullRobotModel.getChest().hashCode());
            tempFramePose.setToZero(chestFrame);
            tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
            message.getDesiredOrientationInWorld().set(tempFramePose.getOrientation());
            message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(0));
            message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(1));

            toolboxInputMessage.getInputs().add().set(message);
         }

         toolboxInputMessage.setStreamToController(streamToController.get());
         toolboxInputMessage.setTimestamp(System.nanoTime());
         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputCommandTopic(robotModel.getSimpleRobotName()), toolboxInputMessage);
         outputFrequencyPlot.recordEvent();
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox(labels.get("Stream to Controller"), streamToController);

      ghostRobotGraphic.renderImGuiWidgets();
      if (ImGui.button(labels.get("Reinitialize")))
      {
         reinitializeToolbox();
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Wake up")))
      {
         wakeUpToolbox();
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Sleep")))
      {
         sleepToolbox();
      }
      ImGui.text("Output:");
      ImGui.sameLine();
      outputFrequencyPlot.renderImGuiWidgets();
      ImGui.text("Status:");
      ImGui.sameLine();
      statusFrequencyPlot.renderImGuiWidgets();

      ImGui.checkbox(labels.get("Show reference frames"), showReferenceFrameGraphics);
   }

   public void setEnabled(boolean enabled)
   {
      if (enabled != this.enabled.get())
         this.enabled.set(enabled);
      if (enabled)
      {
         wakeUpToolbox();
      }
   }

   private void reinitializeToolbox()
   {
      ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
      toolboxStateMessage.setRequestedToolboxState(ToolboxState.REINITIALIZE.toByte());
      ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStateTopic(robotModel.getSimpleRobotName()), toolboxStateMessage);
   }

   private void wakeUpToolbox()
   {
      ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
      toolboxStateMessage.setRequestedToolboxState(ToolboxState.WAKE_UP.toByte());
      ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStateTopic(robotModel.getSimpleRobotName()), toolboxStateMessage);
   }

   private void sleepToolbox()
   {
      ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
      toolboxStateMessage.setRequestedToolboxState(ToolboxState.SLEEP.toByte());
      ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStateTopic(robotModel.getSimpleRobotName()), toolboxStateMessage);
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
         if (status.hasReceivedFirstMessage())
         {
            ghostRobotGraphic.getRenderables(renderables, pool, sceneLevels);
         }
         if (showReferenceFrameGraphics.get())
         {
            for (RobotSide side : RobotSide.values)
               controlFrameGraphics.get(side).getRenderables(renderables, pool);
         }
      }
   }

   public void destroy()
   {
      toolbox.closeAndDispose();
      ghostRobotGraphic.destroy();
      for (RobotSide side : RobotSide.values)
      {
         controlFrameGraphics.get(side).dispose();
      }
   }

   public void sendHandCommand(RobotSide robotSide, HandConfiguration desiredHandConfiguration)
   {
      ros2ControllerHelper.publish(ROS2Tools::getHandConfigurationTopic,
                                   HumanoidMessageTools.createHandDesiredConfigurationMessage(robotSide, desiredHandConfiguration));
   }
}
