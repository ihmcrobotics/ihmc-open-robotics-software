package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.motionRetargeting.RetargetingParameters;
import us.ihmc.motionRetargeting.VRTrackedSegmentType;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.processes.RestartableJavaProcess;
import us.ihmc.rdx.ui.tools.KinematicsRecordReplay;
import us.ihmc.rdx.imgui.ImGuiFrequencyPlot;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rdx.vr.RDXVRControllerModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public class RDXVRKinematicsStreamingMode
{
   private static final double FRAME_AXIS_GRAPHICS_LENGTH = 0.2;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final RestartableJavaProcess kinematicsStreamingToolboxProcess;
   private final RetargetingParameters retargetingParameters;
   private final DRCRobotModel robotModel;
   private RDXMultiBodyGraphic ghostRobotGraphic;
   private FullHumanoidRobotModel ghostFullRobotModel;
   private OneDoFJointBasics[] ghostOneDoFJointsExcludingHands;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean enabled = new ImBoolean(false);
   private IHMCROS2Input<KinematicsToolboxOutputStatus> status;
   private final double streamPeriod = UnitConversions.hertzToSeconds(120.0);
   private final Throttler toolboxInputStreamRateLimiter = new Throttler();
   private final FramePose3D tempFramePose = new FramePose3D();
   private final ImGuiFrequencyPlot statusFrequencyPlot = new ImGuiFrequencyPlot();
   private final ImGuiFrequencyPlot outputFrequencyPlot = new ImGuiFrequencyPlot();
   private final SideDependentList<MutableReferenceFrame> handDesiredControlFrames = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> controllerFrameGraphics = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> handFrameGraphics = new SideDependentList<>();
   private final Map<String, MutableReferenceFrame> trackedSegmentDesiredFrame = new HashMap<>();
   private final Map<String, RDXReferenceFrameGraphic> trackerFrameGraphics = new HashMap<>();
   private final ImBoolean showReferenceFrameGraphics = new ImBoolean(true);
   private final ImBoolean streamToController = new ImBoolean(false);
   private final Throttler messageThrottler = new Throttler();
   private KinematicsRecordReplay kinematicsRecorder;
   private final SceneGraph sceneGraph;
   private KinematicsStreamingToolboxModule toolbox;

   private final HandConfiguration[] handConfigurations = {HandConfiguration.HALF_CLOSE, HandConfiguration.CRUSH, HandConfiguration.CLOSE};
   private int leftIndex = -1;
   private int rightIndex = -1;
   private RDXVRControllerModel controllerModel = RDXVRControllerModel.UNKNOWN;

   public RDXVRKinematicsStreamingMode(ROS2SyncedRobotModel syncedRobot,
                                       ROS2ControllerHelper ros2ControllerHelper,
                                       SceneGraph sceneGraph,
                                       RestartableJavaProcess kinematicsStreamingToolboxProcess,
                                       RetargetingParameters retargetingParameters)
   {
      this.syncedRobot = syncedRobot;
      this.robotModel = syncedRobot.getRobotModel();
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.sceneGraph = sceneGraph;
      this.kinematicsStreamingToolboxProcess = kinematicsStreamingToolboxProcess;
      this.retargetingParameters = retargetingParameters;
   }

   public void create(RDXVRContext vrContext)
   {
      RobotDefinition ghostRobotDefinition = new RobotDefinition(syncedRobot.getRobotModel().getRobotDefinition());
      MaterialDefinition material = new MaterialDefinition(ColorDefinitions.parse("0xDEE934").derive(0.0, 1.0, 1.0, 0.5));
      RobotDefinition.forEachRigidBodyDefinition(ghostRobotDefinition.getRootBodyDefinition(),
                                                 body -> body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(material)));

      ghostFullRobotModel = syncedRobot.getRobotModel().createFullRobotModel();
      ghostOneDoFJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(ghostFullRobotModel);
      ghostRobotGraphic = new RDXMultiBodyGraphic(syncedRobot.getRobotModel().getSimpleRobotName() + " (IK Preview Ghost)");
      ghostRobotGraphic.loadRobotModelAndGraphics(ghostRobotDefinition, ghostFullRobotModel.getElevator());
      ghostRobotGraphic.setActive(true);
      ghostRobotGraphic.create();

      for (RobotSide side : RobotSide.values)
      {
         handFrameGraphics.put(side, new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
         controllerFrameGraphics.put(side, new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
         MutableReferenceFrame handDesiredControlFrame = new MutableReferenceFrame(vrContext.getController(side).getXForwardZUpControllerFrame());
         {
            if (side == RobotSide.LEFT)
            {
               handDesiredControlFrame.getTransformToParent().appendOrientation(retargetingParameters.getYawPitchRollFromTracker(VRTrackedSegmentType.LEFT_HAND));
               handDesiredControlFrame.getTransformToParent().appendTranslation(retargetingParameters.getTranslationFromTracker(VRTrackedSegmentType.LEFT_HAND));
            }
            else
            {
               handDesiredControlFrame.getTransformToParent().appendOrientation(retargetingParameters.getYawPitchRollFromTracker(VRTrackedSegmentType.RIGHT_HAND));
               handDesiredControlFrame.getTransformToParent().appendTranslation(retargetingParameters.getTranslationFromTracker(VRTrackedSegmentType.RIGHT_HAND));
            }
         }
         handDesiredControlFrame.getReferenceFrame().update();
         handDesiredControlFrames.put(side, handDesiredControlFrame);
      }

      status = ros2ControllerHelper.subscribe(KinematicsStreamingToolboxModule.getOutputStatusTopic(syncedRobot.getRobotModel().getSimpleRobotName()));

      kinematicsRecorder = new KinematicsRecordReplay(sceneGraph, enabled);

      KinematicsStreamingToolboxParameters parameters = new KinematicsStreamingToolboxParameters();
      parameters.setDefault();
      parameters.setPublishingPeriod(0.030); // Publishing period in seconds.
      parameters.setDefaultChestMessageAngularWeight(0.15, 0.15, 0.02);
      parameters.setDefaultLinearRateLimit(3.0);
      parameters.setDefaultAngularRateLimit(30.0);
      parameters.getDefaultConfiguration().setEnableLeftHandTaskspace(false);
      parameters.getDefaultConfiguration().setEnableRightHandTaskspace(false);
      parameters.getDefaultConfiguration().setEnableNeckJointspace(false);
      parameters.setUseStreamingPublisher(true);

      boolean startYoVariableServer = false;
      toolbox = new KinematicsStreamingToolboxModule(robotModel, parameters, startYoVariableServer, PubSubImplementation.FAST_RTPS);
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
         // TODO: Extract preset configuration to robot model
         initialConfigurationMap.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH), 0.5);
         initialConfigurationMap.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfRightSide(0.13));
         initialConfigurationMap.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), 0.13);
         initialConfigurationMap.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), -1.0);

         initialConfigurationMap.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), -0.58);
         initialConfigurationMap.put(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), 0.55 + 0.672);
         initialConfigurationMap.put(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH), -0.64);
      }

      return initialConfigurationMap;
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      if (controllerModel == RDXVRControllerModel.UNKNOWN)
         controllerModel = vrContext.getControllerModel();
      vrContext.getController(RobotSide.LEFT).runIfConnected(controller ->
      {
         InputDigitalActionData aButton = controller.getAButtonActionData();
         if (aButton.bChanged() && !aButton.bState())
         {
            streamToController.set(!streamToController.get());
         }

         // NOTE: Implement hand open close for controller trigger button.
         InputDigitalActionData clickTriggerButton = controller.getClickTriggerActionData();
         if (clickTriggerButton.bChanged() && !clickTriggerButton.bState())
         {
            HandConfiguration handConfiguration = nextHandConfiguration(RobotSide.LEFT);
            sendHandCommand(RobotSide.LEFT, handConfiguration);
         }

         // Check if left joystick is pressed in order to trigger recording or replay of motion
         InputDigitalActionData joystickButton = controller.getJoystickPressActionData();
         kinematicsRecorder.processRecordReplayInput(joystickButton);
         if (kinematicsRecorder.isReplayingEnabled().get())
            wakeUpToolbox();
      });

      vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
        {
           InputDigitalActionData aButton = controller.getAButtonActionData();
           if (aButton.bChanged() && !aButton.bState())
           {
              setEnabled(!enabled.get());
           }

           // NOTE: Implement hand open close for controller trigger button.
           InputDigitalActionData clickTriggerButton = controller.getClickTriggerActionData();
           if (clickTriggerButton.bChanged() && !clickTriggerButton.bState())
           {
              HandConfiguration handConfiguration = nextHandConfiguration(RobotSide.RIGHT);
              sendHandCommand(RobotSide.RIGHT, handConfiguration);
           }
        });

      if ((enabled.get() || kinematicsRecorder.isReplaying()) && toolboxInputStreamRateLimiter.run(streamPeriod))
      {
         KinematicsStreamingToolboxInputMessage toolboxInputMessage = new KinematicsStreamingToolboxInputMessage();
         Set<String> additionalTrackedSegments = vrContext.getBodySegmentsWithTrackers();
         for (VRTrackedSegmentType segmentType : VRTrackedSegmentType.values())
            handleTrackedSegment(vrContext, toolboxInputMessage, segmentType, additionalTrackedSegments);

         if (enabled.get())
            toolboxInputMessage.setStreamToController(streamToController.get());
         else
            toolboxInputMessage.setStreamToController(kinematicsRecorder.isReplaying());
         toolboxInputMessage.setTimestamp(System.nanoTime());
         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputCommandTopic(syncedRobot.getRobotModel().getSimpleRobotName()), toolboxInputMessage);
         outputFrequencyPlot.recordEvent();
      }
   }

   private void handleTrackedSegment(RDXVRContext vrContext,
                                     KinematicsStreamingToolboxInputMessage toolboxInputMessage,
                                     VRTrackedSegmentType segmentType,
                                     Set<String> additionalTrackedSegments)
   {
      if (additionalTrackedSegments.contains(segmentType.getSegmentName()))
      {
         vrContext.getTracker(segmentType.getSegmentName()).runIfConnected(tracker ->
         {
            if (!trackerFrameGraphics.containsKey(segmentType.getSegmentName()))
            {
               trackerFrameGraphics.put(segmentType.getSegmentName(),
                                        new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
            }
            if (!trackedSegmentDesiredFrame.containsKey(segmentType.getSegmentName()))
            {
               MutableReferenceFrame trackerDesiredControlFrame = new MutableReferenceFrame(tracker.getXForwardZUpTrackerFrame());
               trackerDesiredControlFrame.getTransformToParent().appendOrientation(retargetingParameters.getYawPitchRollFromTracker(segmentType));
               trackerDesiredControlFrame.getReferenceFrame().update();
               trackedSegmentDesiredFrame.put(segmentType.getSegmentName(), trackerDesiredControlFrame);
            }
            trackerFrameGraphics.get(segmentType.getSegmentName()).setToReferenceFrame(trackedSegmentDesiredFrame.get(segmentType.getSegmentName()).getReferenceFrame());
            RigidBodyBasics controlledSegment = switch (segmentType)
                  {
                     case LEFT_FOREARM -> ghostFullRobotModel.getForearm(RobotSide.LEFT);
                     case RIGHT_FOREARM -> ghostFullRobotModel.getForearm(RobotSide.RIGHT);
                     case CHEST -> ghostFullRobotModel.getChest();
                     default -> throw new IllegalStateException(
                           "Unexpected VR-tracked segment: " + segmentType);
                  };
            KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(controlledSegment,
                                                                               trackedSegmentDesiredFrame.get(segmentType.getSegmentName()).getReferenceFrame(),
                                                                               segmentType.getSegmentName(),
                                                                               segmentType.getPositionWeight(),
                                                                               segmentType.getOrientationWeight());
            toolboxInputMessage.getInputs().add().set(message);
         });
      }
      else if (segmentType.getSegmentName().contains("Hand"))
      {
         vrContext.getController(segmentType.getSegmentSide()).runIfConnected(controller ->
         {
            controllerFrameGraphics.get(segmentType.getSegmentSide())
                                   .setToReferenceFrame(controller.getXForwardZUpControllerFrame());
            handFrameGraphics.get(segmentType.getSegmentSide()).setToReferenceFrame(ghostFullRobotModel.getEndEffectorFrame(segmentType.getSegmentSide(), LimbName.ARM));
            KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(ghostFullRobotModel.getHand(
                                                                                     segmentType.getSegmentSide()),
                                                                               handDesiredControlFrames.get(
                                                                                     segmentType.getSegmentSide()).getReferenceFrame(),
                                                                               segmentType.getSegmentName(),
                                                                               segmentType.getPositionWeight(),
                                                                               segmentType.getOrientationWeight());
            toolboxInputMessage.getInputs().add().set(message);
         });
      }
   }

   private KinematicsToolboxRigidBodyMessage createRigidBodyMessage(RigidBodyBasics segment,
                                                                    ReferenceFrame desiredControlFrame,
                                                                    String frameName,
                                                                    double positionWeight,
                                                                    double orientationWeight)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(segment.hashCode());

      tempFramePose.setToZero(desiredControlFrame);
      tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());

      // Check if controllers or trackers have been occluded in that frame and reset by default to the World origin
      if (tempFramePose.getPosition().getZ() < 0.05)
         streamToController.set(false);
      // record motion if in recording mode
      kinematicsRecorder.framePoseToRecord(tempFramePose, frameName);
      if (kinematicsRecorder.isReplaying())
         kinematicsRecorder.framePoseToPack(tempFramePose); //get values of tempFramePose from replay

      message.getDesiredOrientationInWorld().set(tempFramePose.getOrientation());
      message.getDesiredPositionInWorld().set(tempFramePose.getPosition());

      if (!Double.isNaN(positionWeight))
      {
         if (positionWeight == 0.0)
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
         if (orientationWeight == 0.0)
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
      else
      {
         if (!enabled.get())
            streamToController.set(false);

         if (enabled.get() || kinematicsRecorder.isReplaying())
         {
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
            if (ghostRobotGraphic.isActive())
               ghostRobotGraphic.update();
         }
      }
   }

   public void renderImGuiWidgets()
   {
      if (controllerModel == RDXVRControllerModel.FOCUS3)
         ImGui.text("X Button");
      else
         ImGui.text("Left A Button");
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Control/Stop Robot"), streamToController);
      ImGui.text("Right A Button");
      ImGui.sameLine();
      if (ImGui.checkbox(labels.get("Kinematics streaming"), enabled))
      {
         setEnabled(enabled.get());
      }

      ghostRobotGraphic.renderImGuiWidgets();
      // add widgets for recording/replaying motion in VR
      ImGui.text("Press Left Joystick - Start/Stop recording");
      kinematicsRecorder.renderRecordWidgets(labels);
      ImGui.text("Press Left Joystick - Start/Stop replay");
      kinematicsRecorder.renderReplayWidgets(labels);
      kinematicsRecorder.renderReferenceFrameSelection(labels);
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
         kinematicsRecorder.setReplay(false); //check no concurrency replay and streaming
      }
   }

   private void reinitializeToolbox()
   {
      ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
      toolboxStateMessage.setRequestedToolboxState(ToolboxState.REINITIALIZE.toByte());
      ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStateTopic(syncedRobot.getRobotModel().getSimpleRobotName()), toolboxStateMessage);
   }

   private void wakeUpToolbox()
   {
      ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
      toolboxStateMessage.setRequestedToolboxState(ToolboxState.WAKE_UP.toByte());
      ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStateTopic(syncedRobot.getRobotModel().getSimpleRobotName()), toolboxStateMessage);
   }

   private void sleepToolbox()
   {
      ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
      toolboxStateMessage.setRequestedToolboxState(ToolboxState.SLEEP.toByte());
      ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStateTopic(syncedRobot.getRobotModel().getSimpleRobotName()), toolboxStateMessage);
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (status.hasReceivedFirstMessage())
      {
         ghostRobotGraphic.getRenderables(renderables, pool, sceneLevels);
      }
      if (showReferenceFrameGraphics.get())
      {
         for (RobotSide side : RobotSide.values)
         {
            controllerFrameGraphics.get(side).getRenderables(renderables, pool);
            handFrameGraphics.get(side).getRenderables(renderables, pool);
         }
         for (var trackerGraphics : trackerFrameGraphics.entrySet())
            trackerGraphics.getValue().getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      toolbox.closeAndDispose();
      ghostRobotGraphic.destroy();
      for (RobotSide side : RobotSide.values)
      {
         controllerFrameGraphics.get(side).dispose();
         handFrameGraphics.get(side).dispose();
      }
   }

   public void sendHandCommand(RobotSide robotSide, HandConfiguration desiredHandConfiguration)
   {
      ros2ControllerHelper.publish(ROS2Tools::getHandConfigurationTopic,
                                   HumanoidMessageTools.createHandDesiredConfigurationMessage(robotSide, desiredHandConfiguration));
   }

   public HandConfiguration nextHandConfiguration(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
      {
         leftIndex++;
         return handConfigurations[leftIndex % handConfigurations.length];
      }
      rightIndex++;
      return handConfigurations[rightIndex % handConfigurations.length];
   }
}