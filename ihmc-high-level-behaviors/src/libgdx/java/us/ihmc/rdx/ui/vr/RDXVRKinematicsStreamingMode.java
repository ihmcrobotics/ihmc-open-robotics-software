package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.GoHomeMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.lwjgl.openvr.InputDigitalActionData;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInitialConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.behaviors.tools.walkingController.SwingFootTracker;
import us.ihmc.commons.UnitConversions;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.motionRetargeting.RetargetingParameters;
import us.ihmc.motionRetargeting.VRTrackedSegmentType;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.rdx.imgui.ImGuiFrequencyPlot;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.tools.KinematicsRecordReplay;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRHardwareModel;
import us.ihmc.rdx.vr.RDXVRTeleporter;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;

import javax.annotation.Nullable;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import static us.ihmc.communication.packets.MessageTools.toFrameId;
import static us.ihmc.motionRetargeting.VRTrackedSegmentType.*;

public class RDXVRKinematicsStreamingMode
{
   public static final double FRAME_AXIS_GRAPHICS_LENGTH = 0.2;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final RetargetingParameters retargetingParameters;
   private final DRCRobotModel robotModel;
   private RDXMultiBodyGraphic ghostRobotGraphic;
   private FullHumanoidRobotModel ghostFullRobotModel;
   private OneDoFJointBasics[] ghostOneDoFJointsExcludingHands;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean enabled = new ImBoolean(false);

   @Nullable
   private KinematicsStreamingToolboxModule toolbox;
   private KinematicsStreamingToolboxParameters kstParameters;
   private final KinematicsToolboxConfigurationMessage ikSolverConfigurationMessage = new KinematicsToolboxConfigurationMessage();
   private ROS2Input<KinematicsToolboxOutputStatus> status;
   private final double streamPeriod = UnitConversions.hertzToSeconds(120.0);
   private final Throttler toolboxInputStreamRateLimiter = new Throttler();
   private final FramePose3D tempFramePose = new FramePose3D();
   private final ImGuiFrequencyPlot statusFrequencyPlot = new ImGuiFrequencyPlot();
   private final ImGuiFrequencyPlot outputFrequencyPlot = new ImGuiFrequencyPlot();
   private final SideDependentList<MutableReferenceFrame> handDesiredControlFrames = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> controllerFrameGraphics = new SideDependentList<>();
   private final SideDependentList<Pose3D> ikControlFramePoses = new SideDependentList<>();
   public long controllerLastPollTimeNanos;
   private final SideDependentList<RDXReferenceFrameGraphic> handFrameGraphics = new SideDependentList<>();
   private Set<String> additionalTrackedSegments = new HashSet<>();
   private final Map<String, MutableReferenceFrame> trackerReferenceFrames = new HashMap<>();
   private final Map<String, RDXReferenceFrameGraphic> trackerFrameGraphics = new HashMap<>();
   private final RDXReferenceFrameGraphic chestFrameGraphics = new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH);
   private MutableReferenceFrame headsetReferenceFrame;
   private final ImBoolean showReferenceFrameGraphics = new ImBoolean(false);
   private final ImBoolean streamToController = new ImBoolean(false);
   private final Notification streamingDisabled = new Notification();
   private final Throttler messageThrottler = new Throttler();

   private KinematicsRecordReplay kinematicsRecorder;
   private final SceneGraph sceneGraph;
   private final RDXVRContext vrContext;

   private final ImBoolean controlArmsOnly = new ImBoolean(false);
   private final ImBoolean armScaling = new ImBoolean(false);
   private final ImBoolean comTracking = new ImBoolean(false);
   private RDXVRMotionRetargeting motionRetargeting;

   private RDXVRFootstepStreaming footstepStreaming;
   private boolean reintializingToolbox = false;
   private boolean pausedForWalking = false;
   private double timeNotificationIsDoneWalking = 0.0;
   private final RDXVRFootstepPlacement footstepPlacer;
   private final ControllerStatusTracker controllerStatusTracker;
   private final SwingFootTracker swingFootTracker;
   private RDXVRArmStreaming armStreaming;
   private final SideDependentList<Float> gripButtonsValue = new SideDependentList<>();

   public RDXVRKinematicsStreamingMode(ROS2SyncedRobotModel syncedRobot,
                                       ROS2ControllerHelper ros2ControllerHelper,
                                       RDXVRContext vrContext,
                                       RetargetingParameters retargetingParameters,
                                       SceneGraph sceneGraph,
                                       ControllerStatusTracker controllerStatusTracker,
                                       RDXVRFootstepPlacement footstepPlacer)
   {
      this.syncedRobot = syncedRobot;
      this.robotModel = syncedRobot.getRobotModel();
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.retargetingParameters = retargetingParameters;
      this.sceneGraph = sceneGraph;
      this.vrContext = vrContext;
      this.controllerStatusTracker = controllerStatusTracker;
      this.footstepPlacer = footstepPlacer;
      this.swingFootTracker = new SwingFootTracker(syncedRobot, controllerStatusTracker);
   }

   public void create(boolean createToolbox, KinematicsStreamingToolboxParameters kstParameters)
   {
      this.kstParameters = kstParameters;

      RobotDefinition ghostRobotDefinition = new RobotDefinition(robotModel.getRobotDefinition());
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
         handDesiredControlFrames.put(side, new MutableReferenceFrame(vrContext.getController(side).getXForwardZUpControllerFrame()));
         Pose3D ikControlFramePose = new Pose3D();
         if (side == RobotSide.LEFT)
         {
            ikControlFramePose.getPosition().set(retargetingParameters.getControlFrameOffsetInBodyFrame(VRTrackedSegmentType.LEFT_HAND));
            ikControlFramePose.getOrientation().set(retargetingParameters.getControlFrameOrientationInBodyFrame(VRTrackedSegmentType.LEFT_HAND));
         }
         else
         {
            ikControlFramePose.getPosition().set(retargetingParameters.getControlFrameOffsetInBodyFrame(VRTrackedSegmentType.RIGHT_HAND));
            ikControlFramePose.getOrientation().set(retargetingParameters.getControlFrameOrientationInBodyFrame(VRTrackedSegmentType.RIGHT_HAND));
         }
         ikControlFramePoses.put(side, ikControlFramePose);
      }
      headsetReferenceFrame = new MutableReferenceFrame(vrContext.getHeadset().getXForwardZUpHeadsetFrame());

      status = ros2ControllerHelper.subscribe(KinematicsStreamingToolboxModule.getOutputStatusTopic(syncedRobot.getRobotModel().getSimpleRobotName()));

      kinematicsRecorder = new KinematicsRecordReplay(sceneGraph, enabled);
      motionRetargeting = new RDXVRMotionRetargeting(syncedRobot, handDesiredControlFrames, trackerReferenceFrames, headsetReferenceFrame, retargetingParameters);
      footstepStreaming = new RDXVRFootstepStreaming(syncedRobot, ros2ControllerHelper, footstepPlacer, swingFootTracker);
      armStreaming = new RDXVRArmStreaming(syncedRobot, ros2ControllerHelper, handDesiredControlFrames, trackerReferenceFrames, ikControlFramePoses);

      if (syncedRobot.getRobotModel().getSimpleRobotName().contains("Nadia"))
      {
         // Message for deactivating the spine pitch and roll joints
         ikSolverConfigurationMessage.getJointsToDeactivate().add(syncedRobot.getFullRobotModel().getSpineJoint(SpineJointName.SPINE_PITCH).hashCode());
         ikSolverConfigurationMessage.getJointsToDeactivate().add(syncedRobot.getFullRobotModel().getSpineJoint(SpineJointName.SPINE_ROLL).hashCode());
      }
      if (createToolbox)
      {
         boolean startYoVariableServer = true;
         toolbox = new KinematicsStreamingToolboxModule(robotModel, kstParameters, startYoVariableServer);
      }

      if (vrContext.getVRModel() == RDXVRHardwareModel.FOCUS3)
      {
         RDXBaseUI.getInstance().getKeyBindings().register("Streaming - Enable IK (toggle)", "A button");
         RDXBaseUI.getInstance().getKeyBindings().register("Streaming - Control robot (toggle)", "X button");
      }
      else
      {
         RDXBaseUI.getInstance().getKeyBindings().register("Streaming - Enable IK (toggle)", "Right A button");
         RDXBaseUI.getInstance().getKeyBindings().register("Streaming - Control robot (toggle)", "Left A button");
      }
   }

   public void processVRInput()
   {
      vrContext.getController(RobotSide.LEFT).runIfConnected(controller ->
      {
         InputDigitalActionData aButton = controller.getAButtonActionData();
         if (aButton.bChanged() && !aButton.bState())
         {
            streamToController.set(!streamToController.get());
            if (!streamToController.get())
               streamingDisabled.set();
//            armStreaming.enable(streamToController.get());
//            armStreaming.enableStreaming(streamToController.get());
         }

         InputDigitalActionData bButton = controller.getBButtonActionData();
         if (bButton.bChanged() && !bButton.bState())
         {
            double trajectoryTime = 1.5;
            GoHomeMessage homePelvis = new GoHomeMessage();
            homePelvis.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_PELVIS);
            homePelvis.setTrajectoryTime(trajectoryTime);
            ros2ControllerHelper.publishToController(homePelvis);

            GoHomeMessage homeChest = new GoHomeMessage();
            homeChest.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_CHEST);
            homeChest.setTrajectoryTime(trajectoryTime);

            RDXBaseUI.pushNotification("Commanding home pose...");
            ros2ControllerHelper.publishToController(homeChest);
         }

         InputDigitalActionData clickTriggerButton = controller.getClickTriggerActionData();
         if (clickTriggerButton.bChanged() && !clickTriggerButton.bState())
         {
         }

         // Check if left joystick is pressed in order to trigger recording or replay of motion
         InputDigitalActionData leftJoystickButton = controller.getJoystickPressActionData();

         kinematicsRecorder.processRecordReplayInput(leftJoystickButton);
         if (kinematicsRecorder.isReplayingEnabled().get())
            wakeUpToolbox();

         if (leftJoystickButton.bChanged() && !leftJoystickButton.bState() &&
             !kinematicsRecorder.isReplayingEnabled().get() && !kinematicsRecorder.isRecordingEnabled().get())
         { // reinitialize toolbox
            LogTools.warn("Reinitializing toolbox. Forcing initial lower-body IK configuration to current robot configuration");
            if (enabled.get())
            {
               reinitializeToolboxRobotConfiguration();
            }
         }

         gripButtonsValue.put(RobotSide.LEFT, controller.getGripActionData().x());
      });

      vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
      {
        InputDigitalActionData aButton = controller.getAButtonActionData();
        if (aButton.bChanged() && !aButton.bState())
        {
           setEnabled(!enabled.get());
        }

        InputDigitalActionData clickTriggerButton = controller.getClickTriggerActionData();
        if (clickTriggerButton.bChanged() && !clickTriggerButton.bState())
        {
        }

         gripButtonsValue.put(RobotSide.RIGHT, controller.getGripActionData().x());
      });

      if ((enabled.get() || kinematicsRecorder.isReplaying()))
      {
         KinematicsStreamingToolboxInputMessage toolboxInputMessage = new KinematicsStreamingToolboxInputMessage();
         processTrackers(toolboxInputMessage);
         processControllers(toolboxInputMessage);
         retargetMotion(toolboxInputMessage);

         if (enabled.get())
            toolboxInputMessage.setStreamToController(streamToController.get());
         else
            toolboxInputMessage.setStreamToController(kinematicsRecorder.isReplaying());

         if (toolboxInputStreamRateLimiter.run(streamPeriod) && !pausedForWalking)
         {
            ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputToolboxConfigurationTopic(syncedRobot.getRobotModel().getSimpleRobotName()), ikSolverConfigurationMessage);
            ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputCommandTopic(syncedRobot.getRobotModel().getSimpleRobotName()), toolboxInputMessage);
            outputFrequencyPlot.recordEvent();
         }

         boolean isStepping = gripButtonsValue.get(RobotSide.LEFT) > 0.2f && gripButtonsValue.get(RobotSide.RIGHT) > 0.2f;
         footstepStreaming.processVRInput(isStepping);
      }
   }

   private void processTrackers(KinematicsStreamingToolboxInputMessage messageToPack)
   {
      additionalTrackedSegments = vrContext.getAssignedTrackerRoles();
      for (VRTrackedSegmentType segmentType : VRTrackedSegmentType.getTrackerTypes())
      {
         if (additionalTrackedSegments.contains(segmentType.getSegmentName()) && !controlArmsOnly.get())
         {
            vrContext.getTracker(segmentType.getSegmentName()).runIfConnected(tracker ->
            {
               if (!trackerReferenceFrames.containsKey(segmentType.getSegmentName()))
               {
                  MutableReferenceFrame trackerDesiredControlFrame = new MutableReferenceFrame(tracker.getXForwardZUpTrackerFrame());
                  trackerDesiredControlFrame.getTransformToParent().getRotation().appendInvertOther(retargetingParameters.getControlFrameOrientationInBodyFrame(segmentType));
                  trackerDesiredControlFrame.getReferenceFrame().update();
                  trackerReferenceFrames.put(segmentType.getSegmentName(), trackerDesiredControlFrame);
                  if (segmentType == CHEST)
                     chestFrameGraphics.setToReferenceFrame(ghostFullRobotModel.getChest().getBodyFixedFrame());

                  if (segmentType.isFootRelated())
                     footstepStreaming.setTrackerReference(segmentType.getSegmentSide(), trackerDesiredControlFrame.getReferenceFrame());
               }

               if (!trackerFrameGraphics.containsKey(segmentType.getSegmentName()))
               {
                  trackerFrameGraphics.put(segmentType.getSegmentName(),
                                           new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
               }
               trackerFrameGraphics.get(segmentType.getSegmentName())
                                   .setToReferenceFrame(trackerReferenceFrames.get(segmentType.getSegmentName()).getReferenceFrame());

               if (segmentType.isFootRelated())
               {
                  footstepStreaming.setTrackerVelocity(segmentType.getSegmentSide(),
                                                       new SpatialVector(ReferenceFrame.getWorldFrame(),
                                                                         tracker.getAngularVelocity(),
                                                                         tracker.getLinearVelocity()));
                  footstepStreaming.setTrackerTimestamp(segmentType.getSegmentSide(), tracker.getLastPollTimeNanos());
               }

               if (motionRetargeting.isRetargetingNotNeeded(segmentType))
               {
                  RigidBodyBasics controlledSegment = getControlledSegment(segmentType);
                  if (controlledSegment != null)
                  {
                     KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(controlledSegment,
                                                                                        trackerReferenceFrames.get(segmentType.getSegmentName()).getReferenceFrame(),
                                                                                        segmentType.getSegmentName(),
                                                                                        retargetingParameters.getPositionWeight(segmentType),
                                                                                        retargetingParameters.getOrientationWeight(segmentType),
                                                                                        retargetingParameters.getLinearRateLimitation(segmentType),
                                                                                        retargetingParameters.getAngularRateLimitation(segmentType));
                     message.setHasDesiredLinearVelocity(true);
                     message.getDesiredLinearVelocityInWorld().set(tracker.getLinearVelocity());
                     message.setHasDesiredAngularVelocity(true);
                     message.getDesiredAngularVelocityInWorld().set(tracker.getAngularVelocity());
                     messageToPack.getInputs().add().set(message);
                  }
               }
            });
         }
      }
   }

   private void processControllers(KinematicsStreamingToolboxInputMessage messageToPack)
   {
      for (VRTrackedSegmentType segmentType : VRTrackedSegmentType.getControllerTypes())
      {
         vrContext.getController(segmentType.getSegmentSide()).runIfConnected(controller ->
         {
            MovingReferenceFrame endEffectorFrame = ghostFullRobotModel.getEndEffectorFrame(segmentType.getSegmentSide(), LimbName.ARM);
            if (endEffectorFrame == null)
               return;
            controller.getXForwardZUpControllerFrame().update();
            controllerFrameGraphics.get(segmentType.getSegmentSide())
                                   .setToReferenceFrame(controller.getXForwardZUpControllerFrame());
            handFrameGraphics.get(segmentType.getSegmentSide()).setToReferenceFrame(endEffectorFrame);
            if (!armScaling.get())
            {
               KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(ghostFullRobotModel.getHand(
                                                                                        segmentType.getSegmentSide()),
                                                                                  handDesiredControlFrames.get(
                                                                                        segmentType.getSegmentSide()).getReferenceFrame(),
                                                                                  segmentType.getSegmentName(),
                                                                                  retargetingParameters.getPositionWeight(segmentType),
                                                                                  retargetingParameters.getOrientationWeight(segmentType),
                                                                                  retargetingParameters.getLinearRateLimitation(segmentType),
                                                                                  retargetingParameters.getAngularRateLimitation(segmentType));
               message.getControlFramePositionInEndEffector().set(ikControlFramePoses.get(segmentType.getSegmentSide()).getPosition());
               message.getControlFrameOrientationInEndEffector().set(ikControlFramePoses.get(segmentType.getSegmentSide()).getOrientation());

               message.setHasDesiredLinearVelocity(true);
               message.getDesiredLinearVelocityInWorld().set(controller.getLinearVelocity());
               message.setHasDesiredAngularVelocity(true);
               message.getDesiredAngularVelocityInWorld().set(controller.getAngularVelocity());

               messageToPack.getInputs().add().set(message);
               messageToPack.setTimestamp(controller.getLastPollTimeNanos());
            }
            else
               controllerLastPollTimeNanos = controller.getLastPollTimeNanos();
         });
      }
   }

   private void retargetMotion(KinematicsStreamingToolboxInputMessage messageToPack)
   {
      if (armScaling.get())
      { // Update headset pose, used for retargeting to estimate shoulder position
         vrContext.getHeadset().runIfConnected(headset -> headset.getXForwardZUpHeadsetFrame().update());
      }
      // Correct values from trackers/controllers using retargeting techniques
      motionRetargeting.computeDesiredValues();
      for (VRTrackedSegmentType segmentType : motionRetargeting.getRetargetedSegments())
      {
         RigidBodyBasics controlledSegment = getControlledSegment(segmentType);
         if (controlledSegment != null)
         {
            KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(controlledSegment,
                                                                               motionRetargeting.getDesiredFrame(segmentType),
                                                                               segmentType.getSegmentName(),
                                                                               retargetingParameters.getPositionWeight(segmentType),
                                                                               retargetingParameters.getOrientationWeight(segmentType),
                                                                               retargetingParameters.getLinearRateLimitation(segmentType),
                                                                               retargetingParameters.getAngularRateLimitation(segmentType));
            // TODO. Linear desired velocities from controller/trackers might be wrong now because of scaling
            if (segmentType.isHandRelated())
            {
               // Check arm scaling state not changed -> disabled
               if (!enabled.get()) return;
               message.getControlFramePositionInEndEffector().set(ikControlFramePoses.get(segmentType.getSegmentSide()).getPosition());
               message.getControlFrameOrientationInEndEffector().set(ikControlFramePoses.get(segmentType.getSegmentSide()).getOrientation());
               messageToPack.setTimestamp(controllerLastPollTimeNanos);
            }
            messageToPack.getInputs().add().set(message);
         }
      }
      if (motionRetargeting.isCenterOfMassAvailable())
      {   // If using ankles and waist tracker, create a CoM message for the toolbox
         KinematicsToolboxCenterOfMassMessage comMessage = new KinematicsToolboxCenterOfMassMessage();
         comMessage.setHasDesiredLinearVelocity(false);
         comMessage.getDesiredPositionInWorld().set(motionRetargeting.getDesiredCenterOfMassXYInWorld());
         comMessage.getSelectionMatrix().setSelectionFrameId(toFrameId(ReferenceFrame.getWorldFrame()));
         comMessage.getSelectionMatrix().setXSelected(true);
         comMessage.getSelectionMatrix().setYSelected(true);
         comMessage.getSelectionMatrix().setZSelected(false);
         comMessage.getWeights().setXWeight(1.0);
         comMessage.getWeights().setYWeight(1.0);

         messageToPack.setUseCenterOfMassInput(true);
         messageToPack.getCenterOfMassInput().set(comMessage);
      }
   }

   private RigidBodyBasics getControlledSegment(VRTrackedSegmentType segmentType)
   {
      return switch (segmentType)
      {
         case LEFT_HAND, RIGHT_HAND -> ghostFullRobotModel.getHand(segmentType.getSegmentSide());
         case LEFT_WRIST, RIGHT_WRIST -> ghostFullRobotModel.getForearm(segmentType.getSegmentSide());
         case CHEST -> ghostFullRobotModel.getChest();
         case WAIST -> ghostFullRobotModel.getPelvis();
         default -> throw new IllegalStateException("Unexpected VR-tracked segment: " + segmentType);
      };
   }

   private KinematicsToolboxRigidBodyMessage createRigidBodyMessage(RigidBodyBasics segment,
                                                                    ReferenceFrame desiredControlFrame,
                                                                    String frameName,
                                                                    Vector3D positionWeight,
                                                                    Vector3D orientationWeight,
                                                                    double linearMomentumLimit,
                                                                    double angularMomentumLimit)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(segment.hashCode());

      tempFramePose.setToZero(desiredControlFrame);
      tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());

      // Record motion if in recording mode
      kinematicsRecorder.framePoseToRecord(tempFramePose, frameName);
      if (kinematicsRecorder.isReplaying())
         kinematicsRecorder.framePoseToPack(tempFramePose); //get values of tempFramePose from replay

      message.getDesiredOrientationInWorld().set(tempFramePose.getOrientation());
      message.getDesiredPositionInWorld().set(tempFramePose.getPosition());

      WeightMatrix3D linearWeightMatrix = new WeightMatrix3D();
      message.getLinearSelectionMatrix().setXSelected(positionWeight.getX() != 0.0);
      message.getLinearSelectionMatrix().setYSelected(positionWeight.getY() != 0.0);
      linearWeightMatrix.setXAxisWeight(positionWeight.getX());
      linearWeightMatrix.setYAxisWeight(positionWeight.getY());
      message.getLinearSelectionMatrix().setZSelected(positionWeight.getZ() != 0.0);
      linearWeightMatrix.setZAxisWeight(positionWeight.getZ());
      message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(linearWeightMatrix));

      WeightMatrix3D angularWeightMatrix = new WeightMatrix3D();
      message.getAngularSelectionMatrix().setXSelected(orientationWeight.getX() != 0.0);
      angularWeightMatrix.setXAxisWeight(orientationWeight.getX());
      message.getAngularSelectionMatrix().setYSelected(orientationWeight.getY() != 0.0);
      angularWeightMatrix.setYAxisWeight(orientationWeight.getY());
      message.getAngularSelectionMatrix().setZSelected(orientationWeight.getZ() != 0.0);
      angularWeightMatrix.setZAxisWeight(orientationWeight.getZ());
      message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(angularWeightMatrix));

      message.setLinearRateLimitation(linearMomentumLimit);
      message.setAngularRateLimitation(angularMomentumLimit);

      return message;
   }

   public void update(boolean ikStreamingModeActive)
   {
      // Safety feature, disable streaming when mode is active
      if (!ikStreamingModeActive)
      {
         streamToController.set(false);
      }
      else // Mode active
      {
         if (!enabled.get()) // Safety feature, if KST is not enabled, then stop streaming
         {
            streamToController.set(false);
         }

         if (enabled.get() || kinematicsRecorder.isReplaying()) // If KST or replay enabled
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
                  // Update IK ghost robot
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

            armStreaming.update();

            swingFootTracker.update();
            footstepStreaming.processToolboxOutput();
            // Stepping with ankle trackers pauses streaming until walking is done
            if (!controllerStatusTracker.isWalking())
            {
               if (footstepStreaming.getReadyToStepNotification().poll())
               {
                  streamingDisabled.clear();
                  if (streamToController.get())
                  {
                     streamingDisabled.set();
                     streamToController.set(false);
//                     armStreaming.enableStreaming(true);
                  }
                  pausedForWalking = true;
                  reintializingToolbox = false;
                  sleepToolbox();
                  visualizeIKPreviewGraphic(false);
                  footstepStreaming.getReadyToStepNotification().clear();
                  LogTools.warn("Stepping from VR");
                  footstepStreaming.step(false);
                  controllerStatusTracker.getFinishedWalkingNotification().clear();
                  //  start controlling only the arms of the robot during walking
//                  armStreaming.enable(true);
               }
            }
            else
            {
               if (footstepStreaming.getReadyToStepNotification().poll())
               {
                  LogTools.warn("Consecutive stepping from VR");
                  reintializingToolbox = false;
                  footstepStreaming.setConsecutiveStepping(true);
                  footstepStreaming.step(false);
                  // This prevents wrong logic. The controller might think we're done walking even if we've just sent a new footstep that needs to propagate to the controller
                  controllerStatusTracker.getFinishedWalkingNotification().clear();
               }
            }
            // Resumes streaming once walking is done
            if (pausedForWalking && controllerStatusTracker.getFinishedWalkingNotification().poll())
            {
               footstepStreaming.setConsecutiveStepping(false);
               reintializingToolbox = true;
               timeNotificationIsDoneWalking = System.nanoTime() / 1e9;
            }
            else if (pausedForWalking && reintializingToolbox && (System.nanoTime() / 1e9 - timeNotificationIsDoneWalking) > 0.3
            )
            {
               // disable arm streaming
               armStreaming.enable(false);
               pausedForWalking = false;
               reinitializeToolboxRobotConfiguration();
               visualizeIKPreviewGraphic(true);
               LogTools.warn("Finished walking. Re-enabling standard KST");
               if (streamingDisabled.poll())
               {
                  LogTools.warn("Finished walking. Resuming streaming");
                  streamToController.set(true);
               }
               reintializingToolbox = false;
            }
         }
      }
   }

   private void teleportToRobot()
   {
      ReferenceFrame robotVRHomeReferenceFrame = RDXVRTeleporter.getRobotVRHomeFrame();
      RigidBodyTransform xyYawHeadsetToTeleportTransform = new RigidBodyTransform();
      vrContext.teleport(teleportIHMCZUpToIHMCZUpWorld ->
       {
          xyYawHeadsetToTeleportTransform.setIdentity();
          vrContext.getHeadset().runIfConnected(headset ->
            {
               headset.getXForwardZUpHeadsetFrame().getTransformToDesiredFrame(xyYawHeadsetToTeleportTransform, vrContext.getTeleportFrameIHMCZUp());
               xyYawHeadsetToTeleportTransform.getRotation().setYawPitchRoll(xyYawHeadsetToTeleportTransform.getRotation().getYaw(), 0.0, 0.0);
            });
          teleportIHMCZUpToIHMCZUpWorld.set(xyYawHeadsetToTeleportTransform);
          teleportIHMCZUpToIHMCZUpWorld.invert();

          RigidBodyTransform vrHomeFramePlanarTransformToWorld = new RigidBodyTransform(robotVRHomeReferenceFrame.getTransformToWorldFrame());
          vrHomeFramePlanarTransformToWorld.getRotation().setYawPitchRoll(vrHomeFramePlanarTransformToWorld.getRotation().getYaw(), 0.0, 0.0);
          RigidBodyTransform tempTransform = new RigidBodyTransform();
          tempTransform.set(vrHomeFramePlanarTransformToWorld);
          tempTransform.transform(teleportIHMCZUpToIHMCZUpWorld);
       });
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.checkbox(labels.get("Control/Stop Robot"), streamToController))
      {
         if (!streamToController.get())
            streamingDisabled.set();
      }

      if (ImGui.checkbox(labels.get("Kinematics streaming"), enabled))
      {
         setEnabled(enabled.get());
      }
      if (ImGui.checkbox(labels.get("Control only arms"), controlArmsOnly))
      {
         streamToController.set(false);
         KinematicsStreamingToolboxConfigurationMessage newConfiguration = kstParameters.getDefaultConfiguration();
         newConfiguration.setLockPelvis(controlArmsOnly.get());
         newConfiguration.setLockChest(controlArmsOnly.get());
         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStreamingConfigurationTopic(syncedRobot.getRobotModel().getSimpleRobotName()), newConfiguration);
      }

      Set<String> connectedTrackers = vrContext.getAssignedTrackerRoles();
      if (connectedTrackers.contains(CHEST.getSegmentName()))
      {
         if (ImGui.checkbox(labels.get("Arm Scaling"), armScaling))
         {
            setEnabled(false);
         }
      }
      else if (armScaling.get())
      {
         armScaling.set(false);
      }

      if (connectedTrackers.contains(WAIST.getSegmentName()) &&
          connectedTrackers.contains(LEFT_ANKLE.getSegmentName()) &&
          connectedTrackers.contains(RIGHT_ANKLE.getSegmentName()))
      {
         if (ImGui.checkbox(labels.get("CoM Tracking"), comTracking))
         {
            setEnabled(false);
         }
      }
      else if (comTracking.get())
      {
         comTracking.set(false);
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
      if (enabled)
      {
         if (!this.enabled.get()) // It was disabled
            wakeUpToolbox();
         initialize();
      }
      else // Disable
      {
         streamingDisabled.clear();
         sleepToolbox();
         footstepStreaming.reset();
         pausedForWalking = false;
         reintializingToolbox = false;

         visualizeIKPreviewGraphic(true);
         streamToController.set(false);
      }

      if (enabled != this.enabled.get())
         this.enabled.set(enabled);
   }

   private void initialize()
   {
      kinematicsRecorder.setReplay(false); // Check no concurrency replay and streaming
      trackerReferenceFrames.clear();
      if (!pausedForWalking)
         footstepStreaming.reset();
      motionRetargeting.reset();
      motionRetargeting.setControlArmsOnly(controlArmsOnly.get());
      motionRetargeting.setArmScaling(armScaling.get());
      motionRetargeting.setCoMTracking(comTracking.get());
   }

   private void reinitializeToolboxRobotConfiguration()
   {
      sleepToolbox();
      // Update initial configuration of KST
      KinematicsStreamingToolboxInitialConfigurationMessage initialConfigMessage = KinematicsToolboxMessageFactory.initialConfigurationFromFullRobotModel(syncedRobot.getFullRobotModel());
      List<OneDoFJointBasics> oneDoFJoints = Arrays.asList(syncedRobot.getFullRobotModel().getOneDoFJoints());
      for (RobotSide robotSide : RobotSide.values)
      {
         List<ArmJointName> armJointNames = Arrays.asList(ArmJointName.SHOULDER_PITCH,
                                                          ArmJointName.SHOULDER_ROLL,
                                                          ArmJointName.SHOULDER_YAW,
                                                          ArmJointName.ELBOW_PITCH,
                                                          ArmJointName.WRIST_YAW,
                                                          ArmJointName.WRIST_ROLL,
                                                          ArmJointName.GRIPPER_YAW);
         List<Integer> armIndices = armJointNames.stream()
                                                 .map(jointName -> oneDoFJoints.indexOf(syncedRobot.getFullRobotModel().getArmJoint(robotSide, jointName)))
                                                 .toList();
         for (int i = 0; i < armJointNames.size(); i++)
         {
            if (armIndices.get(i) != -1)
            {
               initialConfigMessage.getInitialJointAngles().set(armIndices.get(i), retargetingParameters.getArmHomePoint(robotSide, armJointNames.get(i)));
            }
         }
      }
      ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStreamingInitialConfigurationTopic(syncedRobot.getRobotModel()
                                                                                                                          .getSimpleRobotName()),
                                                                                                                           initialConfigMessage);
      reinitializeToolbox();
      wakeUpToolbox();
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
      armStreaming.getRenderables(renderables, pool);

      if (showReferenceFrameGraphics.get())
      {
         for (RobotSide side : RobotSide.values)
         {
            controllerFrameGraphics.get(side).getRenderables(renderables, pool);
            handFrameGraphics.get(side).getRenderables(renderables, pool);
            if (armScaling.get())
            {
               motionRetargeting.getShoulderGraphic(side).getRenderables(renderables, pool);
               motionRetargeting.getScaledHandGraphic(side).getRenderables(renderables, pool);
            }
         }

         for (var trackerGraphics : trackerFrameGraphics.entrySet())
            trackerGraphics.getValue().getRenderables(renderables, pool);

         chestFrameGraphics.getRenderables(renderables, pool);
      }
   }

   public boolean isStreaming()
   {
      return streamToController.get();
   }

   public boolean isEnabled()
   {
      return enabled.get();
   }

   public Notification getStreamingDisabledNotification()
   {
      return streamingDisabled;
   }

   public void visualizeIKPreviewGraphic(boolean visualize)
   {
      ghostRobotGraphic.setActive(visualize);
   }

   public void destroy()
   {
      if (toolbox != null)
         toolbox.closeAndDispose();
      if (footstepStreaming != null)
      {
         footstepStreaming.destroy();
      }
      ghostRobotGraphic.destroy();
      for (RobotSide side : RobotSide.values)
      {
         controllerFrameGraphics.get(side).dispose();
         handFrameGraphics.get(side).dispose();
      }
   }
}