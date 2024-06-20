package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.lwjgl.openvr.InputDigitalActionData;
import toolbox_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.DeprecatedAPIs;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.motionRetargeting.RetargetingParameters;
import us.ihmc.motionRetargeting.VRTrackedSegmentType;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.imgui.ImGuiFrequencyPlot;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXManualFootstepPlacement;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.tools.KinematicsRecordReplay;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRControllerModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

import javax.annotation.Nullable;
import java.util.*;

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
   private MutableReferenceFrame headsetReferenceFrame;
   private final ImBoolean showReferenceFrameGraphics = new ImBoolean(false);
   private final ImBoolean streamToController = new ImBoolean(false);
   private final Notification streamingDisabled = new Notification();
   private final Throttler messageThrottler = new Throttler();
   private KinematicsRecordReplay kinematicsRecorder;
   private final SceneGraph sceneGraph;
   private final RDXVRContext vrContext;
   private final ControllerStatusTracker controllerStatusTracker;
   private final RDXManualFootstepPlacement footstepPlacer;
   private boolean pausedForWalking = false;
   private final SideDependentList<Float> gripButtonsValue = new SideDependentList<>();
   private boolean streamingFootstepEnabled = false;
   @Nullable
   private KinematicsStreamingToolboxModule toolbox;
   private final KinematicsToolboxConfigurationMessage ikSolverConfigurationMessage = new KinematicsToolboxConfigurationMessage();

   private final ImBoolean controlArmsOnly = new ImBoolean(false);
   private final ImBoolean armScaling = new ImBoolean(false);
   private final ImBoolean comTracking = new ImBoolean(false);
   private ReferenceFrame initialPelvisFrame;
   private final RigidBodyTransform initialPelvisTransformToWorld = new RigidBodyTransform();
   private ReferenceFrame initialChestFrame;
   private final RigidBodyTransform initialChestTransformToWorld = new RigidBodyTransform();
   private RDXVRMotionRetargeting motionRetargeting;
   private RDXVRPrescientFootstepStreaming prescientFootstepStreaming;
   private long lastStepCompletionTime;
   private boolean reintializingToolbox = false;
   private long pausedStreamingTime;

   private final HandConfiguration[] handConfigurations = {HandConfiguration.HALF_CLOSE, HandConfiguration.CRUSH, HandConfiguration.CLOSE};
   private int leftIndex = -1;
   private int rightIndex = -1;

   public RDXVRKinematicsStreamingMode(ROS2SyncedRobotModel syncedRobot,
                                       ROS2ControllerHelper ros2ControllerHelper,
                                       RDXVRContext vrContext,
                                       RetargetingParameters retargetingParameters,
                                       SceneGraph sceneGraph,
                                       ControllerStatusTracker controllerStatusTracker,
                                       RDXManualFootstepPlacement footstepPlacer)
   {
      this.syncedRobot = syncedRobot;
      this.robotModel = syncedRobot.getRobotModel();
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.retargetingParameters = retargetingParameters;
      this.sceneGraph = sceneGraph;
      this.vrContext = vrContext;
      this.controllerStatusTracker = controllerStatusTracker;
      this.footstepPlacer = footstepPlacer;
   }

   public void create(boolean createToolbox)
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
         handDesiredControlFrames.put(side, new MutableReferenceFrame(vrContext.getController(side).getXForwardZUpControllerFrame()));
         Pose3D ikControlFramePose = new Pose3D();
         if (side == RobotSide.LEFT)
         {
            ikControlFramePose.getPosition().setAndNegate(retargetingParameters.getTranslationFromTracker(VRTrackedSegmentType.LEFT_HAND));
            ikControlFramePose.getOrientation().setAndInvert(retargetingParameters.getYawPitchRollFromTracker(VRTrackedSegmentType.LEFT_HAND));
         }
         else
         {
            ikControlFramePose.getPosition().setAndNegate(retargetingParameters.getTranslationFromTracker(VRTrackedSegmentType.RIGHT_HAND));
            ikControlFramePose.getOrientation().setAndInvert(retargetingParameters.getYawPitchRollFromTracker(VRTrackedSegmentType.RIGHT_HAND));
         }
         ikControlFramePoses.put(side, ikControlFramePose);
      }
      headsetReferenceFrame = new MutableReferenceFrame(vrContext.getHeadset().getXForwardZUpHeadsetFrame());

      status = ros2ControllerHelper.subscribe(KinematicsStreamingToolboxModule.getOutputStatusTopic(syncedRobot.getRobotModel().getSimpleRobotName()));

      kinematicsRecorder = new KinematicsRecordReplay(sceneGraph, enabled);
      motionRetargeting = new RDXVRMotionRetargeting(syncedRobot, handDesiredControlFrames, trackerReferenceFrames, headsetReferenceFrame, retargetingParameters);
      prescientFootstepStreaming = new RDXVRPrescientFootstepStreaming(syncedRobot, footstepPlacer);

      // Message for deactivating the spine pitch and roll joints
      ikSolverConfigurationMessage.getJointsToDeactivate().add(syncedRobot.getFullRobotModel().getSpineJoint(SpineJointName.SPINE_PITCH).hashCode());
      ikSolverConfigurationMessage.getJointsToDeactivate().add(syncedRobot.getFullRobotModel().getSpineJoint(SpineJointName.SPINE_ROLL).hashCode());

      if (createToolbox)
      {
         KinematicsStreamingToolboxParameters parameters = new KinematicsStreamingToolboxParameters();
         parameters.setDefault();
         parameters.setToolboxUpdatePeriod(0.003);
         parameters.setPublishingPeriod(0.006); // Publishing period in seconds.
         boolean usingRealtimePlugin = false;
         parameters.setStreamIntegrationDuration(usingRealtimePlugin ? 2.0 * parameters.getPublishingPeriod() : 0.1);
         parameters.setDefaultChestMessageAngularWeight(1.0, 1.0, 0.5);
         parameters.setDefaultPelvisMessageLinearWeight(10.0, 10.0, 20.0);
         parameters.setDefaultLinearRateLimit(10.0);
         parameters.setDefaultAngularRateLimit(100.0);
         parameters.setDefaultLinearWeight(10.0);
         parameters.setDefaultAngularWeight(0.005); // TODO This is tuned for the 4-DoF arms. We want to relax the orientation tracking which we don't have good control over.
         parameters.setInputPoseLPFBreakFrequency(15.0);
         parameters.setInputPoseCorrectionDuration(0.05); // Need to send inputs at 30Hz.
         parameters.setInputVelocityRawAlpha(0.65); // TODO This prob can be 1.0, afraid of overshoots.
         parameters.setInputStateEstimatorType(KinematicsStreamingToolboxParameters.InputStateEstimatorType.FBC_STYLE);
         parameters.setUseBBXInputFilter(true);
         parameters.setInputBBXFilterSize(2.0, 2.8, 2.6);
         parameters.setInputBBXFilterCenter(0.4, 0.0, 1.25);
         parameters.setOutputLPFBreakFrequency(10.0);
         parameters.setOutputJointVelocityScale(0.65);

         parameters.setMinimizeAngularMomentum(true);
         parameters.setMinimizeLinearMomentum(true);
         parameters.setAngularMomentumWeight(0.20);
         // TODO should prob be something like 0.01, 0.25 makes it feels like it's moving through mud, the pelvis height mainly won't move fast up/down cuz it generates too much momentum.
         parameters.setLinearMomentumWeight(0.01);

         parameters.setMinimizeAngularMomentumRate(true);
         parameters.setMinimizeLinearMomentumRate(true);
         parameters.setAngularMomentumRateWeight(1.0);
         parameters.setLinearMomentumRateWeight(1.0);

         parameters.getDefaultConfiguration().setEnableLeftHandTaskspace(false);
         parameters.getDefaultConfiguration().setEnableRightHandTaskspace(false);
         parameters.getDefaultConfiguration().setEnableNeckJointspace(false);
         parameters.getDefaultSolverConfiguration().setJointVelocityWeight(0.05);
         parameters.getDefaultSolverConfiguration().setJointAccelerationWeight(0.0); // As soon as we increase this guy, we inject springy behavior.

         parameters.getDefaultSolverConfiguration().setEnableJointVelocityLimits(false);

         if (robotModel != null)
         {
            Map<String, Double> jointUpperLimits = new LinkedHashMap<>();
            for (RobotSide robotSide : RobotSide.values)
               jointUpperLimits.put(robotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), -0.10);
            parameters.setJointCustomPositionUpperLimits(jointUpperLimits);
            parameters.setInitialConfigurationMap(createInitialConfiguration(robotModel));
         }

         parameters.setUseStreamingPublisher(Boolean.parseBoolean(System.getProperty("use.streaming.publisher", "true")));

         boolean startYoVariableServer = true;
         toolbox = new KinematicsStreamingToolboxModule(robotModel, parameters, startYoVariableServer, DomainFactory.PubSubImplementation.FAST_RTPS);
      }

      if (vrContext.getControllerModel() == RDXVRControllerModel.FOCUS3)
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

   private Map<String, Double> createInitialConfiguration(DRCRobotModel robotModel)
   {
      Map<String, Double> initialConfigurationMap = new HashMap<>();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      for (OneDoFJointBasics joint : fullRobotModel.getOneDoFJoints())
      {
         String jointName = joint.getName();
         double q = syncedRobot.getFullRobotModel().getOneDoFJointByName(jointName).getQ();
         initialConfigurationMap.put(jointName, q);
      }

      return initialConfigurationMap;
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

         gripButtonsValue.put(RobotSide.LEFT, controller.getGripActionData().x());

         InputDigitalActionData leftJoystickButton = controller.getJoystickPressActionData();
         if (leftJoystickButton.bChanged() && !leftJoystickButton.bState())
         { // reinitialize toolbox
            LogTools.warn("Reinitializing toolbox. Forcing intial IK configuration to current robot configuration");
            if (enabled.get())
            {
               sleepToolbox();
               // Update initial configuration of KST
               KinematicsToolboxInitialConfigurationMessage initialConfigMessage = KinematicsToolboxMessageFactory.initialConfigurationFromFullRobotModel(
                       syncedRobot.getFullRobotModel());
               ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStreamingInitialConfigurationTopic(syncedRobot.getRobotModel()
                       .getSimpleRobotName()), initialConfigMessage);
               wakeUpToolbox();
               reinitializeToolbox();
            }
         }
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
        { // do not want to close grippers while interacting with the panel
           HandConfiguration handConfiguration = nextHandConfiguration(RobotSide.RIGHT);
           sendHandCommand(RobotSide.RIGHT, handConfiguration);
        }

         gripButtonsValue.put(RobotSide.RIGHT, controller.getGripActionData().x());
      });

      if (gripButtonsValue.get(RobotSide.LEFT) > 0.5f && gripButtonsValue.get(RobotSide.RIGHT) > 0.5f)
      {
         streamingFootstepEnabled = true;
      }
      else
      {
         streamingFootstepEnabled = false;
         pausedForWalking = false;
      }

      if ((enabled.get() || kinematicsRecorder.isReplaying()) && toolboxInputStreamRateLimiter.run(streamPeriod))
      {
         KinematicsStreamingToolboxInputMessage toolboxInputMessage = new KinematicsStreamingToolboxInputMessage();

         // ----------  VR Trackers ------------
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
                     trackerDesiredControlFrame.getTransformToParent().appendOrientation(retargetingParameters.getYawPitchRollFromTracker(segmentType));
                     trackerDesiredControlFrame.getReferenceFrame().update();
                     trackerReferenceFrames.put(segmentType.getSegmentName(), trackerDesiredControlFrame);
                  }
                  if (!trackerFrameGraphics.containsKey(segmentType.getSegmentName()))
                  {
                     trackerFrameGraphics.put(segmentType.getSegmentName(),
                                              new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
                  }
                  trackerFrameGraphics.get(segmentType.getSegmentName())
                                      .setToReferenceFrame(trackerReferenceFrames.get(segmentType.getSegmentName()).getReferenceFrame());
                  if (motionRetargeting.isRetargetingNotNeeded(segmentType))
                  {
                     RigidBodyBasics controlledSegment = getControlledSegment(segmentType);
                     if (controlledSegment != null)
                     {
                        KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(controlledSegment,
                                                                                           trackerReferenceFrames.get(segmentType.getSegmentName()).getReferenceFrame(),
                                                                                           segmentType.getSegmentName(),
                                                                                           segmentType.getPositionWeight(),
                                                                                           segmentType.getOrientationWeight(),
                                                                                           segmentType.getLinearRateLimitation(),
                                                                                           segmentType.getAngularRateLimitation());
                        message.setHasDesiredLinearVelocity(true);
                        message.getDesiredLinearVelocityInWorld().set(tracker.getLinearVelocity());
                        message.setHasDesiredAngularVelocity(true);
                        message.getDesiredAngularVelocityInWorld().set(tracker.getAngularVelocity());
                        toolboxInputMessage.getInputs().add().set(message);
                        // TODO. figure out how we can set this correctly after retargeting computation, or if we even need it
                        //toolboxInputMessage.setTimestamp(tracker.getLastPollTimeNanos());
                     }
                  }
               });
               if (segmentType.isFootRelated())
               {
                  prescientFootstepStreaming.setTrackerReference(segmentType.getSegmentSide(), trackerReferenceFrames.get(segmentType.getSegmentName()).getReferenceFrame());
               }
            }
         }
         // ---------- end VR Trackers ------------

         // ----------  VR Controllers ------------
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
                                                                                     segmentType.getPositionWeight(),
                                                                                     segmentType.getOrientationWeight(),
                                                                                     segmentType.getLinearRateLimitation(),
                                                                                     segmentType.getAngularRateLimitation());
                  message.getControlFramePositionInEndEffector().set(ikControlFramePoses.get(segmentType.getSegmentSide()).getPosition());
                  message.getControlFrameOrientationInEndEffector().set(ikControlFramePoses.get(segmentType.getSegmentSide()).getOrientation());

                  message.setHasDesiredLinearVelocity(true);
                  message.getDesiredLinearVelocityInWorld().set(controller.getLinearVelocity());
                  message.setHasDesiredAngularVelocity(true);
                  message.getDesiredAngularVelocityInWorld().set(controller.getAngularVelocity());

                  toolboxInputMessage.getInputs().add().set(message);
                  toolboxInputMessage.setTimestamp(controller.getLastPollTimeNanos());
               }
               else
                  controllerLastPollTimeNanos = controller.getLastPollTimeNanos();
            });
         }
         // ---------- end VR Controllers ------------

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
                                                                                  segmentType.getPositionWeight(),
                                                                                  segmentType.getOrientationWeight(),
                                                                                  segmentType.getLinearRateLimitation(),
                                                                                  segmentType.getAngularRateLimitation());
               // TODO. Linear desired velocities from controller/trackers are probably wrong now because of scaling.
               // TODO. Figure out if they are really needed
               if (segmentType.isHandRelated())
               {
                  // Check arm scaling state not changed -> disabled
                  if (!enabled.get()) return;
                  message.getControlFramePositionInEndEffector().set(ikControlFramePoses.get(segmentType.getSegmentSide()).getPosition());
                  message.getControlFrameOrientationInEndEffector().set(ikControlFramePoses.get(segmentType.getSegmentSide()).getOrientation());
                  toolboxInputMessage.setTimestamp(controllerLastPollTimeNanos);
               }
               toolboxInputMessage.getInputs().add().set(message);
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

            toolboxInputMessage.setUseCenterOfMassInput(true);
            toolboxInputMessage.getCenterOfMassInput().set(comMessage);
         }


         if (controlArmsOnly.get())
         { // If option 'Control Arms Only' is active, lock pelvis and chest to current pose
            lockChest(toolboxInputMessage);
            lockPelvis(toolboxInputMessage);
         }
         else if (!additionalTrackedSegments.contains(WAIST.getSegmentName()) && additionalTrackedSegments.contains(CHEST.getSegmentName()))
         { // If using only the chest tracker, lock the pelvis pose to avoid weird poses
            lockPelvis(toolboxInputMessage);
         }

         if (enabled.get())
            toolboxInputMessage.setStreamToController(streamToController.get());
         else
            toolboxInputMessage.setStreamToController(kinematicsRecorder.isReplaying());

         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputToolboxConfigurationTopic(syncedRobot.getRobotModel().getSimpleRobotName()), ikSolverConfigurationMessage);
         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputCommandTopic(syncedRobot.getRobotModel().getSimpleRobotName()), toolboxInputMessage);
         outputFrequencyPlot.recordEvent();


         // Stepping with ankle trackers
         if (streamingFootstepEnabled)
         {
            prescientFootstepStreaming.streamFootsteps();

            // Stepping with ankle trackers pauses Streaming until walking is done
            if (!controllerStatusTracker.isWalking())
            {
               if (pausedForWalking)
               {
                  if (System.currentTimeMillis() - pausedStreamingTime > RDXVRPrescientFootstepStreaming.WAIT_TIME_BEFORE_STEP)
                  {
                     LogTools.info("Stepping");
                     prescientFootstepStreaming.step();
                  }
               }
               if (prescientFootstepStreaming.getReadyToStepNotification().poll())
               {
                  streamToController.set(false);
                  pausedStreamingTime = System.currentTimeMillis();
                  pausedForWalking = true;
                  LogTools.info("Starting to walk. Paused streaming");
                  visualizeIKPreviewGraphic(false);
               }
            }
            else
            {
               if (prescientFootstepStreaming.getReadyToStepNotification().poll())
               {
                  LogTools.info("Stepping");
                  prescientFootstepStreaming.step();
               }
            }
         }
         else
         {
            prescientFootstepStreaming.reset();
         }
      }
   }

   // TODO. There is a KTConfigurationMessage to lock chest/pelvis. Use that message, and probably tune the weight of that message. This is equivalent
   private void lockChest(KinematicsStreamingToolboxInputMessage toolboxInputMessage)
   {
      if (initialChestFrame == null)
      {
         initialChestTransformToWorld.set(syncedRobot.getFullRobotModel().getChest().getBodyFixedFrame().getTransformToWorldFrame());
         initialChestFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                      initialChestTransformToWorld);
      }

      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(ghostFullRobotModel.getChest().hashCode());
      tempFramePose.setToZero(initialChestFrame);
      tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      message.getDesiredOrientationInWorld().set(tempFramePose.getOrientation());
      message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(0));
      message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(10));

      toolboxInputMessage.getInputs().add().set(message);
   }

   private void lockPelvis(KinematicsStreamingToolboxInputMessage toolboxInputMessage)
   {
      if (initialPelvisFrame == null)
      {
         initialPelvisTransformToWorld.set(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame().getTransformToWorldFrame());
         initialPelvisFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                       initialPelvisTransformToWorld);
      }

      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(ghostFullRobotModel.getPelvis().hashCode());
      tempFramePose.setToZero(initialPelvisFrame);
      tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      message.getDesiredPositionInWorld().set(tempFramePose.getPosition());
      message.getDesiredOrientationInWorld().set(tempFramePose.getOrientation());
      message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(50));
      message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(50));

      toolboxInputMessage.getInputs().add().set(message);
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

   public void update(boolean ikStreamingModeEnabled)
   {
      // Safety features!
      if (!ikStreamingModeEnabled)
      {
         streamToController.set(false);
      }
      else
      {
         if (!enabled.get())
         {
            streamToController.set(false);
         }

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
         }

         if (streamingFootstepEnabled)
         {
            // Resumes streaming once walking is done
            if (pausedForWalking && controllerStatusTracker.getFinishedWalkingNotification().poll())
            {
               LogTools.info("Finished walking. Resuming streaming");
               // Restart KST
               sleepToolbox();
               lastStepCompletionTime = System.currentTimeMillis();
               reintializingToolbox = true;
            }
            else if (pausedForWalking && reintializingToolbox)
            {
               // Wait a bit for robot to stabilize on last footsteps
               if (System.currentTimeMillis() - lastStepCompletionTime > RDXVRPrescientFootstepStreaming.WAIT_TIME_AFTER_STEP)
               {
                  setEnabled(true);
                  visualizeIKPreviewGraphic(true);
                  pausedForWalking = false;
                  streamToController.set(true);
               }
            }
         }
      }
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
         setEnabled(false);
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
         if (!this.enabled.get())
            wakeUpToolbox();
         else
         {
            // Update initial configuration of KST
            KinematicsToolboxInitialConfigurationMessage initialConfigMessage = KinematicsToolboxMessageFactory.initialConfigurationFromFullRobotModel(
                    syncedRobot.getFullRobotModel());
            ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStreamingInitialConfigurationTopic(syncedRobot.getRobotModel()
                    .getSimpleRobotName()), initialConfigMessage);
            wakeUpToolbox();
            reinitializeToolbox();
         }
         kinematicsRecorder.setReplay(false); // Check no concurrency replay and streaming
         initialPelvisFrame = null;
         initialChestFrame = null;
         trackerReferenceFrames.clear();
         if (!pausedForWalking)
            prescientFootstepStreaming.reset();
         motionRetargeting.reset();
         motionRetargeting.setControlArmsOnly(controlArmsOnly.get());
         motionRetargeting.setArmScaling(armScaling.get());
         motionRetargeting.setCoMTracking(comTracking.get());
         reintializingToolbox = false;
      }
      else
      {
         streamingDisabled.poll();
         sleepToolbox();
      }

      if (enabled != this.enabled.get())
         this.enabled.set(enabled);
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
            if (armScaling.get())
            {
               motionRetargeting.getShoulderGraphic(side).getRenderables(renderables, pool);
               motionRetargeting.getScaledHandGraphic(side).getRenderables(renderables, pool);
            }
         }

         for (var trackerGraphics : trackerFrameGraphics.entrySet())
            trackerGraphics.getValue().getRenderables(renderables, pool);
      }
   }

   public boolean isStreaming()
   {
      return streamToController.get();
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
      ghostRobotGraphic.destroy();
      for (RobotSide side : RobotSide.values)
      {
         controllerFrameGraphics.get(side).dispose();
         handFrameGraphics.get(side).dispose();
      }
   }

   public void sendHandCommand(RobotSide robotSide, HandConfiguration desiredHandConfiguration)
   {
      ros2ControllerHelper.publish(DeprecatedAPIs::getHandConfigurationTopic,
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