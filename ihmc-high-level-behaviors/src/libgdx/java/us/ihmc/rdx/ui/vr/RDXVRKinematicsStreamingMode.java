package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.HandLoadBearingMessage;
import ihmc_common_msgs.msg.dds.Point2DMessage;
import ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage;
import ihmc_common_msgs.msg.dds.WeightMatrix3DMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.apache.commons.lang.mutable.MutableBoolean;
import org.lwjgl.openvr.InputDigitalActionData;
import toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessagePubSubType;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxInitialConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import toolbox_msgs.msg.dds.ROS2LogMessage;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.avatar.sakeGripper.SakeHandPreset;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.commons.UnitConversions;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.communication.ros2log.ROS2LogRecord;
import us.ihmc.communication.ros2log.ROS2LogSerialization;
import us.ihmc.communication.ros2log.ROS2LoggerRequestedState;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.idl.serializers.extra.AbstractSerializer;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.motionRetargeting.RetargetingParameters;
import us.ihmc.motionRetargeting.VRTrackedSegmentType;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.rdx.imgui.ImGuiFrequencyPlot;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXManualFootstepPlacement;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.rdx.ui.graphics.RDXMultiContactRegionGraphic;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.teleoperation.RDXHandConfigurationManager;
import us.ihmc.rdx.ui.tools.KinematicsRecordReplay;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRHardwareModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.yoVariables.euclid.filters.RateLimitedYoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import javax.annotation.Nullable;
import java.io.IOException;
import java.sql.Ref;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Function;

import static us.ihmc.communication.packets.MessageTools.toFrameId;
import static us.ihmc.communication.ros2log.ROS2LogIOTools.messageKey;
import static us.ihmc.communication.ros2log.ROS2LogIOTools.timestampKey;
import static us.ihmc.motionRetargeting.VRTrackedSegmentType.*;

public class RDXVRKinematicsStreamingMode
{
   public static final double FRAME_AXIS_GRAPHICS_LENGTH = 0.2;
   private static final boolean USE_TRACKER_FOR_COM = false;
   private static final double COM_CONTROL_JOYSTICK_THRESHOLD = 0.7;
   private static final double COM_JOYSTICK_INCREMENT = 4.0e-4;

   public static final Vector3D CONTACT_NORMAL = new Vector3D(-1.0, 0.0, 0.0);
   public static Function<ROS2SyncedRobotModel, ReferenceFrame> CONTACT_FRAME_SUPPLIER = s -> s.getReferenceFrames().getMidFeetZUpFrame(); // ReferenceFrame.getWorldFrame();

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
   private ROS2Input<CapturabilityBasedStatus> capturabilityBasedStatus;
   private final double streamPeriod = UnitConversions.hertzToSeconds(120.0);
   private final Throttler toolboxInputStreamRateLimiter = new Throttler();
   private final ImGuiFrequencyPlot statusFrequencyPlot = new ImGuiFrequencyPlot();
   private final ImGuiFrequencyPlot outputFrequencyPlot = new ImGuiFrequencyPlot();
   private final SideDependentList<MutableReferenceFrame> handDesiredControlFrames = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> controllerFrameGraphics = new SideDependentList<>();
   private final SideDependentList<Pose3D> ikControlFramePoses = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> handFrameGraphics = new SideDependentList<>();
   private Set<String> additionalTrackedSegments = new HashSet<>();
   private final EnumMap<VRTrackedSegmentType, PoseReferenceFrame> trackerReferenceFrames = new EnumMap<>(VRTrackedSegmentType.class);
   private final EnumMap<VRTrackedSegmentType, FrameVector3D> trackerAngularVelocity = new EnumMap<>(VRTrackedSegmentType.class);
   private final EnumMap<VRTrackedSegmentType, FrameVector3D> trackerLinearVelocity = new EnumMap<>(VRTrackedSegmentType.class);
   private final EnumMap<VRTrackedSegmentType, RDXReferenceFrameGraphic> trackerFrameGraphics = new EnumMap<>(VRTrackedSegmentType.class);
   private final FrameVector3D trackerToCoM = new FrameVector3D();
   private final AtomicBoolean snapTrackerControlFrames = new AtomicBoolean();
   private MutableReferenceFrame headsetReferenceFrame;
   private final ImBoolean showReferenceFrameGraphics = new ImBoolean(false);
   private final ImBoolean streamToController = new ImBoolean(false);
   private final AtomicBoolean requestRecordReplay = new AtomicBoolean(false);
   private final Notification streamingDisabled = new Notification();
   private final Throttler messageThrottler = new Throttler();
   private KinematicsRecordReplay kinematicsRecorder;
   private ROS2Publisher<ROS2LogMessage> ros2LogMessagePublisher;
   private final SceneGraph sceneGraph;
   private final RDXVRContext vrContext;
   private final ControllerStatusTracker controllerStatusTracker;
   private final RDXHandConfigurationManager handManager;
   private boolean pausedForWalking = false;
   private final SideDependentList<Float> gripButtonsValue = new SideDependentList<>();
   @Nullable
   private KinematicsStreamingToolboxModule toolbox;
   private final KinematicsToolboxConfigurationMessage ikSolverConfigurationMessage = new KinematicsToolboxConfigurationMessage();

   public long previousControllerLastPollTimeNanos = -1;
   public long controllerLastPollTimeNanos;

   private final FramePoint3D comPositionInitial = new FramePoint3D();
   private final FrameVector3D comTrackerOffset = new FrameVector3D();
   private final RateLimitedYoFrameVector3D comTrackerOffsetRL = new RateLimitedYoFrameVector3D("", "", new YoRegistry(getClass().getSimpleName()), 10.0, streamPeriod, ReferenceFrame.getWorldFrame());
   private double leftForwardJoystick;
   private double leftLateralJoystick;
   private double rightForwardJoystick;
   private double rightLateralJoystick;
   private boolean isUpperBodyLoadBearing = false;

   private final ImBoolean controlArmsOnly = new ImBoolean(false);
   private final ImBoolean enableStabilityObjective = new ImBoolean(false);
   private final ImBoolean enableContactAdjustment = new ImBoolean(false);
   private final ImBoolean recordROS2Log = new ImBoolean(false);

   private final FramePoint3D desiredCoMPositionFiltered = new FramePoint3D();
   private final FramePose3D previousTrackerPose = new FramePose3D();
   private boolean checkTrackerContinuity = false;

   private RDXMultiContactRegionGraphic multiContactStabilityGraphic;
   private final HumanoidKinematicsToolboxConfigurationMessage ikHumanoidSolverConfigurationMessage = new HumanoidKinematicsToolboxConfigurationMessage();
   private final KinematicsStreamingToolboxConfigurationMessage streamingToolboxConfigurationMessage = new KinematicsStreamingToolboxConfigurationMessage();

   private SideDependentList<MutableBoolean> handsAreOpen = new SideDependentList<>(new MutableBoolean(false), new MutableBoolean(false));
   private final SideDependentList<Boolean> handsAreLoaded = new SideDependentList<>(false, false);
   private final SideDependentList<RDXHandControlMode> handControlModes = new SideDependentList<>(RDXHandControlMode.GRIPPER, RDXHandControlMode.GRIPPER);

   private final FramePose3D tempFramePose = new FramePose3D();
   private final FrameVector3D tempFrameVector0 = new FrameVector3D();
   private final FrameVector3D tempFrameVector1 = new FrameVector3D();
   private static final Vector3D zeroVector = new Vector3D();

   public RDXVRKinematicsStreamingMode(ROS2SyncedRobotModel syncedRobot,
                                       ROS2ControllerHelper ros2ControllerHelper,
                                       RDXVRContext vrContext,
                                       RetargetingParameters retargetingParameters,
                                       SceneGraph sceneGraph,
                                       ControllerStatusTracker controllerStatusTracker,
                                       RDXManualFootstepPlacement footstepPlacer,
                                       RDXHandConfigurationManager handManager)
   {
      this.syncedRobot = syncedRobot;
      this.robotModel = syncedRobot.getRobotModel();
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.retargetingParameters = retargetingParameters;
      this.sceneGraph = sceneGraph;
      this.vrContext = vrContext;
      this.controllerStatusTracker = controllerStatusTracker;
      this.handManager = handManager;
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

      multiContactStabilityGraphic = new RDXMultiContactRegionGraphic(ghostFullRobotModel, ros2ControllerHelper.getROS2Node());

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
      capturabilityBasedStatus = ros2ControllerHelper.subscribeToController(CapturabilityBasedStatus.class);

      kinematicsRecorder = new KinematicsRecordReplay(enabled, handDesiredControlFrames);

      ros2LogMessagePublisher = ros2ControllerHelper.getROS2Node().createPublisher(ROS2LogRecord.getROS2LogTopic());
      kinematicsRecorder.setRecordCallback(this::onPlaybackChanged);
      kinematicsRecorder.setReplayCallback(this::onPlaybackChanged);

      if (syncedRobot.getRobotModel().getSimpleRobotName().contains("Nadia"))
      {
         // Message for deactivating the spine pitch and roll joints
         ikSolverConfigurationMessage.getJointsToDeactivate().add(syncedRobot.getFullRobotModel().getSpineJoint(SpineJointName.SPINE_PITCH).hashCode());
         ikSolverConfigurationMessage.getJointsToDeactivate().add(syncedRobot.getFullRobotModel().getSpineJoint(SpineJointName.SPINE_ROLL).hashCode());
      }

      if (createToolbox)
      {
         KinematicsStreamingToolboxParameters parameters = new KinematicsStreamingToolboxParameters();
         parameters.setDefault();
         parameters.setToolboxUpdatePeriod(0.003);
         parameters.setPublishingPeriod(0.006); // Publishing period in seconds.
         boolean usingRealtimePlugin = false;
         parameters.setStreamIntegrationDuration(usingRealtimePlugin ? 2.0 * parameters.getPublishingPeriod() : 0.1);
         parameters.setHoldChestAngularWeight(1.0, 1.0, 0.5);
         parameters.setHoldPelvisLinearWeight(10.0, 10.0, 20.0);
         parameters.setDefaultLinearRateLimit(10.0);
         parameters.setDefaultAngularRateLimit(100.0);
         parameters.setDefaultLinearWeight(10.0);
         parameters.setDefaultAngularWeight(0.005); // TODO This is tuned for the 4-DoF arms. We want to relax the orientation tracking which we don't have good control over.
         parameters.setInputPoseLPFBreakFrequency(15.0);
         parameters.setInputPoseCorrectionDuration(0.05); // Need to send inputs at 30Hz.
         parameters.setInputVelocityRawAlpha(0.65); // TODO This prob can be 1.0, afraid of overshoots.
         parameters.setInputStateEstimatorType(KinematicsStreamingToolboxParameters.InputStateEstimatorType.FBC_STYLE);
         parameters.setUseBBXInputFilter(false);
         parameters.setInputBBXFilterSize(2.0, 2.8, 2.6);
         parameters.setInputBBXFilterCenter(0.4, 0.0, 1.25);
         parameters.setOutputLPFBreakFrequency(10.0);
         parameters.setOutputJointVelocityScale(0.65);

         parameters.setMinimizeAngularMomentum(true);
         parameters.setMinimizeLinearMomentum(false);
         parameters.setAngularMomentumWeight(0.20);
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

         parameters.getDefaultSolverConfiguration().setEnableJointVelocityLimits(true);

         if (robotModel != null)
         {
            reduceElbowJointLimits(parameters, robotModel);
            parameters.setInitialConfigurationMap(createInitialConfiguration(robotModel));
         }

         parameters.setUseStreamingPublisher(Boolean.parseBoolean(System.getProperty("use.streaming.publisher", "true")));

         boolean startYoVariableServer = true;
         toolbox = new KinematicsStreamingToolboxModule(robotModel, parameters, startYoVariableServer);
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

   private void onPlaybackChanged(boolean isStarting)
   {
      if (isStarting)
         openGrippers();

      if (!recordROS2Log.get())
         return;
      ROS2LogMessage message = new ROS2LogMessage();
      message.setRequestedState((isStarting ? ROS2LoggerRequestedState.START : ROS2LoggerRequestedState.FINISH).toByte());
      ros2LogMessagePublisher.publish(message);
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

   private void reduceElbowJointLimits(KinematicsStreamingToolboxParameters parameters, DRCRobotModel robotModel)
   {
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      // reduce limit for elbow to avoid singularity
      Map<String, Double> jointUpperLimits = new LinkedHashMap<>();
      Map<String, Double> jointLowerLimits = new LinkedHashMap<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJointBasics elbowJoint = fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH);
         double upperLimit = elbowJoint.getJointLimitUpper();
         double lowerLimit = elbowJoint.getJointLimitLower();
         double fullyExtendedLimit = Math.abs(upperLimit) < Math.abs(lowerLimit) ? upperLimit : lowerLimit;
         if (fullyExtendedLimit > 0)
         {
            fullyExtendedLimit = -0.10;
            jointUpperLimits.put(robotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), fullyExtendedLimit);
         }
         else
         {
            fullyExtendedLimit = 0.10;
            jointLowerLimits.put(robotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), fullyExtendedLimit);
         }
      }

      parameters.setJointCustomPositionUpperLimits(jointUpperLimits);
      parameters.setJointCustomPositionLowerLimits(jointLowerLimits);
   }

   public void processVRInput()
   {
      if (!toolboxInputStreamRateLimiter.run(streamPeriod))
         return;

      kinematicsRecorder.onUpdateStart();

      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJointBasics fingerJoint = syncedRobot.getFullRobotModel().getOneDoFJointByName(robotSide.getUpperCaseName() + "_GRIPPER_X1");
         if (fingerJoint != null)
         {
            handsAreOpen.get(robotSide).setValue(fingerJoint.getQ() > 0.9);
         }
      }

      // Handle left joystick input
      if (kinematicsRecorder.isReplaying())
      {
         boolean leftAButtonPressed = kinematicsRecorder.getAButtonPressed(RobotSide.LEFT);
         boolean leftBButtonPressed = kinematicsRecorder.getBButtonPressed(RobotSide.LEFT);
         boolean leftTriggerPressed = kinematicsRecorder.getTriggerPressed(RobotSide.LEFT);
         handleLeftControllerJoystickInput(leftAButtonPressed, leftBButtonPressed, leftTriggerPressed, false);
      }
      else
      {
         vrContext.getController(RobotSide.LEFT).runIfConnected(controller ->
                                                                {
                                                                   InputDigitalActionData aButton = controller.getAButtonActionData();
                                                                   InputDigitalActionData bButton = controller.getBButtonActionData();
                                                                   InputDigitalActionData clickTriggerButton = controller.getClickTriggerActionData();
                                                                   InputDigitalActionData leftJoystickButton = controller.getJoystickPressActionData();
                                                                   boolean leftJoystickButtonClicked = leftJoystickButton.bChanged() && !leftJoystickButton.bState();

                                                                   boolean leftAButtonPressed = aButton.bChanged() && !aButton.bState();
                                                                   boolean leftBButtonPressed = bButton.bChanged() && !bButton.bState();
                                                                   boolean leftTriggerPressed = clickTriggerButton.bChanged() && !clickTriggerButton.bState();
                                                                   handleLeftControllerJoystickInput(leftAButtonPressed, leftBButtonPressed, leftTriggerPressed, leftJoystickButtonClicked);

                                                                   leftForwardJoystick = controller.getJoystickActionData().y();
                                                                   leftLateralJoystick = -controller.getJoystickActionData().x();

                                                                   // Check if left joystick is pressed in order to trigger recording or replay of motion
                                                                   gripButtonsValue.put(RobotSide.LEFT, controller.getGripActionData().x());
                                                                   kinematicsRecorder.recordInputData(RobotSide.LEFT, leftAButtonPressed, leftBButtonPressed, leftTriggerPressed, controller.getAngularVelocity(), controller.getLinearVelocity(), getTrajectoryRecordFrame());
                                                                });
      }

      if (requestRecordReplay.getAndSet(false))
      {
         kinematicsRecorder.requestRecordReplay();
      }

      // Handle right joystick input
      if (kinematicsRecorder.isReplaying())
      {
         boolean rightAButtonPressed = kinematicsRecorder.getAButtonPressed(RobotSide.RIGHT);
         boolean rightBButtonPressed = kinematicsRecorder.getBButtonPressed(RobotSide.RIGHT);
         boolean rightTriggerPressed = kinematicsRecorder.getTriggerPressed(RobotSide.RIGHT);
         handleRightControllerJoystickInput(rightAButtonPressed, rightBButtonPressed, rightTriggerPressed);
      }
      else
      {
         vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
                                                                 {
                                                                    InputDigitalActionData aButton = controller.getAButtonActionData();
                                                                    InputDigitalActionData bButton = controller.getBButtonActionData();
                                                                    InputDigitalActionData clickTriggerButton = controller.getClickTriggerActionData();

                                                                    boolean rightAButtonPressed = aButton.bChanged() && !aButton.bState();
                                                                    boolean rightBButtonPressed = bButton.bChanged() && !bButton.bState();
                                                                    boolean rightTriggerPressed = clickTriggerButton.bChanged() && !clickTriggerButton.bState();
                                                                    handleRightControllerJoystickInput(rightAButtonPressed, rightBButtonPressed, rightTriggerPressed);

                                                                    rightForwardJoystick = controller.getJoystickActionData().y();
                                                                    rightLateralJoystick = -controller.getJoystickActionData().x();

                                                                    gripButtonsValue.put(RobotSide.RIGHT, controller.getGripActionData().x());
                                                                    kinematicsRecorder.recordInputData(RobotSide.RIGHT, rightAButtonPressed, rightBButtonPressed, rightTriggerPressed, controller.getAngularVelocity(), controller.getLinearVelocity(), getTrajectoryRecordFrame());
                                                                 });
      }

      // Update tracker poses
      {
         VRTrackedSegmentType segmentType = WAIST;

         PoseReferenceFrame trackerFrame = trackerReferenceFrames.computeIfAbsent(segmentType, k -> new PoseReferenceFrame(segmentType + "ControlFrame", ReferenceFrame.getWorldFrame()));
         FrameVector3D angularVelocity = trackerAngularVelocity.computeIfAbsent(segmentType, k -> new FrameVector3D());
         FrameVector3D linearVelocity = trackerLinearVelocity.computeIfAbsent(segmentType, k -> new FrameVector3D());

         if (kinematicsRecorder.isReplaying())
         {
            // Update from logged frame
            kinematicsRecorder.packLoggedData(segmentType, tempFramePose, angularVelocity, linearVelocity);
            tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
            tempFrameVector0.changeFrame(ReferenceFrame.getWorldFrame());
            tempFrameVector1.changeFrame(ReferenceFrame.getWorldFrame());
            trackerFrame.setPoseAndUpdate(tempFramePose);
         }
         else
         {
            if (USE_TRACKER_FOR_COM)
            {
//               LogTools.info(vrContext.getTrackersRoleMap());

               vrContext.getTracker(segmentType.getSegmentName()).runIfConnected(tracker ->
                                                                                 {
                                                                                    // Update from current tracker pose
                                                                                    FramePose3D trackerPose = new FramePose3D(tracker.getXForwardZUpTrackerFrame());
                                                                                    trackerPose.changeFrame(ReferenceFrame.getWorldFrame());

                                                                                    if (checkTrackerContinuity)
                                                                                    {
                                                                                       if (trackerPose.getPositionDistance(previousTrackerPose) > 0.15)
                                                                                       {
                                                                                          trackerPose.set(previousTrackerPose);
                                                                                       }
                                                                                    }

                                                                                    trackerFrame.setPoseAndUpdate(trackerPose);
                                                                                    angularVelocity.set(tracker.getAngularVelocity());
                                                                                    linearVelocity.set(tracker.getLinearVelocity());
                                                                                    kinematicsRecorder.recordTrackerData(segmentType, trackerFrame, tracker.getAngularVelocity(), tracker.getLinearVelocity(), getTrajectoryRecordFrame());

                                                                                    previousTrackerPose.setIncludingFrame(trackerPose);
                                                                                 });
            }
            else
            {
               comTrackerOffset.changeFrame(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
               if (Math.abs(rightForwardJoystick) > COM_CONTROL_JOYSTICK_THRESHOLD)
                  comTrackerOffset.addX(Math.signum(rightForwardJoystick) * COM_JOYSTICK_INCREMENT);
               if (Math.abs(rightLateralJoystick) > COM_CONTROL_JOYSTICK_THRESHOLD)
                  comTrackerOffset.addY(Math.signum(rightLateralJoystick) * COM_JOYSTICK_INCREMENT);
               if (Math.abs(leftForwardJoystick) > COM_CONTROL_JOYSTICK_THRESHOLD)
                  comTrackerOffset.addZ(Math.signum(leftForwardJoystick) * COM_JOYSTICK_INCREMENT);
               comTrackerOffset.changeFrame(ReferenceFrame.getWorldFrame());
               comTrackerOffsetRL.update(comTrackerOffset);

               FramePoint3D trackerPosition = new FramePoint3D(comPositionInitial);
               trackerPosition.changeFrame(ReferenceFrame.getWorldFrame());
               trackerPosition.add(comTrackerOffsetRL);

               FramePose3D trackerPose = new FramePose3D(trackerPosition, new FrameQuaternion());
               trackerFrame.setPoseAndUpdate(trackerPose);
               angularVelocity.setToZero();
               linearVelocity.setToZero();

               kinematicsRecorder.recordTrackerData(segmentType, trackerFrame, angularVelocity, linearVelocity, getTrajectoryRecordFrame());
            }
         }

         RDXReferenceFrameGraphic frameGraphic = trackerFrameGraphics.computeIfAbsent(segmentType, k -> new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
         frameGraphic.setToReferenceFrame(trackerFrame);
      }

      if (enabled.get())
      {
         KinematicsStreamingToolboxInputMessage toolboxInputMessage = new KinematicsStreamingToolboxInputMessage();
         boolean snapTrackerControlFrames = this.snapTrackerControlFrames.getAndSet(false);

         // ----------  VR Trackers ------------
//         additionalTrackedSegments = vrContext.getAssignedTrackerRoles();
//         for (VRTrackedSegmentType segmentType : VRTrackedSegmentType.TRACKER_TYPES)
//         {
//            if ((!USE_TRACKER_FOR_COM || additionalTrackedSegments.contains(segmentType.getSegmentName())) && !controlArmsOnly.get())
//            {
//               handleTrackerInput(segmentType, toolboxInputMessage, snapTrackerControlFrames, tempFrameVector0, tempFrameVector1);
//            }
//         }

         handleTrackerInput(WAIST, toolboxInputMessage, snapTrackerControlFrames, tempFrameVector0, tempFrameVector1);

         // ---------- end VR Trackers ------------

         // ----------  VR Controllers ------------
         for (VRTrackedSegmentType segmentType : VRTrackedSegmentType.CONTROLLER_TYPES)
         {
            boolean handIsLoaded = (segmentType == LEFT_HAND && handsAreLoaded.get(RobotSide.LEFT)) || (segmentType == RIGHT_HAND && handsAreLoaded.get(RobotSide.RIGHT));
            Vector3D positionWeight = handIsLoaded ? zeroVector : retargetingParameters.getPositionWeight(segmentType);

            if (kinematicsRecorder.isReplaying())
            {
               KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(segmentType,
                                                                                  ghostFullRobotModel.getHand(segmentType.getSegmentSide()),
                                                                                  null,
                                                                                  null,
                                                                                  null,
                                                                                  positionWeight,
                                                                                  retargetingParameters.getOrientationWeight(segmentType),
                                                                                  retargetingParameters.getLinearRateLimitation(segmentType),
                                                                                  retargetingParameters.getAngularRateLimitation(segmentType));

               message.getControlFramePositionInEndEffector().set(ikControlFramePoses.get(segmentType.getSegmentSide()).getPosition());
               message.getControlFrameOrientationInEndEffector().set(ikControlFramePoses.get(segmentType.getSegmentSide()).getOrientation());
               toolboxInputMessage.getInputs().add().set(message);
               controllerLastPollTimeNanos = System.nanoTime();
            }
            else
            {
               vrContext.getController(segmentType.getSegmentSide()).runIfConnected(controller ->
                                                                                    {
                                                                                       MovingReferenceFrame endEffectorFrame = ghostFullRobotModel.getEndEffectorFrame(segmentType.getSegmentSide(), LimbName.ARM);
                                                                                       if (endEffectorFrame == null)
                                                                                          return;

                                                                                       controller.getXForwardZUpControllerFrame().update();
                                                                                       controllerFrameGraphics.get(segmentType.getSegmentSide()).setToReferenceFrame(controller.getXForwardZUpControllerFrame());
                                                                                       handFrameGraphics.get(segmentType.getSegmentSide()).setToReferenceFrame(endEffectorFrame);
                                                                                       KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(
                                                                                             segmentType,
                                                                                             ghostFullRobotModel.getHand(segmentType.getSegmentSide()),
                                                                                             handDesiredControlFrames.get(segmentType.getSegmentSide()).getReferenceFrame(),
                                                                                             controller.getAngularVelocity(),
                                                                                             controller.getLinearVelocity(),
                                                                                             positionWeight,
                                                                                             retargetingParameters.getOrientationWeight(segmentType),
                                                                                             retargetingParameters.getLinearRateLimitation(segmentType),
                                                                                             retargetingParameters.getAngularRateLimitation(segmentType));

                                                                                          message.getControlFramePositionInEndEffector().set(ikControlFramePoses.get(segmentType.getSegmentSide()).getPosition());
                                                                                          message.getControlFrameOrientationInEndEffector().set(ikControlFramePoses.get(segmentType.getSegmentSide()).getOrientation());
                                                                                          toolboxInputMessage.getInputs().add().set(message);

                                                                                          controllerLastPollTimeNanos = Math.max(controllerLastPollTimeNanos, controller.getLastPollTimeNanos());
                                                                                    });
            }
         }
         // ---------- end VR Controllers ------------

         if (enabled.get())
            toolboxInputMessage.setStreamToController(streamToController.get());
         else
            toolboxInputMessage.setStreamToController(kinematicsRecorder.isReplaying());

//         if (controllerLastPollTimeNanos > previousControllerLastPollTimeNanos)
         {
            toolboxInputMessage.setTimestamp(System.nanoTime());
            //         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputToolboxConfigurationTopic(syncedRobot.getRobotModel().getSimpleRobotName()), ikSolverConfigurationMessage);
            ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputCommandTopic(syncedRobot.getRobotModel().getSimpleRobotName()), toolboxInputMessage);

            previousControllerLastPollTimeNanos = controllerLastPollTimeNanos;
         }

         outputFrequencyPlot.recordEvent();
      }

      kinematicsRecorder.onUpdateEnd(getTrajectoryRecordFrame());
   }

   private ReferenceFrame getTrajectoryRecordFrame()
   {
      return syncedRobot.getReferenceFrames().getMidFeetZUpFrame();
   }

   private void handleLeftControllerJoystickInput(boolean leftAButtonPressed, boolean leftBButtonPressed, boolean leftTriggerPressed, boolean leftJoystickButtonClicked)
   {
      if (leftAButtonPressed)
      {
         streamToController.set(!streamToController.get());
         if (!streamToController.get())
            streamingDisabled.set();
      }

      if (leftTriggerPressed)
      {
         performHandAction(RobotSide.LEFT);
      }

      if (leftBButtonPressed)
      {
         ikHumanoidSolverConfigurationMessage.setEnableStabilityObjective(false);
//         ikHumanoidSolverConfigurationMessage.setEnableRegionPreview(false);
         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputToolboxHumanoidConfigurationTopic(syncedRobot.getRobotModel().getSimpleRobotName()), ikHumanoidSolverConfigurationMessage);
      }

      if (leftJoystickButtonClicked)
      {
         kinematicsRecorder.requestRecordReplay();
      }

      boolean isReplaying = kinematicsRecorder.isReplayingEnabled().get();
      boolean isRecording = kinematicsRecorder.isRecordingEnabled().get();

      if (leftJoystickButtonClicked && !isReplaying && !isRecording)
      { // reinitialize toolbox
         LogTools.warn("Reinitializing toolbox. Forcing initial lower-body IK configuration to current robot configuration");
         if (enabled.get())
         {
            sleepToolbox();

            // Update initial configuration of KST
            KinematicsToolboxInitialConfigurationMessage initialConfigMessage = KinematicsToolboxMessageFactory.initialConfigurationFromFullRobotModel(syncedRobot.getFullRobotModel());
            List<OneDoFJointBasics> oneDoFJoints = Arrays.asList(syncedRobot.getFullRobotModel().getOneDoFJoints());
            for (RobotSide robotSide : RobotSide.values)
            {
               int shyIndex = oneDoFJoints.indexOf(syncedRobot.getFullRobotModel().getArmJoint(robotSide, ArmJointName.SHOULDER_PITCH));
               int shxIndex = oneDoFJoints.indexOf(syncedRobot.getFullRobotModel().getArmJoint(robotSide, ArmJointName.SHOULDER_ROLL));
               int shzIndex = oneDoFJoints.indexOf(syncedRobot.getFullRobotModel().getArmJoint(robotSide, ArmJointName.SHOULDER_YAW));
               int elyIndex = oneDoFJoints.indexOf(syncedRobot.getFullRobotModel().getArmJoint(robotSide, ArmJointName.ELBOW_PITCH));

               // TODO extract this initial configuration in robot specific class
               initialConfigMessage.getInitialJointAngles().set(shyIndex, -0.5f);
               initialConfigMessage.getInitialJointAngles().set(shxIndex, robotSide.negateIfRightSide(-0.3f));
               initialConfigMessage.getInitialJointAngles().set(shzIndex, robotSide.negateIfRightSide(-0.5f));
               initialConfigMessage.getInitialJointAngles().set(elyIndex, -2.2f);
               // TODO add also default for wrist joints if they exist
            }

            ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStreamingInitialConfigurationTopic(syncedRobot.getRobotModel()
                                                                                                                                .getSimpleRobotName()), initialConfigMessage);
            wakeUpToolbox();
            reinitializeToolbox();
            wakeUpToolbox();
         }
      }
   }

   private void openGrippers()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (handControlModes.get(robotSide) == RDXHandControlMode.GRIPPER)
         {
            handsAreOpen.get(robotSide).setValue(true);
            handManager.publishHandCommand(robotSide, SakeHandPreset.FULLY_OPEN, false, false);
         }
      }
   }

   private void handleRightControllerJoystickInput(boolean rightAButtonPressed, boolean rightBButtonPressed, boolean rightTriggerPressed)
   {
      if (rightAButtonPressed)
      {
         LogTools.info("Right A Pressed - Enabling");

         if (!enabled.get())
         { // Enabling, configure appropriate settings
            ikHumanoidSolverConfigurationMessage.setHoldCurrentCenterOfMassXyPosition(false);
            ikHumanoidSolverConfigurationMessage.setEnableStabilityObjective(enableStabilityObjective.get());

            // Either way specify the normal if available
            if (multiContactStabilityGraphic.hasRegion())
            {
               PlanarRegion bracingRegion = multiContactStabilityGraphic.getBracingRegion();
               ikHumanoidSolverConfigurationMessage.getRegionNormal().set(bracingRegion.getNormal());
            }

            // Configure region if doing contact adjustment
            boolean enableContactAdjustmentRequested = enableContactAdjustment.get();
            if (enableContactAdjustmentRequested && multiContactStabilityGraphic.hasRegion())
            {
               ikHumanoidSolverConfigurationMessage.setEnableContactAdjustment(true);

               // Region frame
               PlanarRegion bracingRegion = multiContactStabilityGraphic.getBracingRegion();
               FramePose3D bracingRegionPose = new FramePose3D(ReferenceFrame.getWorldFrame(), bracingRegion.getTransformToWorld());
               ikHumanoidSolverConfigurationMessage.getRegionPoint().set(bracingRegionPose.getPosition());
               ikHumanoidSolverConfigurationMessage.getRegionOrientation().set(bracingRegionPose.getOrientation());

               // Convex hull
               Object<Point2DMessage> regionVertices = ikHumanoidSolverConfigurationMessage.getRegionVertices();
               regionVertices.clear();
               ConvexPolygon2D convexHull = bracingRegion.getConvexHull();
               for (int i = 0; i < convexHull.getNumberOfVertices(); i++)
               {
                  Point2DMessage messageVertex = regionVertices.add();
                  messageVertex.setX(convexHull.getVertex(i).getX());
                  messageVertex.setY(convexHull.getVertex(i).getY());
               }
            }
            else
            {
               ikHumanoidSolverConfigurationMessage.setEnableContactAdjustment(false);
            }

            ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputToolboxHumanoidConfigurationTopic(syncedRobot.getRobotModel().getSimpleRobotName()), ikHumanoidSolverConfigurationMessage);

            streamingToolboxConfigurationMessage.setEnableCenterOfMassControl(true);
            ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStreamingConfigurationTopic(syncedRobot.getRobotModel().getSimpleRobotName()), streamingToolboxConfigurationMessage);
         }

         setEnabled(!enabled.get());
         previousControllerLastPollTimeNanos = -1;
      }

//      if (rightBButtonPressed)
//      {
//         LogTools.info("Requesting region preview");
//         ikHumanoidSolverConfigurationMessage.setEnableStabilityObjective(enableStabilityObjective.get());
//         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputToolboxHumanoidConfigurationTopic(syncedRobot.getRobotModel().getSimpleRobotName()), ikHumanoidSolverConfigurationMessage);
//      }

      if (rightTriggerPressed)
      { // do not want to close grippers while interacting with the panel
         performHandAction(RobotSide.RIGHT);
      }
   }

   private void handleTrackerInput(VRTrackedSegmentType segmentType,
                                   KinematicsStreamingToolboxInputMessage toolboxInputMessage,
                                   boolean snapTrackerControlFrames,
                                   Vector3DReadOnly angularVelocity,
                                   Vector3DReadOnly linearVelocity)
   {
      PoseReferenceFrame trackerFrame = trackerReferenceFrames.get(segmentType);
      FramePoint3D trackerPosition = new FramePoint3D(trackerFrame);
      FramePoint3D desiredCenterOfMass = new FramePoint3D(trackerPosition);
      desiredCenterOfMass.changeFrame(ReferenceFrame.getWorldFrame());

      // Filter and do a safety check on the desired CoM position in mid-feed z-up frame
      if (USE_TRACKER_FOR_COM)
      {
         if (snapTrackerControlFrames)
         {
            // Control frame here is used to store the nominal x_com in mid-feet zup frame
            FramePoint3D comPosition = new FramePoint3D(syncedRobot.getReferenceFrames().getCenterOfMassFrame());
            trackerPosition.changeFrame(ReferenceFrame.getWorldFrame());
            comPosition.changeFrame(ReferenceFrame.getWorldFrame());
            trackerToCoM.sub(comPosition, trackerPosition);
         }

         // Compute desired CoM position
         desiredCenterOfMass.add(trackerToCoM);

         if (snapTrackerControlFrames)
         {
            desiredCoMPositionFiltered.setIncludingFrame(desiredCenterOfMass);
         }

         desiredCenterOfMass.changeFrame(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
         desiredCoMPositionFiltered.changeFrame(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());

         double interpolationAlphaXZ = 0.007;
         double interpolationAlphaY = 0.003;

         double minComX = -0.02;
         double maxComX = 0.2;

         double maxComY = 0.05;

         double nominalComZ = 0.88;
         double minComZ = -0.25;
         double maxComZ = 0.04;

         desiredCenterOfMass.setX(EuclidCoreTools.clamp(desiredCenterOfMass.getX(), minComX, maxComX));
         desiredCenterOfMass.setY(EuclidCoreTools.clamp(desiredCenterOfMass.getY(), maxComY));
         desiredCenterOfMass.setZ(nominalComZ + EuclidCoreTools.clamp(desiredCenterOfMass.getZ() - nominalComZ, minComZ, maxComZ));

         desiredCoMPositionFiltered.setX(EuclidCoreTools.interpolate(desiredCoMPositionFiltered.getX(), desiredCenterOfMass.getX(), interpolationAlphaXZ));
         desiredCoMPositionFiltered.setY(EuclidCoreTools.interpolate(desiredCoMPositionFiltered.getY(), desiredCenterOfMass.getY(), interpolationAlphaY));
         desiredCoMPositionFiltered.setZ(EuclidCoreTools.interpolate(desiredCoMPositionFiltered.getZ(), desiredCenterOfMass.getZ(), interpolationAlphaXZ));

         if (!isUpperBodyLoadBearing)
         {
            desiredCoMPositionFiltered.setToZero(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
            desiredCoMPositionFiltered.setX(-0.008);
            desiredCoMPositionFiltered.setZ(0.88);
            desiredCoMPositionFiltered.changeFrame(ReferenceFrame.getWorldFrame());
         }

         desiredCoMPositionFiltered.changeFrame(ReferenceFrame.getWorldFrame());
      }
      else
      {
         if (snapTrackerControlFrames)
         {
            desiredCoMPositionFiltered.setIncludingFrame(desiredCenterOfMass);
         }
         else
         {
            double interpolationAlpha = 0.03;
            desiredCoMPositionFiltered.interpolate(desiredCenterOfMass, interpolationAlpha);
         }
      }

      // Create and pack IK toolbox message
      KinematicsToolboxCenterOfMassMessage comMessage = new KinematicsToolboxCenterOfMassMessage();
      comMessage.getDesiredPositionInWorld().set(desiredCoMPositionFiltered);
      comMessage.getDesiredLinearVelocityInWorld().setToZero();

      comMessage.setHasDesiredLinearVelocity(true);

//      if (snapTrackerControlFrames)
//      {
//         LogTools.info("Initial desired CoM");
//         LogTools.info(desiredCoMPositionFiltered);
//         LogTools.info("Tracker to CoM");
//         LogTools.info(trackerToCoM);
//      }

      comMessage.getSelectionMatrix().setSelectionFrameId(toFrameId(ReferenceFrame.getWorldFrame()));
      comMessage.getSelectionMatrix().setXSelected(true);
      comMessage.getSelectionMatrix().setYSelected(true);
      comMessage.getSelectionMatrix().setZSelected(true);
      comMessage.setHasDesiredLinearVelocity(false);

      double comWeight = 2.0 / ghostFullRobotModel.getTotalMass();
      comMessage.getWeights().setXWeight(comWeight);
      comMessage.getWeights().setYWeight(comWeight);
      comMessage.getWeights().setZWeight(comWeight);

      toolboxInputMessage.setUseCenterOfMassInput(true);
      toolboxInputMessage.getCenterOfMassInput().set(comMessage);
   }

   private KinematicsToolboxRigidBodyMessage createRigidBodyMessage(VRTrackedSegmentType segmentType,
                                                                    RigidBodyBasics segment,
                                                                    ReferenceFrame desiredControlFrame,
                                                                    Vector3DReadOnly angularVelocity,
                                                                    Vector3DReadOnly linearVelocity,
                                                                    Vector3D positionWeight,
                                                                    Vector3D orientationWeight,
                                                                    double linearMomentumLimit,
                                                                    double angularMomentumLimit)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(segment.hashCode());

      boolean hasDesiredAngularVelocity = false;
      boolean hasDesiredLinearVelocity = false;

      if (desiredControlFrame != null)
      {
         tempFramePose.setToZero(desiredControlFrame);
         tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      }

      if (angularVelocity != null)
      {
         tempFrameVector0.set(ReferenceFrame.getWorldFrame(), angularVelocity);
         hasDesiredAngularVelocity = true;
      }
      if (linearVelocity != null)
      {
         tempFrameVector1.set(ReferenceFrame.getWorldFrame(), linearVelocity);
         hasDesiredLinearVelocity = true;
      }

      if (kinematicsRecorder.isReplaying())
      {
         kinematicsRecorder.packLoggedData(segmentType, tempFramePose, tempFrameVector0, tempFrameVector1);
         hasDesiredAngularVelocity = true;
         hasDesiredLinearVelocity = true;
      }

      message.setHasDesiredLinearVelocity(hasDesiredAngularVelocity);
      message.setHasDesiredAngularVelocity(hasDesiredLinearVelocity);

      message.getDesiredOrientationInWorld().set(tempFramePose.getOrientation());
      message.getDesiredPositionInWorld().set(tempFramePose.getPosition());
      message.getDesiredAngularVelocityInWorld().set(tempFrameVector0);
      message.getDesiredLinearVelocityInWorld().set(tempFrameVector1);

      configureWeightAndSelectionMatrices(positionWeight, message.getLinearSelectionMatrix(), message.getLinearWeightMatrix());
      configureWeightAndSelectionMatrices(orientationWeight, message.getAngularSelectionMatrix(), message.getAngularWeightMatrix());

      message.setLinearRateLimitation(linearMomentumLimit);
      message.setAngularRateLimitation(angularMomentumLimit);

      return message;
   }

   private static void configureWeightAndSelectionMatrices(Vector3D weightVector, SelectionMatrix3DMessage selectionMatrixMessage, WeightMatrix3DMessage weightMatrixMessage)
   {
      selectionMatrixMessage.setXSelected(weightVector.getX() != 0.0);
      selectionMatrixMessage.setYSelected(weightVector.getY() != 0.0);
      selectionMatrixMessage.setZSelected(weightVector.getZ() != 0.0);
      weightMatrixMessage.setXWeight(weightVector.getX());
      weightMatrixMessage.setYWeight(weightVector.getY());
      weightMatrixMessage.setZWeight(weightVector.getZ());
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
                  multiContactStabilityGraphic.updateSupportRegionAndGraphics(latestStatus, desiredCoMPositionFiltered);
               }
            }
            if (capturabilityBasedStatus.getMessageNotification().poll())
            {
               CapturabilityBasedStatus capturabilityBasedStatus = this.capturabilityBasedStatus.getMessageNotification().read();
               for (RobotSide robotSide : RobotSide.values)
               {
                  handsAreLoaded.put(robotSide, HumanoidMessageTools.isHandLoadBearing(robotSide, capturabilityBasedStatus));
               }

               isUpperBodyLoadBearing = handsAreLoaded.get(RobotSide.LEFT) || handsAreLoaded.get(RobotSide.RIGHT);
            }
            if (ghostRobotGraphic.isActive())
               ghostRobotGraphic.update();
         }
         else
         { // Only update normal when IK is disabled
            multiContactStabilityGraphic.updateBracingRegionAndGraphics();
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

      ImGui.checkbox(labels.get("Enable stability objective"), enableStabilityObjective);
      ImGui.checkbox(labels.get("Enable contact adjustment"), enableContactAdjustment);
      ImGui.checkbox(labels.get("Record ROS 2 Log"), recordROS2Log);

      if (ImGui.button(labels.get("Start record/replay")))
      {
         requestRecordReplay.set(true);
      }
      if (ImGui.button(labels.get("Load latest csv")))
      {
         kinematicsRecorder.loadLatestReplayFile();
      }

      ghostRobotGraphic.renderImGuiWidgets();
      // add widgets for recording/replaying motion in VR
      ImGui.text("Press Left Joystick - Start/Stop recording");
      kinematicsRecorder.renderRecordWidgets(labels);
      ImGui.text("Press Left Joystick - Start/Stop replay");
      kinematicsRecorder.renderReplayWidgets(labels);
      ImGui.text("Output:");
      ImGui.sameLine();
      outputFrequencyPlot.renderImGuiWidgets();
      ImGui.text("Status:");
      ImGui.sameLine();
      statusFrequencyPlot.renderImGuiWidgets();

      ImGui.checkbox(labels.get("Show reference frames"), showReferenceFrameGraphics);
   }

   public void setEnabled(boolean enable)
   {
      if (enable)
      {
         if (!this.enabled.get())
         {
            wakeUpToolbox();
            snapTrackerControlFrames.set(true);
            comPositionInitial.setToZero(syncedRobot.getReferenceFrames().getCenterOfMassFrame());
            comPositionInitial.changeFrame(ReferenceFrame.getWorldFrame());
            comTrackerOffset.setToZero();
            comTrackerOffsetRL.setToZero();
         }

         kinematicsRecorder.setReplay(false); // Check no concurrency replay and streaming
         trackerReferenceFrames.clear();
      }
      else
      {
         streamingDisabled.poll();
         sleepToolbox();
         visualizeIKPreviewGraphic(true);
         streamToController.set(false);
      }

      if (enable != this.enabled.get())
      {
         this.enabled.set(enable);
      }

      checkTrackerContinuity = enable;
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
      multiContactStabilityGraphic.getRenderables(renderables, pool);

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

   /**
    * Performs hand action based on handControlMode:
    * - GRIPPER: sends a HandConfiguration based on the next entry in handConfigurations.
    * - LOAD_BEARING: loads the hand at the hand control frame with the normal handContactNormalInMidFeetZUpFrame.
    * - NONE: you guessed it, nothing.
    */
   private void performHandAction(RobotSide robotSide)
   {
      if (handControlModes.get(robotSide) == RDXHandControlMode.GRIPPER)
      {
         publishHandCommand(robotSide);
      }
      else if (handControlModes.get(robotSide) == RDXHandControlMode.LOAD_BEARING)
      {
         if (enabled.get())
         {
            LogTools.error("Ignoring hand load bearing message, cannot live-update while IK is running. TODO fixme :)");
         }
         else
         {
            sendHandLoadBearingMessage(robotSide);
         }
      }
   }

   private void sendHandLoadBearingMessage(RobotSide robotSide)
   {
      HandLoadBearingMessage handLoadBearingMessage = new HandLoadBearingMessage();
      handLoadBearingMessage.setRobotSide(robotSide.toByte());

      if (handsAreLoaded.get(robotSide))
      {
         handLoadBearingMessage.setLoad(false);
         handsAreLoaded.put(robotSide, false);

         ikHumanoidSolverConfigurationMessage.setHoldCurrentCenterOfMassXyPosition(false);
         ros2ControllerHelper.publish(ControllerAPI.getTopic(KinematicsStreamingToolboxModule.getInputTopic(robotModel.getSimpleRobotName()), HumanoidKinematicsToolboxConfigurationMessage.class), ikHumanoidSolverConfigurationMessage);
      }
      else
      {
         handLoadBearingMessage.setLoad(true);

         double handCoefficientOfFriction = 0.55;
         handLoadBearingMessage.setCoefficientOfFriction(handCoefficientOfFriction);

         // Contact point assumed to be at hand control frame and is using the nubs
         FramePoint3D contactPoint = new FramePoint3D(syncedRobot.getFullRobotModel().getHandControlFrame(robotSide));
         contactPoint.changeFrame(syncedRobot.getFullRobotModel().getHand(robotSide).getBodyFixedFrame());
         handLoadBearingMessage.getContactPointInBodyFrame().set(contactPoint);

         if (multiContactStabilityGraphic.hasRegion())
         { // From perception
            handLoadBearingMessage.getContactNormalInWorld().set(multiContactStabilityGraphic.getBracingRegion().getNormal());
         }
         else
         { // Fall back on hard-coded
            FrameVector3D contactNormal = new FrameVector3D(CONTACT_FRAME_SUPPLIER.apply(syncedRobot), CONTACT_NORMAL);
            contactNormal.changeFrame(ReferenceFrame.getWorldFrame());
            handLoadBearingMessage.getContactNormalInWorld().set(contactNormal);
         }

         handsAreLoaded.put(robotSide, true);

         ikHumanoidSolverConfigurationMessage.setHoldCurrentCenterOfMassXyPosition(false);
         ros2ControllerHelper.publish(ControllerAPI.getTopic(KinematicsStreamingToolboxModule.getInputTopic(robotModel.getSimpleRobotName()), HumanoidKinematicsToolboxConfigurationMessage.class), ikHumanoidSolverConfigurationMessage);
      }

      LogTools.info("Publishing hand load bearing message " + robotSide + " hand, loading = " + handLoadBearingMessage.getLoad());
      ros2ControllerHelper.publishToController(handLoadBearingMessage);
   }

   public void publishHandCommand(RobotSide side)
   {
      boolean close = handsAreOpen.get(side).booleanValue();
      handsAreOpen.get(side).setValue(!close);
      handManager.publishHandCommand(side, close ? SakeHandPreset.GRIP : SakeHandPreset.FULLY_OPEN, false, false);
   }

   public void setVRHandConfiguration(RDXHandControlMode leftHandControlMode, RDXHandControlMode rightHandControlMode)
   {
      handControlModes.put(RobotSide.LEFT, leftHandControlMode);
      handControlModes.put(RobotSide.RIGHT, rightHandControlMode);
   }

   private void computeSnappedControlFrame(FramePose3DBasics controlFrameToPack, ReferenceFrame trackerFrame, RigidBodyReadOnly bodyToSnapTo)
   {
      controlFrameToPack.setToZero(trackerFrame);
      controlFrameToPack.changeFrame(bodyToSnapTo.getBodyFixedFrame());

      // centers in Y, this direction should be intuitive for the user to line up
//      controlFrameToPack.setY(0.0);
   }

   public static void main(String[] args) throws IOException
   {
      HumanoidKinematicsToolboxConfigurationMessage packet = new HumanoidKinematicsToolboxConfigurationMessage();
      HumanoidKinematicsToolboxConfigurationMessagePubSubType topicDataType = new HumanoidKinematicsToolboxConfigurationMessagePubSubType();

      ObjectMapper objectMapper = new ObjectMapper(new JsonFactory());
      ObjectNode rootNode = objectMapper.createObjectNode();
      AbstractSerializer serializer = new JSONSerializer<>(topicDataType);

      ObjectNode topicObject = rootNode.putObject(packet.getClass().getName());
      ArrayNode timestamps = topicObject.putArray(timestampKey);
      ArrayNode messages = topicObject.putArray(messageKey);

      timestamps.add(0);
      messages.add(serializer.serializeToString(packet));

      objectMapper.writerWithDefaultPrettyPrinter().writeValue(System.out, rootNode);

   }
}