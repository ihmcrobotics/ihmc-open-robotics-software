package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.HandLoadBearingMessage;
import controller_msgs.msg.dds.ObjectCarryMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.apache.commons.lang.mutable.MutableBoolean;
import org.lwjgl.openvr.InputDigitalActionData;
import toolbox_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.avatar.sakeGripper.SakeHandPreset;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
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
import us.ihmc.rdx.ui.graphics.RDXMultiContactRegionGraphic;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.teleoperation.RDXHandConfigurationManager;
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

import static us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule.getInputTopic;
import static us.ihmc.communication.packets.MessageTools.toFrameId;
import static us.ihmc.motionRetargeting.VRTrackedSegmentType.*;

public class RDXVRKinematicsStreamingMode
{
   public static final double FRAME_AXIS_GRAPHICS_LENGTH = 0.2;
   final double COM_CONTROL_JOYSTICK_THRESHOLD = 0.7;
   final double COM_JOYSTICK_INCREMENT = 0.001;
   private static final double OBJECT_MASS = 0.4;

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
   private final ImBoolean replayInMidFeetZUpFrame = new ImBoolean(false);
   private final Notification streamingDisabled = new Notification();
   private final Throttler messageThrottler = new Throttler();
   private KinematicsRecordReplay kinematicsRecorder;
   private final SceneGraph sceneGraph;
   private final RDXVRContext vrContext;
   private final ControllerStatusTracker controllerStatusTracker;
   private final RDXManualFootstepPlacement footstepPlacer;
   private final RDXHandConfigurationManager handManager;
   private boolean pausedForWalking = false;
   private final SideDependentList<Float> gripButtonsValue = new SideDependentList<>();
   @Nullable
   private KinematicsStreamingToolboxModule toolbox;
   private final KinematicsToolboxConfigurationMessage ikSolverConfigurationMessage = new KinematicsToolboxConfigurationMessage();
   private final HumanoidKinematicsToolboxConfigurationMessage ikHumanoidSolverConfigurationMessage = new HumanoidKinematicsToolboxConfigurationMessage();

   private SideDependentList<MutableBoolean> handsAreOpen = new SideDependentList<>(new MutableBoolean(false), new MutableBoolean(false));

   private final ImBoolean controlArmsOnly = new ImBoolean(false);
   private final ImBoolean armScaling = new ImBoolean(false);
   private final ImBoolean comTracking = new ImBoolean(false);
   private final FramePoint3D comJoystickXYInput = new FramePoint3D();
   private final FramePoint3D comJoystickZInput = new FramePoint3D();
   private ReferenceFrame initialPelvisFrame;
   private final RigidBodyTransform initialPelvisTransformToWorld = new RigidBodyTransform();
   private ReferenceFrame initialChestFrame;
   private final RigidBodyTransform initialChestTransformToWorld = new RigidBodyTransform();
   private RDXVRMotionRetargeting motionRetargeting;
   private RDXVRPrescientFootstepStreaming prescientFootstepStreaming;
   private long lastStepCompletionTime;
   private boolean reintializingToolbox = false;
   private long pausedStreamingTime;

   private ROS2Input<CapturabilityBasedStatus> capturabilityBasedStatus;
   private final HandConfiguration[] handConfigurations = {HandConfiguration.HALF_CLOSE, HandConfiguration.CRUSH, HandConfiguration.CLOSE};
   private int leftIndex = -1;
   private int rightIndex = -1;
   private RDXMultiContactRegionGraphic polygonGraphic;

   public static final Vector3D HAND_CONTACT_NORMAL_IN_MID_FEET_ZUP_FRAME = new Vector3D(-1.0, 0.0, 0.0);
   public static final Vector3D HAND_CONTACT_NORMAL_IN_WORLD = new Vector3D();
   private static final double HAND_CONTACT_COEFFICIENT_OF_FRICTION = 0.7; // 0.3;
   private static final boolean CONTROL_LOADED_HAND_ORIENTATION = true;

   private final SideDependentList<Boolean> handsAreLoaded = new SideDependentList<>(false, false);
   private final SideDependentList<HandControlMode> handControlModes = new SideDependentList<>(HandControlMode.GRIPPER, HandControlMode.LOAD_BEARING);

   private enum HandControlMode
   {
      NONE,
      GRIPPER,
      LOAD_BEARING
   }

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
      this.footstepPlacer = footstepPlacer;
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

      polygonGraphic = new RDXMultiContactRegionGraphic(ghostFullRobotModel);

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
      capturabilityBasedStatus = ros2ControllerHelper.subscribeToController(CapturabilityBasedStatus.class);

      kinematicsRecorder = new KinematicsRecordReplay(sceneGraph, enabled, handDesiredControlFrames);
      motionRetargeting = new RDXVRMotionRetargeting(syncedRobot, handDesiredControlFrames, trackerReferenceFrames, headsetReferenceFrame, retargetingParameters);
      prescientFootstepStreaming = new RDXVRPrescientFootstepStreaming(syncedRobot, footstepPlacer);

      // TODO Luigi. remove when Nadia chest link has been replaced and we can remove the fake joints from the urdf
      // Message for deactivating the spine pitch and roll joints
      ikSolverConfigurationMessage.getJointsToDeactivate().add(syncedRobot.getFullRobotModel().getSpineJoint(SpineJointName.SPINE_PITCH).hashCode());
      ikSolverConfigurationMessage.getJointsToDeactivate().add(syncedRobot.getFullRobotModel().getSpineJoint(SpineJointName.SPINE_ROLL).hashCode());

      // Message for deactivating holding CoM XY
      ikHumanoidSolverConfigurationMessage.setHoldCurrentCenterOfMassXyPosition(false);

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

         parameters.setMinimizeAngularMomentum(false);
         parameters.setMinimizeLinearMomentum(false);
         parameters.setAngularMomentumWeight(0.20);
         // TODO should prob be something like 0.01, 0.25 makes it feels like it's moving through mud, the pelvis height mainly won't move fast up/down cuz it generates too much momentum.
         parameters.setLinearMomentumWeight(0.01);

         parameters.setMinimizeAngularMomentumRate(false);
         parameters.setMinimizeLinearMomentumRate(false);
         parameters.setAngularMomentumRateWeight(1.0);
         parameters.setLinearMomentumRateWeight(1.0);

         parameters.getDefaultConfiguration().setEnableLeftHandTaskspace(false);
         parameters.getDefaultConfiguration().setEnableRightHandTaskspace(false);
         parameters.getDefaultConfiguration().setEnableNeckJointspace(false);
         parameters.getDefaultSolverConfiguration().setJointVelocityWeight(0.4);
         parameters.getDefaultSolverConfiguration().setJointAccelerationWeight(0.0); // As soon as we increase this guy, we inject springy behavior.

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
         boolean runPostureOptimizer = true;

         toolbox = new KinematicsStreamingToolboxModule(robotModel, parameters, runPostureOptimizer, startYoVariableServer, DomainFactory.PubSubImplementation.FAST_RTPS);
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
      kinematicsRecorder.onUpdateStart();

      // Handle left joystick input
      if (kinematicsRecorder.isReplaying())
      {
         boolean leftAButtonPressed = kinematicsRecorder.getAButtonPressed(RobotSide.LEFT);
         boolean leftTriggerPressed = kinematicsRecorder.getTriggerPressed(RobotSide.LEFT);
         double lateralJoystickValue = kinematicsRecorder.getLateralJoystickValue(RobotSide.LEFT);
         double forwardJoystickValue = kinematicsRecorder.getForwardJoystickValue(RobotSide.LEFT);
         handleLeftControllJoystickInput(leftAButtonPressed, leftTriggerPressed, lateralJoystickValue, forwardJoystickValue, false);
      }
      else
      {
         vrContext.getController(RobotSide.LEFT).runIfConnected(controller ->
                                                                {
                                                                   InputDigitalActionData aButton = controller.getAButtonActionData();
                                                                   InputDigitalActionData clickTriggerButton = controller.getClickTriggerActionData();
                                                                   InputDigitalActionData leftJoystickButton = controller.getJoystickPressActionData();
                                                                   boolean leftJoystickButtonClicked = leftJoystickButton.bChanged() && !leftJoystickButton.bState();

                                                                   boolean leftAButtonPressed = aButton.bChanged() && !aButton.bState();
                                                                   boolean leftTriggerPressed = clickTriggerButton.bChanged() && !clickTriggerButton.bState();
                                                                   double lateralJoystickValue = controller.getJoystickActionData().x();
                                                                   double forwardJoystickValue = controller.getJoystickActionData().y();
                                                                   handleLeftControllJoystickInput(leftAButtonPressed, leftTriggerPressed, forwardJoystickValue, lateralJoystickValue, leftJoystickButtonClicked);

                                                                   // Check if left joystick is pressed in order to trigger recording or replay of motion
                                                                   gripButtonsValue.put(RobotSide.LEFT, controller.getGripActionData().x());
                                                                   if (replayInMidFeetZUpFrame.get())
                                                                      kinematicsRecorder.recordControllerData(RobotSide.LEFT, leftAButtonPressed, leftTriggerPressed, forwardJoystickValue, lateralJoystickValue, syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
                                                                   else
                                                                      kinematicsRecorder.recordControllerData(RobotSide.LEFT, leftAButtonPressed, leftTriggerPressed, forwardJoystickValue, lateralJoystickValue, ReferenceFrame.getWorldFrame());
         });
      }

      // Handle right joystick input
      if (kinematicsRecorder.isReplaying())
      {
         boolean rightAButtonPressed = kinematicsRecorder.getAButtonPressed(RobotSide.RIGHT);
         boolean rightTriggerPressed = kinematicsRecorder.getTriggerPressed(RobotSide.RIGHT);
         double lateralJoystickValue = kinematicsRecorder.getLateralJoystickValue(RobotSide.RIGHT);
         double forwardJoystickValue = kinematicsRecorder.getForwardJoystickValue(RobotSide.RIGHT);
         handleRightControllJoystickInput(rightAButtonPressed, rightTriggerPressed, forwardJoystickValue, lateralJoystickValue);
      }
      else
      {
         vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
                                                                 {
                                                                    InputDigitalActionData aButton = controller.getAButtonActionData();
                                                                    InputDigitalActionData clickTriggerButton = controller.getClickTriggerActionData();

                                                                    boolean rightAButtonPressed = aButton.bChanged() && !aButton.bState();
                                                                    boolean rightTriggerPressed = clickTriggerButton.bChanged() && !clickTriggerButton.bState();
                                                                    double forwardJoystickValue = controller.getJoystickActionData().y();
                                                                    double lateralJoystickValue = -controller.getJoystickActionData().x();
                                                                    handleRightControllJoystickInput(rightAButtonPressed,
                                                                                                     rightTriggerPressed,
                                                                                                     forwardJoystickValue,
                                                                                                     lateralJoystickValue);

                                                                    gripButtonsValue.put(RobotSide.RIGHT, controller.getGripActionData().x());
                                                                    if (replayInMidFeetZUpFrame.get())
                                                                       kinematicsRecorder.recordControllerData(RobotSide.RIGHT, rightAButtonPressed, rightTriggerPressed, forwardJoystickValue, lateralJoystickValue, syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
                                                                    else
                                                                       kinematicsRecorder.recordControllerData(RobotSide.RIGHT, rightAButtonPressed, rightTriggerPressed, forwardJoystickValue, lateralJoystickValue, ReferenceFrame.getWorldFrame());
                                                                 });
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
                                                                                                                                                             null,
                                                                                                                                                             segmentType.getPositionWeight(),
                                                                                                                                                             segmentType.getOrientationWeight(),
                                                                                                                                                             segmentType.getLinearRateLimitation(),
                                                                                                                                                             segmentType.getAngularRateLimitation(),
                                                                                                                                                             false);

                                                                                          // TODO commenting out because it was overly responsive in sim. Possibly remove for real robot
                                                                                          //                        message.setHasDesiredLinearVelocity(true);
                                                                                          //                        message.setHasDesiredAngularVelocity(true);
                                                                                          //                        message.getDesiredLinearVelocityInWorld().set(tracker.getLinearVelocity());
                                                                                          //                        message.getDesiredAngularVelocityInWorld().set(tracker.getAngularVelocity());

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
            boolean handIsLoaded =
                  (segmentType == LEFT_HAND && handsAreLoaded.get(RobotSide.LEFT)) || (segmentType == RIGHT_HAND && handsAreLoaded.get(RobotSide.RIGHT));

            if (handIsLoaded && !CONTROL_LOADED_HAND_ORIENTATION)
               continue;

            if (kinematicsRecorder.isReplaying())
            {
               KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(ghostFullRobotModel.getHand(segmentType.getSegmentSide()),
                                                                                  handDesiredControlFrames.get(segmentType.getSegmentSide())
                                                                                                          .getReferenceFrame(),
                                                                                  segmentType.getSegmentSide(),
                                                                                  segmentType.getPositionWeight(),
                                                                                  segmentType.getOrientationWeight(),
                                                                                  segmentType.getLinearRateLimitation(),
                                                                                  segmentType.getAngularRateLimitation(),
                                                                                  handIsLoaded);

               message.getControlFramePositionInEndEffector().set(ikControlFramePoses.get(segmentType.getSegmentSide()).getPosition());
               message.getControlFrameOrientationInEndEffector().set(ikControlFramePoses.get(segmentType.getSegmentSide()).getOrientation());

               // TODO commenting out because it was overly responsive in sim. Possibly remove for real robot
               message.setHasDesiredLinearVelocity(false);
               message.setHasDesiredAngularVelocity(false);
               message.getDesiredLinearVelocityInWorld().setToZero();
               message.getDesiredAngularVelocityInWorld().setToZero();

               toolboxInputMessage.getInputs().add().set(message);
               toolboxInputMessage.setTimestamp(System.nanoTime());
            }
            else
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
                                                                                          KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(ghostFullRobotModel.getHand(segmentType.getSegmentSide()),
                                                                                                                                                             handDesiredControlFrames.get(segmentType.getSegmentSide()).getReferenceFrame(),
                                                                                                                                                             segmentType.getSegmentSide(),
                                                                                                                                                             segmentType.getPositionWeight(),
                                                                                                                                                             segmentType.getOrientationWeight(),
                                                                                                                                                             segmentType.getLinearRateLimitation(),
                                                                                                                                                             segmentType.getAngularRateLimitation(),
                                                                                                                                                             handIsLoaded);

                                                                                          message.getControlFramePositionInEndEffector().set(ikControlFramePoses.get(segmentType.getSegmentSide()).getPosition());
                                                                                          message.getControlFrameOrientationInEndEffector().set(ikControlFramePoses.get(segmentType.getSegmentSide()).getOrientation());

                                                                                          // TODO commenting out because it was overly responsive in sim. Possibly remove for real robot
                                                                                          message.setHasDesiredLinearVelocity(true);
                                                                                          message.setHasDesiredAngularVelocity(true);
                                                                                          message.getDesiredLinearVelocityInWorld().set(controller.getLinearVelocity());
                                                                                          message.getDesiredAngularVelocityInWorld().set(controller.getAngularVelocity());

                                                                                          toolboxInputMessage.getInputs().add().set(message);
                                                                                          toolboxInputMessage.setTimestamp(controller.getLastPollTimeNanos());
                                                                                       }
                                                                                       else
                                                                                          controllerLastPollTimeNanos = controller.getLastPollTimeNanos();
                                                                                    });
            }
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
                                                                                  null,
                                                                                  segmentType.getPositionWeight(),
                                                                                  segmentType.getOrientationWeight(),
                                                                                  segmentType.getLinearRateLimitation(),
                                                                                  segmentType.getAngularRateLimitation(),
                                                                                  false);
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
         else
         { // control center of mass with right joystick
            KinematicsToolboxCenterOfMassMessage comMessage = new KinematicsToolboxCenterOfMassMessage();
            comMessage.setHasDesiredLinearVelocity(false);
            comMessage.getDesiredPositionInWorld().set(comJoystickXYInput.getX(), comJoystickXYInput.getY(), comJoystickZInput.getZ());

            if (comJoystickXYInput.containsNaN() || comJoystickZInput.containsNaN())
            {
               LogTools.info("CoM joystick offset contains NaN");
            }

            comMessage.getSelectionMatrix().setSelectionFrameId(toFrameId(ReferenceFrame.getWorldFrame()));
            comMessage.getSelectionMatrix().setXSelected(true);
            comMessage.getSelectionMatrix().setYSelected(true);
            comMessage.getSelectionMatrix().setZSelected(true);

            //            double comWeight = 0.5 / ghostFullRobotModel.getTotalMass();
            double comWeight = 2.0 / ghostFullRobotModel.getTotalMass();
            comMessage.getWeights().setXWeight(comWeight);
            comMessage.getWeights().setYWeight(comWeight);
            comMessage.getWeights().setZWeight(comWeight);

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

         // Stepping with ankle trackers
         if (gripButtonsValue.get(RobotSide.LEFT) > 0.5f && gripButtonsValue.get(RobotSide.RIGHT) > 0.5f)
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
               sleepToolbox();
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

         if (enabled.get())
            toolboxInputMessage.setStreamToController(streamToController.get());
         else
            toolboxInputMessage.setStreamToController(kinematicsRecorder.isReplaying());
         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputToolboxConfigurationTopic(syncedRobot.getRobotModel().getSimpleRobotName()), ikSolverConfigurationMessage);
         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputCommandTopic(syncedRobot.getRobotModel().getSimpleRobotName()), toolboxInputMessage);
         outputFrequencyPlot.recordEvent();
      }

      kinematicsRecorder.onUpdateEnd(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
   }

   private void handleLeftControllJoystickInput(boolean leftAButtonPressed, boolean leftTriggerPressed, double forwardJoystickValue, double lateralJoystickValue, boolean leftJoystickButtonClicked)
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
      if (forwardJoystickValue != 0.0)
      {
         comJoystickZInput.changeFrame(syncedRobot.getReferenceFrames().getCenterOfMassFrame());

         // Adjust X-axis based on forward joystick value
         if (Math.abs(forwardJoystickValue) > COM_CONTROL_JOYSTICK_THRESHOLD)
            comJoystickZInput.setZ(comJoystickZInput.getZ() + Math.signum(forwardJoystickValue) * COM_JOYSTICK_INCREMENT);

         //            LogTools.info(comJoystickZInput.getZ());
         comJoystickZInput.changeFrame(ReferenceFrame.getWorldFrame());
      }

      //         kinematicsRecorder.processRecordReplayInput(leftJoystickButton);

      boolean isReplaying = kinematicsRecorder.isReplayingEnabled().get();
      boolean isRecording = kinematicsRecorder.isRecordingEnabled().get();

      if (leftJoystickButtonClicked)
      {
         kinematicsRecorder.requestRecordReplay();
      }

      //         if (isReplaying)
      //            wakeUpToolbox();

      if (leftJoystickButtonClicked && !isReplaying && !isRecording)
      { // reinitialize toolbox
         LogTools.warn("Reinitializing toolbox. Forcing initial IK configuration to current robot configuration");
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

               initialConfigMessage.getInitialJointAngles().set(shyIndex, -0.5f);
               initialConfigMessage.getInitialJointAngles().set(shxIndex, robotSide.negateIfRightSide(-0.3f));
               initialConfigMessage.getInitialJointAngles().set(shzIndex, robotSide.negateIfRightSide(-0.5f));
               initialConfigMessage.getInitialJointAngles().set(elyIndex, -2.2f);
            }

            ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStreamingInitialConfigurationTopic(syncedRobot.getRobotModel()
                                                                                                                                .getSimpleRobotName()), initialConfigMessage);
            wakeUpToolbox();
            reinitializeToolbox();
            wakeUpToolbox();
         }
      }

   }

   private void handleRightControllJoystickInput(boolean rightAButtonPressed, boolean rightTriggerPressed, double forwardJoystickValue, double lateralJoystickValue)
   {
      if (rightAButtonPressed)
      {
         setEnabled(!enabled.get());
      }

      if (rightTriggerPressed)
      { // do not want to close grippers while interacting with the panel
         performHandAction(RobotSide.RIGHT);

         // TODO discuss and possibly remap to different button...

         //           double trajectoryTime = 1.5;
         //           GoHomeMessage homePelvis = new GoHomeMessage();
         //           homePelvis.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_PELVIS);
         //           homePelvis.setTrajectoryTime(trajectoryTime);
         //           ros2ControllerHelper.publishToController(homePelvis);
         //
         //           GoHomeMessage homeChest = new GoHomeMessage();
         //           homeChest.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_CHEST);
         //           homeChest.setTrajectoryTime(trajectoryTime);
         //
         //           RDXBaseUI.pushNotification("Commanding home pose...");
         //           ros2ControllerHelper.publishToController(homeChest);
         //
         //           prescientFootstepStreaming.reset();
         //           pausedForWalking = false;
         //           reintializingToolbox = false;
      }

      //         LogTools.info(forwardJoystickValue + ", " + lateralJoystickValue);

      if (forwardJoystickValue != 0.0 || lateralJoystickValue != 0.0)
      {
         comJoystickXYInput.changeFrame(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());

         // Adjust X-axis based on forward joystick value
         if (Math.abs(forwardJoystickValue) > COM_CONTROL_JOYSTICK_THRESHOLD)
            comJoystickXYInput.setX(comJoystickXYInput.getX() + Math.signum(forwardJoystickValue) * COM_JOYSTICK_INCREMENT);

         // Adjust Y-axis based on lateral joystick value
         if (Math.abs(lateralJoystickValue) > COM_CONTROL_JOYSTICK_THRESHOLD)
            comJoystickXYInput.setY(comJoystickXYInput.getY() + Math.signum(lateralJoystickValue) * COM_JOYSTICK_INCREMENT);

         //            LogTools.info(comJoystickXYInput.getX());
         comJoystickXYInput.changeFrame(ReferenceFrame.getWorldFrame());
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
      if (handControlModes.get(robotSide) == HandControlMode.GRIPPER)
      {
         publishHandCommand(robotSide);
      }
      else if (handControlModes.get(robotSide) == HandControlMode.LOAD_BEARING)
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
         ros2ControllerHelper.publish(ControllerAPI.getTopic(getInputTopic(robotModel.getSimpleRobotName()), HumanoidKinematicsToolboxConfigurationMessage.class), ikHumanoidSolverConfigurationMessage);
      }
      else
      {
         handLoadBearingMessage.setLoad(true);
         handLoadBearingMessage.setCoefficientOfFriction(HAND_CONTACT_COEFFICIENT_OF_FRICTION);

         // Contact point assumed to be at hand control frame and is using the nubs
         FramePoint3D contactPoint = new FramePoint3D(syncedRobot.getFullRobotModel().getHandControlFrame(robotSide));
         contactPoint.changeFrame(syncedRobot.getFullRobotModel().getHand(robotSide).getBodyFixedFrame());
         handLoadBearingMessage.getContactPointInBodyFrame().set(contactPoint);

         // Contact normal is hard-coded - HARDWARE
         //         FrameVector3D contactNormal = new FrameVector3D(syncedRobot.getReferenceFrames().getMidFeetZUpFrame(), HAND_CONTACT_NORMAL_IN_MID_FEET_ZUP_FRAME);
         //         contactNormal.changeFrame(ReferenceFrame.getWorldFrame());
         //         handLoadBearingMessage.getContactNormalInWorld().set(contactNormal);

         // Contact normal is hard-coded - SIMULATION
         handLoadBearingMessage.getContactNormalInWorld().set(HAND_CONTACT_NORMAL_IN_WORLD);

         handsAreLoaded.put(robotSide, true);

         ikHumanoidSolverConfigurationMessage.setHoldCurrentCenterOfMassXyPosition(false);
         ros2ControllerHelper.publish(ControllerAPI.getTopic(getInputTopic(robotModel.getSimpleRobotName()), HumanoidKinematicsToolboxConfigurationMessage.class), ikHumanoidSolverConfigurationMessage);
      }

      LogTools.info("publishing hand load bearing message " + robotSide + " hand, loading = " + handLoadBearingMessage.getLoad());
      ros2ControllerHelper.publishToController(handLoadBearingMessage);
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

   int counter = 0;

   private KinematicsToolboxRigidBodyMessage createRigidBodyMessage(RigidBodyBasics segment,
                                                                    ReferenceFrame desiredControlFrame,
                                                                    RobotSide robotSide,
                                                                    Vector3D positionWeight,
                                                                    Vector3D orientationWeight,
                                                                    double linearMomentumLimit,
                                                                    double angularMomentumLimit,
                                                                    boolean ignorePosition)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(segment.hashCode());

      tempFramePose.setToZero(desiredControlFrame);
      tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());

      // Record motion if in recording mode
      //      kinematicsRecorder.framePoseToRecord(tempFramePose, frameName);

      if (kinematicsRecorder.isReplaying())
         kinematicsRecorder.framePoseToPack(robotSide, tempFramePose); //get values of tempFramePose from replay

      message.getDesiredOrientationInWorld().set(tempFramePose.getOrientation());
      message.getDesiredPositionInWorld().set(tempFramePose.getPosition());

      WeightMatrix3D linearWeightMatrix = new WeightMatrix3D();
      message.getLinearSelectionMatrix().setXSelected(positionWeight.getX() != 0.0);
      message.getLinearSelectionMatrix().setYSelected(positionWeight.getY() != 0.0);
      linearWeightMatrix.setXAxisWeight(positionWeight.getX());
      linearWeightMatrix.setYAxisWeight(positionWeight.getY());
      message.getLinearSelectionMatrix().setZSelected(positionWeight.getZ() != 0.0);
      linearWeightMatrix.setZAxisWeight(positionWeight.getZ());

      if (ignorePosition)
      {
         message.getLinearSelectionMatrix().setXSelected(false);
         message.getLinearSelectionMatrix().setYSelected(false);
         message.getLinearSelectionMatrix().setZSelected(false);
      }

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
                  polygonGraphic.update(latestStatus, comJoystickXYInput.getX(), comJoystickXYInput.getY(), comJoystickZInput.getZ());
               }
            }
            if (capturabilityBasedStatus.getMessageNotification().poll())
            {
               CapturabilityBasedStatus capturabilityBasedStatus = this.capturabilityBasedStatus.getMessageNotification().read();
               for (RobotSide robotSide : RobotSide.values)
               {
                  handsAreLoaded.put(robotSide, HumanoidMessageTools.isHandLoadBearing(robotSide, capturabilityBasedStatus));
               }
            }

            if (ghostRobotGraphic.isActive())
               ghostRobotGraphic.update();
         }

         // Resumes streaming once walking is done
         if (pausedForWalking && controllerStatusTracker.getFinishedWalkingNotification().poll())
         {
            LogTools.info("Finished walking. Resuming streaming");
            // Restart KST
            lastStepCompletionTime = System.currentTimeMillis();
            reintializingToolbox = true;
         }
         else if (pausedForWalking && reintializingToolbox)
         {
            // Wait a bit for robot to stabilize on last footsteps
            if (System.currentTimeMillis() - lastStepCompletionTime > RDXVRPrescientFootstepStreaming.WAIT_TIME_AFTER_STEP)
            {
               pausedForWalking = false;
               setEnabled(true);
               visualizeIKPreviewGraphic(true);
               streamToController.set(true);
               reintializingToolbox = false;
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

      ImGui.checkbox(labels.get("Replay in midFeetZUp Frame"), replayInMidFeetZUpFrame);

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
         // reset desired com to equal the robot's actual current com
         comJoystickZInput.setToZero(syncedRobot.getReferenceFrames().getCenterOfMassFrame());
         comJoystickZInput.changeFrame(ReferenceFrame.getWorldFrame());
         comJoystickXYInput.setIncludingFrame(comJoystickZInput);

         if (!this.enabled.get())
         {
            wakeUpToolbox();
         }
         else
         {
            // Update initial configuration of KST
            KinematicsToolboxInitialConfigurationMessage initialConfigMessage = KinematicsToolboxMessageFactory.initialConfigurationFromFullRobotModel(
                  syncedRobot.getFullRobotModel());
            List<OneDoFJointBasics> oneDoFJoints = Arrays.asList(syncedRobot.getFullRobotModel().getOneDoFJoints());

            for (RobotSide robotSide : RobotSide.values)
            {
               int shyIndex = oneDoFJoints.indexOf(syncedRobot.getFullRobotModel().getArmJoint(robotSide, ArmJointName.SHOULDER_PITCH));
               int shxIndex = oneDoFJoints.indexOf(syncedRobot.getFullRobotModel().getArmJoint(robotSide, ArmJointName.SHOULDER_ROLL));
               int shzIndex = oneDoFJoints.indexOf(syncedRobot.getFullRobotModel().getArmJoint(robotSide, ArmJointName.SHOULDER_YAW));
               int elyIndex = oneDoFJoints.indexOf(syncedRobot.getFullRobotModel().getArmJoint(robotSide, ArmJointName.ELBOW_PITCH));

               initialConfigMessage.getInitialJointAngles().set(shyIndex, -0.5f);
               initialConfigMessage.getInitialJointAngles().set(shxIndex, robotSide.negateIfRightSide(-0.3f));
               initialConfigMessage.getInitialJointAngles().set(shzIndex, robotSide.negateIfRightSide(-0.5f));
               initialConfigMessage.getInitialJointAngles().set(elyIndex, -2.2f);
            }

            ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStreamingInitialConfigurationTopic(syncedRobot.getRobotModel()
                                                                                                                                .getSimpleRobotName()), initialConfigMessage);
            wakeUpToolbox();
            reinitializeToolbox();
            wakeUpToolbox();
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
      }
      else
      {
         streamingDisabled.poll();
         sleepToolbox();
         prescientFootstepStreaming.reset();
         pausedForWalking = false;
         reintializingToolbox = false;

         visualizeIKPreviewGraphic(true);
         streamToController.set(false);
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
      polygonGraphic.getRenderables(renderables, pool);

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

   public void publishHandCommand(RobotSide side)
   {
      boolean close = handsAreOpen.get(side).booleanValue();
      handsAreOpen.get(side).setValue(!close);
      handManager.publishHandCommand(side, close ? SakeHandPreset.CLOSE : SakeHandPreset.FULLY_OPEN, false, false);

      // TODO need to figure out a better option for how to distinguish open/close from carrying an object

      if (side == RobotSide.RIGHT)
         return;

      ObjectCarryMessage objectCarryMessage = new ObjectCarryMessage();
      objectCarryMessage.setObjectMass(OBJECT_MASS);
      objectCarryMessage.setIsPickingUp(true);
      objectCarryMessage.setRobotSide(side.toByte());

      LogTools.info("publishing object carry message");
      ros2ControllerHelper.publishToController(objectCarryMessage);
   }
}