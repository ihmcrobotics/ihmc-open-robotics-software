package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import gnu.trove.map.hash.TLongObjectHashMap;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.lwjgl.openvr.InputDigitalActionData;
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
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.DeprecatedAPIs;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
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
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.tools.KinematicsRecordReplay;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRControllerModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

public class RDXVRKinematicsStreamingMode
{
   private static final double FRAME_AXIS_GRAPHICS_LENGTH = 0.2;
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
   private final SideDependentList<Pose3D> ikHandControlFramePoses = new SideDependentList<>();
   private final SideDependentList<Pose3D> ikForearmControlFramePoses = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> ikHandControlFrameGraphics = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> ikForearmControlFrameGraphics = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> handFrameGraphics = new SideDependentList<>();
   private final Map<String, MutableReferenceFrame> trackedSegmentDesiredFrame = new HashMap<>();
   private final Map<String, RDXReferenceFrameGraphic> trackerFrameGraphics = new HashMap<>();
   private final ImBoolean showReferenceFrameGraphics = new ImBoolean(false);
   private final ImBoolean streamToController = new ImBoolean(false);
   private final Throttler messageThrottler = new Throttler();
   private KinematicsRecordReplay kinematicsRecorder;
   private final SceneGraph sceneGraph;
   @Nullable
   private KinematicsStreamingToolboxModule toolbox;

   private final ImBoolean controlArmsOnly = new ImBoolean(false);
   private ReferenceFrame pelvisFrame;
   private final RigidBodyTransform pelvisTransformToWorld = new RigidBodyTransform();
   private ReferenceFrame chestFrame;
   private final RigidBodyTransform chestTransformToWorld = new RigidBodyTransform();

   private final HandConfiguration[] handConfigurations = {HandConfiguration.HALF_CLOSE, HandConfiguration.CRUSH, HandConfiguration.CLOSE};
   private int leftIndex = -1;
   private int rightIndex = -1;
   private RDXVRControllerModel controllerModel = RDXVRControllerModel.UNKNOWN;


   private final AtomicReference<KinematicsStreamingToolboxInputMessage> toolboxInputMessagePending = new AtomicReference<>(null);

   public RDXVRKinematicsStreamingMode(ROS2SyncedRobotModel syncedRobot,
                                       ROS2ControllerHelper ros2ControllerHelper,
                                       RetargetingParameters retargetingParameters,
                                       SceneGraph sceneGraph)
   {
      this.syncedRobot = syncedRobot;
      this.robotModel = syncedRobot.getRobotModel();
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.retargetingParameters = retargetingParameters;
      this.sceneGraph = sceneGraph;
   }

   public void create(RDXVRContext vrContext, boolean createToolbox)
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

         ikHandControlFrameGraphics.put(side, new RDXReferenceFrameGraphic(1.25 * FRAME_AXIS_GRAPHICS_LENGTH));
         ikForearmControlFrameGraphics.put(side, new RDXReferenceFrameGraphic(1.25 * FRAME_AXIS_GRAPHICS_LENGTH));

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
         ikHandControlFramePoses.put(side, ikControlFramePose);

         Pose3D ikForearmControlFramePose = new Pose3D();
         ikForearmControlFramePoses.put(side, ikForearmControlFramePose);
      }

      status = ros2ControllerHelper.subscribe(KinematicsStreamingToolboxModule.getOutputStatusTopic(syncedRobot.getRobotModel().getSimpleRobotName()));

      kinematicsRecorder = new KinematicsRecordReplay(sceneGraph, enabled);

      KinematicsStreamingToolboxParameters parameters = new KinematicsStreamingToolboxParameters();
      parameters.setDefault();
      parameters.setPublishingPeriod(0.015); // Publishing period in seconds.
      parameters.setDefaultChestMessageAngularWeight(1.0, 1.0, 0.5);
      parameters.setDefaultPelvisMessageLinearWeight(10.0, 10.0, 15.0);
      parameters.setDefaultLinearRateLimit(10.0);
      parameters.setDefaultAngularRateLimit(100.0);
      parameters.setDefaultLinearWeight(10.0);
      parameters.setDefaultAngularWeight(0.1);
      parameters.setInputPoseLPFBreakFrequency(15.0);
      parameters.setInputPoseCorrectionDuration(0.05); // Need to send inputs at 30Hz.
      parameters.setInputStateEstimatorType(KinematicsStreamingToolboxParameters.InputStateEstimatorType.FBC_STYLE);

      parameters.setMinimizeAngularMomentum(true);
      parameters.setMinimizeLinearMomentum(true);
      parameters.setAngularMomentumWeight(0.25);
      parameters.setLinearMomentumWeight(0.25);

      parameters.getDefaultConfiguration().setEnableLeftHandTaskspace(false);
      parameters.getDefaultConfiguration().setEnableRightHandTaskspace(false);
      parameters.getDefaultConfiguration().setEnableNeckJointspace(false);
      parameters.getDefaultSolverConfiguration().setJointVelocityWeight(0.05);
      parameters.getDefaultSolverConfiguration().setEnableJointVelocityLimits(false);
      parameters.setUseStreamingPublisher(true);

      if (createToolbox)
      {
         boolean startYoVariableServer = true;
         toolbox = new KinematicsStreamingToolboxModule(robotModel, parameters, startYoVariableServer, DomainFactory.PubSubImplementation.FAST_RTPS);
         ((KinematicsStreamingToolboxController) toolbox.getToolboxController()).setInitialRobotConfigurationNamedMap(createInitialConfiguration(robotModel));
      }

      RDXBaseUI.getInstance().getKeyBindings().register("Streaming - Enable IK (toggle)", "Right A button");
      RDXBaseUI.getInstance().getKeyBindings().register("Streaming - Control robot (toggle)", "Left A button");
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
           { // do not want to close grippers while interacting with the panel
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

         if (controlArmsOnly.get())
         {
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
            message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(10));

            toolboxInputMessage.getInputs().add().set(message);
         }

         if (enabled.get())
            toolboxInputMessage.setStreamToController(streamToController.get());
         else
            toolboxInputMessage.setStreamToController(kinematicsRecorder.isReplaying());
//         toolboxInputMessage.setTimestamp();

         toolboxInputMessagePending.set(toolboxInputMessage);

         outputFrequencyPlot.recordEvent();
      }
   }

   private void handleTrackedSegment(RDXVRContext vrContext,
                                     KinematicsStreamingToolboxInputMessage toolboxInputMessage,
                                     VRTrackedSegmentType segmentType,
                                     Set<String> additionalTrackedSegments)
   {
      if (additionalTrackedSegments.contains(segmentType.getSegmentName()) && !controlArmsOnly.get())
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
            if (controlledSegment != null)
            {
               KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(controlledSegment,
                                                                                  trackedSegmentDesiredFrame.get(segmentType.getSegmentName())
                                                                                                            .getReferenceFrame(),
                                                                                  segmentType.getSegmentName(),
                                                                                  segmentType.getPositionWeight(),
                                                                                  segmentType.getOrientationWeight());
               toolboxInputMessage.getInputs().add().set(message);
            }
         });
      }
      else if (segmentType.getSegmentName().contains("Hand"))
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
            KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(ghostFullRobotModel.getHand(
                                                                                     segmentType.getSegmentSide()),
                                                                               handDesiredControlFrames.get(
                                                                                     segmentType.getSegmentSide()).getReferenceFrame(),
                                                                               segmentType.getSegmentName(),
                                                                               segmentType.getPositionWeight(),
                                                                               segmentType.getOrientationWeight());
            message.getControlFramePositionInEndEffector().set(ikHandControlFramePoses.get(segmentType.getSegmentSide()).getPosition());
            message.getControlFrameOrientationInEndEffector().set(ikHandControlFramePoses.get(segmentType.getSegmentSide()).getOrientation());

            message.setHasLinearVelocity(true);
            message.getLinearVelocityInWorld().set(controller.getLinearVelocity());
            message.setHasAngularVelocity(true);
            message.getAngularVelocityInWorld().set(controller.getAngularVelocity());
//            message.getDesiredOrientationInWorld().transform(message.getAngularVelocityInWorld());

            toolboxInputMessage.getInputs().add().set(message);
            toolboxInputMessage.setTimestamp(controller.getLastPollTimeNanos());

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
      // For updating on the fly TODO Remove me
      for (RobotSide robotSide : RobotSide.values)
      {
         if(robotSide == RobotSide.LEFT)
         {
            ikHandControlFramePoses.get(robotSide).getPosition().set(0.0, 0.05, 0.1);
            ikHandControlFramePoses.get(robotSide).getOrientation().setYawPitchRoll(0, 0.0, -Math.PI);
            ikForearmControlFramePoses.get(robotSide).getPosition().set(0.0, -0.15, 0.0);
            ikForearmControlFramePoses.get(robotSide).getOrientation().setYawPitchRoll(0.0, 0.0, -5 * Math.PI / 4);
         }
         else if(robotSide == RobotSide.RIGHT)
         {
            ikHandControlFramePoses.get(robotSide).getPosition().set(0.0, 0.2, 0.0);
            ikHandControlFramePoses.get(robotSide).getOrientation().setYawPitchRoll(0.0, 0.0, 0.0);
            ikForearmControlFramePoses.get(robotSide).getPosition().set(0.0, 0.0, -0.1);
            ikForearmControlFramePoses.get(robotSide).getOrientation().setYawPitchRoll(0.0, 0.0, 0.0);
         }

         RigidBodyBasics hand = ghostFullRobotModel.getHand(robotSide);
         if (hand != null)
         {
            RDXReferenceFrameGraphic handControlFrameGraphic = ikHandControlFrameGraphics.get(robotSide);
            handControlFrameGraphic.getFramePose3D().setToZero(hand.getBodyFixedFrame());
            handControlFrameGraphic.getFramePose3D().appendTransform(ikHandControlFramePoses.get(robotSide));
            handControlFrameGraphic.updateFromFramePose();
         }

         RigidBodyBasics forearm = ghostFullRobotModel.getForearm(robotSide);
         if (forearm != null)
         {
            RDXReferenceFrameGraphic forearmControlFrameGraphic = ikForearmControlFrameGraphics.get(robotSide);
            forearmControlFrameGraphic.getFramePose3D().setToZero(forearm.getBodyFixedFrame());
            forearmControlFrameGraphic.getFramePose3D().appendTransform(ikForearmControlFramePoses.get(robotSide));
            forearmControlFrameGraphic.updateFromFramePose();
         }

      }


      // ------------------------------------------------------------- End of Remove me

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

         // Publish input message if available
         KinematicsStreamingToolboxInputMessage inputToSend = toolboxInputMessagePending.getAndSet(null);
         if (inputToSend != null)
            ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputCommandTopic(syncedRobot.getRobotModel().getSimpleRobotName()), inputToSend);
      }
   }

   public void addIKRigidBodyInputs(List<KinematicsToolboxRigidBodyMessage> rigidBodyInputs)
   {
      KinematicsStreamingToolboxInputMessage inputToSend = toolboxInputMessagePending.getAndSet(null);
      if (inputToSend == null)
         return;

      TLongObjectHashMap<KinematicsToolboxRigidBodyMessage> hashCodeToRibitBodyInputMap = new TLongObjectHashMap<>();
      for (int i = 0; i < inputToSend.getInputs().size(); i++)
      {
         KinematicsToolboxRigidBodyMessage rigidBodyInput = inputToSend.getInputs().get(i);
         hashCodeToRibitBodyInputMap.put(rigidBodyInput.getEndEffectorHashCode(), rigidBodyInput);
      }

      // Add the new inputs after to give them priority
      for (KinematicsToolboxRigidBodyMessage rigidBodyInput : rigidBodyInputs)
      {
         hashCodeToRibitBodyInputMap.put(rigidBodyInput.getEndEffectorHashCode(), rigidBodyInput);
      }
      inputToSend.getInputs().clear();
      for (KinematicsToolboxRigidBodyMessage rigidBodyInput : hashCodeToRibitBodyInputMap.valueCollection())
      {
         inputToSend.getInputs().add().set(rigidBodyInput);
      }
      if(enabled.get())
      {
         inputToSend.setStreamToController(streamToController.get());
      }
      else
         inputToSend.setStreamToController(kinematicsRecorder.isReplaying());
      toolboxInputMessagePending.set(inputToSend);
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox(labels.get("Control/Stop Robot"), streamToController);
      
      if (ImGui.checkbox(labels.get("Kinematics streaming"), enabled))
      {
         setEnabled(enabled.get());
      }
      if (ImGui.checkbox(labels.get("Control only arms"), controlArmsOnly))
      {
         if (controlArmsOnly.get())
         {
            pelvisFrame = null;
            chestFrame = null;
         }
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

      // TODO Remove me?
      for (RobotSide side : RobotSide.values)
      {
         ikHandControlFrameGraphics.get(side).getRenderables(renderables, pool);
         ikForearmControlFrameGraphics.get(side).getRenderables(renderables, pool);
      }
   }

   public boolean isStreaming()
   {
      return streamToController.get();
   }

   public void visualizeIKPreviewGraphic(boolean visualize)
   {
      ghostRobotGraphic.setActive(visualize);
   }

   public void destroy()
   {
//      toolbox.closeAndDispose();
      ghostRobotGraphic.destroy();
      for (RobotSide side : RobotSide.values)
      {
         controllerFrameGraphics.get(side).dispose();
         handFrameGraphics.get(side).dispose();
         ikHandControlFrameGraphics.get(side).dispose();
         ikForearmControlFrameGraphics.get(side).dispose();
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

   public KinematicsToolboxRigidBodyMessage remoteCapturyStreaming(ReferenceFrame referenceFrame, int handSide, int type)
   {
      KinematicsToolboxRigidBodyMessage output = new KinematicsToolboxRigidBodyMessage();
      trackerFrameGraphics.put(VRTrackedSegmentType.LEFT_FOREARM.getSegmentName(),
                               new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
      trackerFrameGraphics.put(VRTrackedSegmentType.RIGHT_FOREARM.getSegmentName(),
                               new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
      if(handSide == 0)
      {
         if(type == 0)
         {
            handDesiredControlFrames.put(RobotSide.LEFT, new MutableReferenceFrame(referenceFrame));
            MovingReferenceFrame endEffectorFrame = ghostFullRobotModel.getEndEffectorFrame(VRTrackedSegmentType.LEFT_HAND.getSegmentSide(), LimbName.ARM);
            controllerFrameGraphics.get(VRTrackedSegmentType.LEFT_HAND.getSegmentSide()).setToReferenceFrame(referenceFrame);
            handFrameGraphics.get(VRTrackedSegmentType.LEFT_HAND.getSegmentSide()).setToReferenceFrame(endEffectorFrame);
            KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(ghostFullRobotModel.getHand(RobotSide.LEFT),
                                                                               handDesiredControlFrames.get(VRTrackedSegmentType.LEFT_HAND.getSegmentSide()).getReferenceFrame(),
                                                                               VRTrackedSegmentType.LEFT_HAND.getSegmentName(),
                                                                               VRTrackedSegmentType.LEFT_HAND.getPositionWeight(),
                                                                               VRTrackedSegmentType.LEFT_HAND.getOrientationWeight());
            message.getControlFramePositionInEndEffector().set(ikHandControlFramePoses.get(VRTrackedSegmentType.LEFT_HAND.getSegmentSide()).getPosition());
            message.getControlFrameOrientationInEndEffector().set(ikHandControlFramePoses.get(VRTrackedSegmentType.LEFT_HAND.getSegmentSide()).getOrientation());
            message.setHasAngularVelocity(true);
            message.setHasLinearVelocity(true);
            output.set(message);
         }
         else if(type == 1)
         {
            MutableReferenceFrame trackerDesiredControlFrame = new MutableReferenceFrame(referenceFrame);
            trackerDesiredControlFrame.getReferenceFrame().update();
            trackedSegmentDesiredFrame.put(VRTrackedSegmentType.LEFT_FOREARM.getSegmentName(), trackerDesiredControlFrame);
            trackerFrameGraphics.get(VRTrackedSegmentType.LEFT_FOREARM.getSegmentName()).setToReferenceFrame(trackedSegmentDesiredFrame.get(VRTrackedSegmentType.LEFT_FOREARM.getSegmentName()).getReferenceFrame());
            RigidBodyBasics controlledSegment = ghostFullRobotModel.getForearm(RobotSide.LEFT);

            if (controlledSegment != null)
            {
               KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(controlledSegment,
                                                                                  trackedSegmentDesiredFrame.get(VRTrackedSegmentType.LEFT_FOREARM.getSegmentName())
                                                                                                            .getReferenceFrame(),
                                                                                  VRTrackedSegmentType.LEFT_FOREARM.getSegmentName(),
                                                                                  VRTrackedSegmentType.LEFT_FOREARM.getPositionWeight(),
                                                                                  VRTrackedSegmentType.LEFT_FOREARM.getOrientationWeight());
               message.getControlFramePositionInEndEffector().set(ikForearmControlFramePoses.get(VRTrackedSegmentType.LEFT_FOREARM.getSegmentSide()).getPosition());
               message.getControlFrameOrientationInEndEffector().set(ikForearmControlFramePoses.get(VRTrackedSegmentType.LEFT_FOREARM.getSegmentSide()).getOrientation());
               message.setHasAngularVelocity(true);
               message.setHasLinearVelocity(true);
               output.set(message);
            }
         }

      }
      else if(handSide == 1)
      {
         if(type == 0)
         {
            handDesiredControlFrames.put(RobotSide.RIGHT, new MutableReferenceFrame(referenceFrame));
            MovingReferenceFrame endEffectorFrame = ghostFullRobotModel.getEndEffectorFrame(VRTrackedSegmentType.RIGHT_HAND.getSegmentSide(), LimbName.ARM);
            controllerFrameGraphics.get(VRTrackedSegmentType.RIGHT_HAND.getSegmentSide()).setToReferenceFrame(referenceFrame);
            handFrameGraphics.get(VRTrackedSegmentType.RIGHT_HAND.getSegmentSide()).setToReferenceFrame(endEffectorFrame);
            KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(ghostFullRobotModel.getHand(RobotSide.RIGHT),
                                                                               handDesiredControlFrames.get(VRTrackedSegmentType.RIGHT_HAND.getSegmentSide()).getReferenceFrame(),
                                                                               VRTrackedSegmentType.RIGHT_HAND.getSegmentName(),
                                                                               VRTrackedSegmentType.RIGHT_HAND.getPositionWeight(),
                                                                               VRTrackedSegmentType.RIGHT_HAND.getOrientationWeight());
            message.getControlFramePositionInEndEffector().set(ikHandControlFramePoses.get(VRTrackedSegmentType.RIGHT_HAND.getSegmentSide()).getPosition());
            message.getControlFrameOrientationInEndEffector().set(ikHandControlFramePoses.get(VRTrackedSegmentType.RIGHT_HAND.getSegmentSide()).getOrientation());
            message.setHasAngularVelocity(true);
            message.setHasLinearVelocity(true);
            output.set(message);
         }
         else if(type == 1)
         {
            MutableReferenceFrame trackerDesiredControlFrame = new MutableReferenceFrame(referenceFrame);
            trackerDesiredControlFrame.getReferenceFrame().update();
            trackedSegmentDesiredFrame.put(VRTrackedSegmentType.RIGHT_FOREARM.getSegmentName(), trackerDesiredControlFrame);
            trackerFrameGraphics.get(VRTrackedSegmentType.RIGHT_FOREARM.getSegmentName()).setToReferenceFrame(referenceFrame);
            RigidBodyBasics controlledSegment = ghostFullRobotModel.getForearm(RobotSide.RIGHT);

            if (controlledSegment != null)
            {
               KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(controlledSegment,
                                                                                  trackedSegmentDesiredFrame.get(VRTrackedSegmentType.RIGHT_FOREARM.getSegmentName())
                                                                                                            .getReferenceFrame(),
                                                                                  VRTrackedSegmentType.RIGHT_FOREARM.getSegmentName(),
                                                                                  VRTrackedSegmentType.RIGHT_FOREARM.getPositionWeight(),
                                                                                  VRTrackedSegmentType.RIGHT_FOREARM.getOrientationWeight());
               message.getControlFramePositionInEndEffector().set(ikForearmControlFramePoses.get(VRTrackedSegmentType.RIGHT_FOREARM.getSegmentSide()).getPosition());
               message.getControlFrameOrientationInEndEffector().set(ikForearmControlFramePoses.get(VRTrackedSegmentType.RIGHT_FOREARM.getSegmentSide()).getOrientation());
               message.setHasAngularVelocity(true);
               message.setHasLinearVelocity(true);
               output.set(message);
            }
         }
      }
      return output;
   }
}