package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage;
import ihmc_common_msgs.msg.dds.WeightMatrix3DMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.lwjgl.openvr.InputDigitalActionData;
import toolbox_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.UnitConversions;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.communication.ros2log.ROS2LogRecord;
import us.ihmc.communication.ros2log.ROS2LoggerRequestedState;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.motionRetargeting.RetargetingParameters;
import us.ihmc.motionRetargeting.VRTrackedSegmentType;
import us.ihmc.rdx.imgui.ImGuiFrequencyPlot;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.rdx.ui.graphics.RDXMultiContactRegionGraphic;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2RobotVisualizer;
import us.ihmc.rdx.ui.tools.VRControlRecordReplay;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRHardwareModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
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
import java.util.concurrent.atomic.AtomicBoolean;

import static us.ihmc.communication.packets.MessageTools.toFrameId;
import static us.ihmc.motionRetargeting.VRTrackedSegmentType.*;

public class RDXVRKinematicsStreamingMode
{
   public static final boolean ENABLE_YO_VARIABLE_TOOLBOX_SERVERS = false;
   public static final double FRAME_AXIS_GRAPHICS_LENGTH = 0.2;

   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final RetargetingParameters retargetingParameters;
   private final DRCRobotModel robotModel;
   private float userRobotOpacity = 1.0f; // store this so we can avoid overriding the user
   private RDXMultiBodyGraphic ghostRobotGraphic;
   private final ImBoolean showGhosts = new ImBoolean(true);
   private FullHumanoidRobotModel ghostFullRobotModel;
   private OneDoFJointBasics[] ghostOneDoFJointsExcludingHands;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean isKSTEnabled = new ImBoolean(false);
   private final ImBoolean streamToController = new ImBoolean(false);
   private final AtomicBoolean requestRecordReplay = new AtomicBoolean(false);

   @Nullable
   private KinematicsStreamingToolboxModule toolbox;
   private KinematicsStreamingToolboxParameters kstParameters;
   private final KinematicsToolboxConfigurationMessage ikSolverConfigurationMessage = new KinematicsToolboxConfigurationMessage();

   private ROS2Input<KinematicsToolboxOutputStatus> status;
   private final double streamPeriod = UnitConversions.hertzToSeconds(120.0);
   private final Throttler toolboxInputStreamRateLimiter = new Throttler();
   private final ImGuiFrequencyPlot statusFrequencyPlot = new ImGuiFrequencyPlot();
   private final ImGuiFrequencyPlot outputFrequencyPlot = new ImGuiFrequencyPlot();
   public long controllerLastPollTimeNanos;

   private final FramePose3D tempFramePose = new FramePose3D();
   private final SideDependentList<MutableReferenceFrame> handDesiredControlFrames = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> controllerFrameGraphics = new SideDependentList<>();
   private final SideDependentList<Pose3D> ikControlFramePoses = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> handFrameGraphics = new SideDependentList<>();
   private Set<String> additionalTrackedSegments = new HashSet<>();
   private final Map<String, MutableReferenceFrame> trackerReferenceFrames = new HashMap<>();
   private final Map<String, RDXReferenceFrameGraphic> trackerFrameGraphics = new HashMap<>();
   private final RDXReferenceFrameGraphic chestFrameGraphics = new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH);
   private MutableReferenceFrame headsetReferenceFrame;
   private final ImBoolean showReferenceFrameGraphics = new ImBoolean(false);
   private final Throttler messageThrottler = new Throttler();

   private VRControlRecordReplay kinematicsRecorder;
   private ROS2Publisher<ROS2LogMessage> ros2LogMessagePublisher;
   private final RDXVRContext vrContext;

   private final RDXROS2RobotVisualizer robotVisualizer;

   private final ImBoolean controlArmsOnly = new ImBoolean(false);
   private final ImBoolean armScaling = new ImBoolean(false);
   private final ImBoolean comTracking = new ImBoolean(true);
   private RDXVRMotionRetargeting motionRetargeting;

   private RDXMultiContactRegionGraphic multiContactStabilityGraphic;

   private final FramePoint3D comPositionInitial = new FramePoint3D();
   private final FrameVector3D comTrackerOffset = new FrameVector3D();
   private final FramePoint3D desiredCoMPositionFiltered = new FramePoint3D();

   public RDXVRKinematicsStreamingMode(ROS2SyncedRobotModel syncedRobot,
                                       ROS2ControllerHelper ros2ControllerHelper,
                                       RDXVRContext vrContext,
                                       RetargetingParameters retargetingParameters,
                                       RDXROS2RobotVisualizer robotVisualizer)
   {
      this.syncedRobot = syncedRobot;
      this.robotModel = syncedRobot.getRobotModel();
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.retargetingParameters = retargetingParameters;
      this.vrContext = vrContext;
      this.robotVisualizer = robotVisualizer;
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

      multiContactStabilityGraphic = new RDXMultiContactRegionGraphic(ghostFullRobotModel);

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

      kinematicsRecorder = new VRControlRecordReplay(isKSTEnabled, handDesiredControlFrames);
      ros2LogMessagePublisher = ros2ControllerHelper.getROS2Node().createPublisher(ROS2LogRecord.getROS2LogTopic());
      kinematicsRecorder.setRecordCallback(this::onPlaybackChanged);
      kinematicsRecorder.setReplayCallback(this::onPlaybackChanged);

      motionRetargeting = new RDXVRMotionRetargeting(syncedRobot.getFullRobotModel(), ghostFullRobotModel, handDesiredControlFrames, trackerReferenceFrames, headsetReferenceFrame, retargetingParameters);

      if (createToolbox)
      {
         toolbox = new KinematicsStreamingToolboxModule(robotModel, kstParameters, ENABLE_YO_VARIABLE_TOOLBOX_SERVERS);
      }

      if (vrContext.getVRModel() == RDXVRHardwareModel.FOCUS3)
      {
         RDXBaseUI.getInstance().getKeyBindings().register("Show/Hide ghosts", "Y button");
         RDXBaseUI.getInstance().getKeyBindings().register("Streaming - Enable IK (toggle)", "A button");
         RDXBaseUI.getInstance().getKeyBindings().register("Streaming - Control robot (toggle)", "X button");
      }
      else
      {
         RDXBaseUI.getInstance().getKeyBindings().register("Show/Hide ghosts", "Left B button");
         RDXBaseUI.getInstance().getKeyBindings().register("Streaming - Enable IK (toggle)", "Right A button");
         RDXBaseUI.getInstance().getKeyBindings().register("Streaming - Control robot (toggle)", "Left A button");
         RDXBaseUI.getInstance().getKeyBindings().register("Footstep Streaming - Control robot stepping (ankle trackers required)", "Hold both handle grippers");
      }
   }

   private void onPlaybackChanged(boolean isStarting)
   {
      ROS2LogMessage message = new ROS2LogMessage();
      message.setRequestedState((isStarting ? ROS2LoggerRequestedState.START : ROS2LoggerRequestedState.FINISH).toByte());
      ros2LogMessagePublisher.publish(message);
   }

   public void processVRInput()
   {
      kinematicsRecorder.onUpdateStart();

      // Handle left joystick input
      if (kinematicsRecorder.isReplaying())
      {
         boolean leftAButtonPressed = kinematicsRecorder.getAButtonPressed(RobotSide.LEFT);
         boolean leftBButtonPressed = kinematicsRecorder.getBButtonPressed(RobotSide.LEFT);
         boolean leftTriggerPressed = kinematicsRecorder.getTriggerPressed(RobotSide.LEFT);
         double joystickX = kinematicsRecorder.getJoystickX(RobotSide.LEFT);
         double joystickY = kinematicsRecorder.getJoystickY(RobotSide.LEFT);
         handleLeftControllerJoystickInput(leftAButtonPressed, leftBButtonPressed, leftTriggerPressed, joystickX, joystickY);
      }
      else
      {
         vrContext.getController(RobotSide.LEFT).runIfConnected(controller ->
         {
            controller.setAButtonText(streamToController.get() ? "Stop control" : "Start control");
            if (streamToController.get())
               controller.setBButtonText(showGhosts.get() ? "Hide ghosts" : "Show ghosts");
            else
               controller.setBButtonText("Relax hands");

            InputDigitalActionData aButton = controller.getAButtonActionData();
            InputDigitalActionData bButton = controller.getBButtonActionData();
            InputDigitalActionData clickTriggerButton = controller.getClickTriggerActionData();
            InputDigitalActionData leftJoystickButton = controller.getJoystickPressActionData();
            boolean leftJoystickButtonClicked = leftJoystickButton.bChanged() && !leftJoystickButton.bState();

            float joystickX = controller.getJoystickActionData().x();
            float joystickY = controller.getJoystickActionData().y();

            boolean leftAButtonPressed = aButton.bChanged() && !aButton.bState();
            boolean leftBButtonPressed = bButton.bChanged() && !bButton.bState();
            boolean leftTriggerPressed = clickTriggerButton.bChanged() && !clickTriggerButton.bState();
            handleLeftControllerJoystickInput(leftAButtonPressed, leftBButtonPressed, leftTriggerPressed, joystickX, joystickY);

            kinematicsRecorder.recordControllerData(RobotSide.LEFT,
                                                    leftAButtonPressed,
                                                    leftBButtonPressed,
                                                    leftTriggerPressed,
                                                    joystickX,
                                                    joystickY,
                                                    controller.getAngularVelocity(),
                                                    controller.getLinearVelocity(),
                                                    getTrajectoryRecordFrame());
            controllerLastPollTimeNanos = controller.getLastPollTimeNanos();
         });
      }

      // Process record/replay request from button in the desktop view
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
         double joystickX = kinematicsRecorder.getJoystickX(RobotSide.RIGHT);
         double joystickY = kinematicsRecorder.getJoystickY(RobotSide.RIGHT);
         handleRightControllerJoystickInput(rightAButtonPressed, rightBButtonPressed, rightTriggerPressed, joystickX, joystickY);
      }
      else
      {
         vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
         {
            controller.setAButtonText(isKSTEnabled.get() ? "Stop preview" : "Start preview");

            if (kinematicsRecorder.isRecording())
               controller.setBButtonText("Stop recording");
            else if (kinematicsRecorder.isReplaying())
               controller.setBButtonText("Stop replay");
            else
               controller.setBButtonText("Record/Replay");

            InputDigitalActionData aButton = controller.getAButtonActionData();
            InputDigitalActionData bButton = controller.getBButtonActionData();
            InputDigitalActionData clickTriggerButton = controller.getClickTriggerActionData();

            float joystickX = controller.getJoystickActionData().x();
            float joystickY = controller.getJoystickActionData().y();

            boolean rightAButtonPressed = aButton.bChanged() && !aButton.bState();
            boolean rightBButtonPressed = bButton.bChanged() && !bButton.bState();
            boolean rightTriggerPressed = clickTriggerButton.bChanged() && !clickTriggerButton.bState();
            handleRightControllerJoystickInput(rightAButtonPressed, rightBButtonPressed, rightTriggerPressed, joystickX, joystickY);

            kinematicsRecorder.recordControllerData(RobotSide.RIGHT,
                                                    rightAButtonPressed,
                                                    rightBButtonPressed,
                                                    rightTriggerPressed,
                                                    joystickX,
                                                    joystickY,
                                                    controller.getAngularVelocity(),
                                                    controller.getLinearVelocity(),
                                                    getTrajectoryRecordFrame());
            controllerLastPollTimeNanos = controller.getLastPollTimeNanos();
         });
      }

      if (isKSTEnabled.get() && toolboxInputStreamRateLimiter.run(streamPeriod))
      {
         KinematicsStreamingToolboxInputMessage toolboxInputMessage = new KinematicsStreamingToolboxInputMessage();
         processTrackers(toolboxInputMessage);
         processControllers(toolboxInputMessage);
//         doCoMControl(toolboxInputMessage);
         retargetMotion(toolboxInputMessage);

         if (isKSTEnabled.get())
            toolboxInputMessage.setStreamToController(streamToController.get());
         else
            toolboxInputMessage.setStreamToController(kinematicsRecorder.isReplaying());

         toolboxInputMessage.setTimestamp(kinematicsRecorder.isReplaying() ? System.nanoTime() : controllerLastPollTimeNanos);

         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputToolboxConfigurationTopic(syncedRobot.getRobotModel().getSimpleRobotName()),
                                      ikSolverConfigurationMessage);
         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputCommandTopic(syncedRobot.getRobotModel().getSimpleRobotName()),
                                      toolboxInputMessage);
         outputFrequencyPlot.recordEvent();
      }

      kinematicsRecorder.onUpdateEnd(getTrajectoryRecordFrame());
   }

   private ReferenceFrame getTrajectoryRecordFrame()
   {
      return syncedRobot.getReferenceFrames().getMidFeetZUpFrame();
   }

   private void handleLeftControllerJoystickInput(boolean leftAButtonPressed,
                                                  boolean leftBButtonPressed,
                                                  boolean leftTriggerPressed,
                                                  double joystickX,
                                                  double joystickY)
   {
      if (isKSTEnabled.get() && leftAButtonPressed)
         setStreamToController(!streamToController.get(), true);

      if (leftBButtonPressed)
      {
         if (streamToController.get())
            showGhosts.set(!showGhosts.get());
      }

      if (streamToController.get() && leftTriggerPressed)
      {

      }
   }

   private void handleRightControllerJoystickInput(boolean rightAButtonPressed,
                                                   boolean rightBButtonPressed,
                                                   boolean rightTriggerPressed,
                                                   double joystickX,
                                                   double joystickY)
   {
      if (rightAButtonPressed)
      {
         setKSTEnabled(!isKSTEnabled.get());
      }

      if (rightTriggerPressed)
      { // do not want to close grippers while interacting with the panel
         reinitializeToolboxRobotConfiguration();
      }

      if (rightBButtonPressed)
      {
         kinematicsRecorder.requestRecordReplay();
      }
   }

   private void processTrackers(KinematicsStreamingToolboxInputMessage messageToPack)
   {
      additionalTrackedSegments = vrContext.getAssignedTrackerRoles();
      for (VRTrackedSegmentType segmentType : VRTrackedSegmentType.TRACKER_TYPES)
      {
         if (additionalTrackedSegments.contains(segmentType.getSegmentName()) && !controlArmsOnly.get())
         {
            FramePose3D desiredPose = new FramePose3D();
            FrameVector3D desiredAngularVelocity = new FrameVector3D();
            FrameVector3D desiredLinearVelocity = new FrameVector3D();

            if (kinematicsRecorder.isReplaying())
            {
               kinematicsRecorder.packLoggedData(segmentType, desiredPose, desiredAngularVelocity, desiredLinearVelocity);
            }
            else
            {
               vrContext.getTracker(segmentType.getSegmentName()).runIfConnected(tracker ->
               {
                  if (!trackerReferenceFrames.containsKey(segmentType.getSegmentName()))
                  {
                     MutableReferenceFrame trackerDesiredControlFrame = new MutableReferenceFrame(tracker.getXForwardZUpTrackerFrame());
                     trackerDesiredControlFrame.getTransformToParent()
                                               .getRotation()
                                               .appendInvertOther(retargetingParameters.getControlFrameOrientationInBodyFrame(segmentType));
                     trackerDesiredControlFrame.getReferenceFrame().update();
                     trackerReferenceFrames.put(segmentType.getSegmentName(), trackerDesiredControlFrame);
                     if (segmentType == CHEST)
                        chestFrameGraphics.setToReferenceFrame(ghostFullRobotModel.getChest().getBodyFixedFrame());
                  }

                  if (!trackerFrameGraphics.containsKey(segmentType.getSegmentName()))
                  {
                     trackerFrameGraphics.put(segmentType.getSegmentName(), new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
                  }
                  trackerFrameGraphics.get(segmentType.getSegmentName())
                                      .setToReferenceFrame(trackerReferenceFrames.get(segmentType.getSegmentName()).getReferenceFrame());

                  desiredPose.setToZero(trackerReferenceFrames.get(segmentType.getSegmentName()).getReferenceFrame());
                  desiredPose.changeFrame(ReferenceFrame.getWorldFrame());
                  desiredAngularVelocity.set(tracker.getAngularVelocity());
                  desiredLinearVelocity.set(tracker.getLinearVelocity());

                  kinematicsRecorder.recordTrackerData(segmentType,
                                                       trackerReferenceFrames.get(segmentType.getSegmentName()).getReferenceFrame(),
                                                       desiredAngularVelocity,
                                                       desiredLinearVelocity,
                                                       getTrajectoryRecordFrame());
               });
            }
            if (motionRetargeting.isRetargetingNotNeeded(segmentType))
            {
               RigidBodyBasics controlledSegment = getControlledSegment(segmentType);

               if (controlledSegment != null)
               {
                  KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(controlledSegment,
                          desiredPose,
                          desiredAngularVelocity,
                          desiredLinearVelocity,
                          retargetingParameters.getPositionWeight(segmentType),
                          retargetingParameters.getOrientationWeight(segmentType),
                          retargetingParameters.getLinearRateLimitation(segmentType),
                          retargetingParameters.getAngularRateLimitation(segmentType));
                  messageToPack.getInputs().add().set(message);
               }
            }
         }
      }
   }

   private void processControllers(KinematicsStreamingToolboxInputMessage messageToPack)
   {
      for (VRTrackedSegmentType segmentType : CONTROLLER_TYPES)
      {
         Vector3D positionWeight = retargetingParameters.getPositionWeight(segmentType);
         Vector3D orientationWeight = retargetingParameters.getOrientationWeight(segmentType);
         double linearRateLimitation = retargetingParameters.getLinearRateLimitation(segmentType);
         double angularRateLimitation = retargetingParameters.getAngularRateLimitation(segmentType);

         FramePose3D desiredPose = new FramePose3D();
         FrameVector3D desiredAngularVelocity = new FrameVector3D();
         FrameVector3D desiredLinearVelocity = new FrameVector3D();

         RigidBodyBasics hand = ghostFullRobotModel.getHand(segmentType.getSegmentSide());
         ReferenceFrame handControlFrame = handDesiredControlFrames.get(segmentType.getSegmentSide()).getReferenceFrame();

         if (kinematicsRecorder.isReplaying())
         {
            kinematicsRecorder.packLoggedData(segmentType, desiredPose, desiredAngularVelocity, desiredLinearVelocity);
            controllerLastPollTimeNanos = System.nanoTime();
         }
         else
         {
            vrContext.getController(segmentType.getSegmentSide()).runIfConnected(controller ->
            {
               MovingReferenceFrame endEffectorFrame = ghostFullRobotModel.getEndEffectorFrame(segmentType.getSegmentSide(), LimbName.ARM);
               if (endEffectorFrame == null)
                  return;

               desiredPose.setToZero(handControlFrame);
               desiredPose.changeFrame(ReferenceFrame.getWorldFrame());
               controller.getXForwardZUpControllerFrame().update();
               controllerFrameGraphics.get(segmentType.getSegmentSide()).setToReferenceFrame(controller.getXForwardZUpControllerFrame());
               handFrameGraphics.get(segmentType.getSegmentSide()).setToReferenceFrame(endEffectorFrame);

               desiredLinearVelocity.set(controller.getLinearVelocity());
               desiredAngularVelocity.set(controller.getAngularVelocity());
               controllerLastPollTimeNanos = controller.getLastPollTimeNanos();
            });
         }

         if (!armScaling.get())
         {
            KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(hand,
                                                                               desiredPose,
                                                                               desiredAngularVelocity,
                                                                               desiredLinearVelocity,
                                                                               positionWeight,
                                                                               orientationWeight,
                                                                               linearRateLimitation,
                                                                               angularRateLimitation);
            message.getControlFramePositionInEndEffector().set(ikControlFramePoses.get(segmentType.getSegmentSide()).getPosition());
            message.getControlFrameOrientationInEndEffector().set(ikControlFramePoses.get(segmentType.getSegmentSide()).getOrientation());
            messageToPack.getInputs().add().set(message);
         }
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
      // Update contact state
      if (motionRetargeting.isControllingFeet())
      {
         KinematicsStreamingToolboxContactConfigurationMessage contactConfigMessage = new KinematicsStreamingToolboxContactConfigurationMessage();
         contactConfigMessage.setLeftFootInContact(motionRetargeting.isFootInContact(RobotSide.LEFT));
         contactConfigMessage.setRightFootInContact(motionRetargeting.isFootInContact(RobotSide.RIGHT));
         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStreamingContactConfigurationTopic(syncedRobot.getRobotModel().getSimpleRobotName()), contactConfigMessage);
      }

      for (VRTrackedSegmentType segmentType : motionRetargeting.getRetargetedSegments())
      {
         RigidBodyBasics controlledSegment = getControlledSegment(segmentType);
         if (controlledSegment != null)
         {
            tempFramePose.setToZero(motionRetargeting.getDesiredFrame(segmentType));
            tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());

            KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(controlledSegment,
                                                                               tempFramePose,
                                                                               null,
                                                                               null,
                                                                               retargetingParameters.getPositionWeight(segmentType),
                                                                               retargetingParameters.getOrientationWeight(segmentType),
                                                                               retargetingParameters.getLinearRateLimitation(segmentType),
                                                                               retargetingParameters.getAngularRateLimitation(segmentType));
            if (segmentType.isHandRelated())
            {
               // TODO. Linear desired velocities from controller/trackers might be wrong now because of scaling
               // Check arm scaling state not changed -> disabled
               if (!isKSTEnabled.get())
                  return;
               message.getControlFramePositionInEndEffector().set(ikControlFramePoses.get(segmentType.getSegmentSide()).getPosition());
               message.getControlFrameOrientationInEndEffector().set(ikControlFramePoses.get(segmentType.getSegmentSide()).getOrientation());
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
         case LEFT_ANKLE, RIGHT_ANKLE -> ghostFullRobotModel.getFoot(segmentType.getSegmentSide());
         case LEFT_WRIST, RIGHT_WRIST -> ghostFullRobotModel.getForearm(segmentType.getSegmentSide());
         case CHEST -> ghostFullRobotModel.getChest();
         case WAIST -> ghostFullRobotModel.getPelvis();
         default -> null;
      };
   }

   private KinematicsToolboxRigidBodyMessage createRigidBodyMessage(RigidBodyBasics segment,
                                                                    FramePose3DReadOnly desiredPose,
                                                                    Vector3DReadOnly angularVelocity,
                                                                    Vector3DReadOnly linearVelocity,
                                                                    Vector3D positionWeight,
                                                                    Vector3D orientationWeight,
                                                                    double linearMomentumLimit,
                                                                    double angularMomentumLimit)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(segment.hashCode());

      message.getDesiredOrientationInWorld().set(desiredPose.getOrientation());
      message.getDesiredPositionInWorld().set(desiredPose.getPosition());

      message.setHasDesiredAngularVelocity(angularVelocity != null);
      message.setHasDesiredLinearVelocity(linearVelocity != null);

      if (message.getHasDesiredAngularVelocity())
         message.getDesiredAngularVelocityInWorld().set(angularVelocity);
      if (message.getHasDesiredLinearVelocity())
         message.getDesiredLinearVelocityInWorld().set(linearVelocity);

      message.setLinearRateLimitation(linearMomentumLimit);
      message.setAngularRateLimitation(angularMomentumLimit);

      configureWeightAndSelectionMatrices(positionWeight, message.getLinearSelectionMatrix(), message.getLinearWeightMatrix());
      configureWeightAndSelectionMatrices(orientationWeight, message.getAngularSelectionMatrix(), message.getAngularWeightMatrix());

      return message;
   }

   private static void configureWeightAndSelectionMatrices(Vector3D weightVector,
                                                           SelectionMatrix3DMessage selectionMatrixMessage,
                                                           WeightMatrix3DMessage weightMatrixMessage)
   {
      selectionMatrixMessage.setXSelected(weightVector.getX() != 0.0);
      selectionMatrixMessage.setYSelected(weightVector.getY() != 0.0);
      selectionMatrixMessage.setZSelected(weightVector.getZ() != 0.0);
      weightMatrixMessage.setXWeight(weightVector.getX());
      weightMatrixMessage.setYWeight(weightVector.getY());
      weightMatrixMessage.setZWeight(weightVector.getZ());
   }

   public void update(boolean ikStreamingModeActive)
   {
      // Safety feature, disable streaming when mode is active
      if (!ikStreamingModeActive)
      {
         setStreamToController(false, false);
      }
      else // Mode active
      {
         if (!isKSTEnabled.get())
         {
            setStreamToController(false, false);
         }

         if (isKSTEnabled.get() || kinematicsRecorder.isReplaying()) // If KST or replay enabled
         {
            if (streamToController.get())
            {
               ghostRobotGraphic.setActive(showGhosts.get());
               robotVisualizer.setActive(showGhosts.get());
               if (showGhosts.get())
                  robotVisualizer.setOpacity(0.5f);
            }

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
                  multiContactStabilityGraphic.update(latestStatus, desiredCoMPositionFiltered);
               }
            }

            if (ghostRobotGraphic.isActive())
               ghostRobotGraphic.update();
         }
      }
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.checkbox(labels.get("Control robot"), streamToController))
      {
         setStreamToController(streamToController.get(), true);
      }
      if (ImGui.checkbox(labels.get("Kinematics streaming"), isKSTEnabled))
      {
         setKSTEnabled(isKSTEnabled.get());
      }
      if (ImGui.checkbox(labels.get("Control only arms"), controlArmsOnly))
      {
         KinematicsStreamingToolboxConfigurationMessage newConfiguration = kstParameters.getDefaultConfiguration();
         newConfiguration.setLockPelvis(controlArmsOnly.get());
         newConfiguration.setLockChest(controlArmsOnly.get());
         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStreamingConfigurationTopic(syncedRobot.getRobotModel().getSimpleRobotName()),
                                      newConfiguration);
         setKSTEnabled(false);
      }
      ImGui.checkbox(labels.get("Show ghosts during control"), showGhosts);
      if (ImGui.button(labels.get("Start record/replay")))
      {
         requestRecordReplay.set(true);
      }
      if (ImGui.button(labels.get("Load latest csv")))
      {
         kinematicsRecorder.loadLatestReplayFile();
      }

      Set<String> connectedTrackers = vrContext.getAssignedTrackerRoles();
      if (connectedTrackers.contains(CHEST.getSegmentName()))
      {
         if (ImGui.checkbox(labels.get("Arm Scaling"), armScaling))
         {
            setKSTEnabled(false);
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
            setKSTEnabled(false);
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
      ImGui.text("Output:");
      ImGui.sameLine();
      outputFrequencyPlot.renderImGuiWidgets();
      ImGui.text("Status:");
      ImGui.sameLine();
      statusFrequencyPlot.renderImGuiWidgets();

      ImGui.checkbox(labels.get("Show reference frames"), showReferenceFrameGraphics);
   }

   public void setKSTEnabled(boolean enabled)
   {
      if (enabled)
      {
         if (!isKSTEnabled.get())
         {
            initialize();
            wakeUpToolbox();
            ghostRobotGraphic.setActive(true);

            comPositionInitial.setToZero(syncedRobot.getReferenceFrames().getCenterOfMassFrame());
            comPositionInitial.changeFrame(ReferenceFrame.getWorldFrame());
            comTrackerOffset.setToZero();
            desiredCoMPositionFiltered.set(comPositionInitial);
         }
      }
      else // Disable
      {
         sleepToolbox();
         visualizeIKPreviewGraphic(true);
         setStreamToController(false, false);
      }

      isKSTEnabled.set(enabled);
   }

   public void setStreamToController(boolean enabled, boolean changed)
   {
      if (changed || enabled != streamToController.get())
      {
         if (enabled) // becomes enabled
         {
            userRobotOpacity = robotVisualizer.getOpacity();
         }
         else // becomes disabled
         {
            robotVisualizer.setOpacity(userRobotOpacity);
            robotVisualizer.setActive(true);
            ghostRobotGraphic.setActive(true);
         }
      }

      streamToController.set(enabled);
   }

   private void initialize()
   {
      kinematicsRecorder.setReplay(false); // Check no concurrency replay and streaming
      trackerReferenceFrames.clear();
      motionRetargeting.reset();
      motionRetargeting.setControlArmsOnly(controlArmsOnly.get());
      motionRetargeting.setArmScaling(armScaling.get());
      motionRetargeting.setCoMTracking(comTracking.get());
   }

   private void reinitializeToolboxRobotConfiguration()
   {
      sleepToolbox();
      // Update initial configuration of KST
      KinematicsStreamingToolboxInitialConfigurationMessage initialConfigMessage
            = KinematicsToolboxMessageFactory.initialConfigurationFromFullRobotModel(syncedRobot.getFullRobotModel());
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
      LogTools.warn("Reinitializing KSt configuration");
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
      multiContactStabilityGraphic.getRenderables(renderables, pool);

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
}