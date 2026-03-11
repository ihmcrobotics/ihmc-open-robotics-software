package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.VertexAttributes.Usage;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.HighLevelStateMessage;
import ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage;
import ihmc_common_msgs.msg.dds.WeightMatrix3DMessage;
import imgui.ImGui;
import imgui.flag.ImGuiInputTextFlags;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import imgui.type.ImString;
import net.mgsx.gltf.scene3d.attributes.PBRTextureAttribute;
import org.lwjgl.openvr.InputDigitalActionData;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxContactConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInitialConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import toolbox_msgs.msg.dds.ROS2LogMessage;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.drcRobot.RobotVersion;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.UnitConversions;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.communication.ros2log.ROS2LogRecord;
import us.ihmc.communication.ros2log.ROS2LogReplay;
import us.ihmc.communication.ros2log.ROS2LoggerRequestedState;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.handsros2.HandType;
import us.ihmc.handsros2.abilityHand.AbilityHandModel.AbilityHandJointName;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.motionRetargeting.RetargetingParameters;
import us.ihmc.motionRetargeting.VRTrackedSegmentType;
import us.ihmc.rdx.imgui.ImGuiFrequencyPlot;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2RobotVisualizer;
import us.ihmc.rdx.ui.hands.RDXHandManager;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRHardwareModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelWrapper;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;

import javax.annotation.Nullable;
import java.io.File;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import static us.ihmc.communication.packets.MessageTools.toFrameId;
import static us.ihmc.motionRetargeting.VRTrackedSegmentType.*;

public class RDXVRWholeBodyKinematicStreaming
{
   public static final boolean ENABLE_YO_VARIABLE_TOOLBOX_SERVER = false;
   public static final double FRAME_AXIS_GRAPHICS_LENGTH = 0.2;

   private final RDXVRMultiContact multiContact;
   private final RDXVRHandControl handControl;
   private final SideDependentList<RDXHandControlMode> handControlModes = new SideDependentList<>(RDXHandControlMode.HAND_CONFIGURATION, RDXHandControlMode.HAND_CONFIGURATION);
   private final SideDependentList<Boolean> handHasFingers = new SideDependentList<>(true, true);
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final RDXVRContext vrContext;
   private final RDXROS2RobotVisualizer robotVisualizer;
   private float userRobotOpacity = 1.0f; // store this so we can avoid overriding the user
   private final RDXMultiBodyGraphic ghostRobotGraphic;
   private final RDXVRMiniGhostPreview miniGhostKST;
   private final RDXVRMiniGhostPreview miniGhostReal;
   private boolean miniGhostEnabled;
   private final ImBoolean showGhosts = new ImBoolean(true);
   private final ImBoolean showMiniGhost = new ImBoolean(true);
   private final FullHumanoidRobotModel ghostFullRobotModel;
   private final OneDoFJointBasics[] ghostOneDoFJointsExcludingHands;
   private final int[] ghostOneDoFJointExcludingHandsIndices;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RetargetingParameters retargetingParameters;
   private final ImBoolean isKSTEnabled = new ImBoolean(false);
   private final ImBoolean streamToController = new ImBoolean(false);

   private final ImBoolean demonstrationMode = new ImBoolean(false);
   private int demonstrationTaskIndex = 0;
   private final int[] demonstrationCounts = new int[2];
   private final ImInt performingDemonstration = new ImInt(-1);
   private final ROS2Publisher<ROS2LogMessage> ros2LogMessagePublisher;
   private boolean recordRequest = false;
   private ModelInstance recordingGraphics;

   @Nullable
   private KinematicsStreamingToolboxModule toolbox;
   private final KinematicsStreamingToolboxParameters kstParameters;
   private final KinematicsToolboxConfigurationMessage ikSolverConfigurationMessage = new KinematicsToolboxConfigurationMessage();

   private final ROS2Input<KinematicsToolboxOutputStatus> status;
   private final double streamPeriod = UnitConversions.hertzToSeconds(1000.0);
   private final Throttler toolboxInputStreamRateLimiter = new Throttler();
   private final ImGuiFrequencyPlot statusFrequencyPlot = new ImGuiFrequencyPlot();
   private final ImGuiFrequencyPlot outputFrequencyPlot = new ImGuiFrequencyPlot();
   public long controllerLastPollTimeNanos;

   private final FramePose3D tempFramePose = new FramePose3D();
   private final MutableReferenceFrame headsetReferenceFrame;
   private final SideDependentList<MutableReferenceFrame> handDesiredControlFrames = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> controllerFrameGraphics = new SideDependentList<>();
   private final SideDependentList<Pose3D> ikControlFramePoses = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> handFrameGraphics = new SideDependentList<>();
   private final Map<String, MutableReferenceFrame> trackerReferenceFrames = new HashMap<>();
   private final Map<String, RDXReferenceFrameGraphic> trackerFrameGraphics = new HashMap<>();
   private final RDXReferenceFrameGraphic chestFrameGraphics = new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH);
   private final SideDependentList<RDXReferenceFrameGraphic> wristFrameGraphics = new SideDependentList<>();
   private final ImBoolean showReferenceFrameGraphics = new ImBoolean(false);
   private final Throttler messageThrottler = new Throttler();

   private final ImBoolean controlArmsOnly = new ImBoolean(false);
   private final ImBoolean lockPelvis = new ImBoolean(false);
   private final ImBoolean armScaling = new ImBoolean(false);
   private final ImBoolean comTracking = new ImBoolean(true);
   private final RDXVRMotionRetargeting motionRetargeting;

   private ReferenceFrame initialPelvisFrame;
   private final RigidBodyTransform initialPelvisTransformToWorld = new RigidBodyTransform();
   private ReferenceFrame initialChestFrame;
   private final RigidBodyTransform initialChestTransformToWorld = new RigidBodyTransform();

   private final ImBoolean replayMotion = new ImBoolean(false);
   private final ImBoolean pauseReplay = new ImBoolean(true);
   private final ROS2LogReplay replayer;
   private final ImString logDirectory = new ImString(System.getProperty("user.home") + "/.ihmc/logs/ros2/");
   private final ImString logFileName = new ImString();
   private volatile boolean replayThreadRunning = false;
   private Thread replayThread;
   private final float[] replaySpeed = {1.0f};

   public RDXVRWholeBodyKinematicStreaming(ROS2SyncedRobotModel syncedRobot,
                                           ROS2ControllerHelper ros2ControllerHelper,
                                           RDXROS2RobotVisualizer robotVisualizer,
                                           RDXVRContext vrContext,
                                           RetargetingParameters retargetingParameters,
                                           KinematicsStreamingToolboxParameters kstParameters,
                                           boolean createToolbox,
                                           boolean recordKSTOutput,
                                           RDXHandManager handManager,
                                           RobotDefinition miniGhostRobotDefinition,
                                           ROS2LogReplay replayer)
   {
      this.syncedRobot = syncedRobot;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.retargetingParameters = retargetingParameters;
      this.vrContext = vrContext;
      this.robotVisualizer = robotVisualizer;
      this.kstParameters = kstParameters;
      this.replayer = replayer;

      RobotDefinition ghostRobotDefinition = new RobotDefinition(syncedRobot.getRobotModel().getRobotDefinition());
      MaterialDefinition material = new MaterialDefinition(ColorDefinitions.parse("0xDEE934").derive(0.0, 1.0, 1.0, 0.5));
      RobotDefinition.forEachRigidBodyDefinition(ghostRobotDefinition.getRootBodyDefinition(),
                                                 body -> body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(material)));

      RobotVersion robotVersion = syncedRobot.getRobotModel().getRobotVersion();
      ghostFullRobotModel = syncedRobot.getRobotModel().createFullRobotModel();
      ghostOneDoFJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(ghostFullRobotModel);
      HumanoidJointNameMap jointMap = syncedRobot.getRobotModel().getJointMap();
      OneDoFJointBasics[] oneDoFJoints = ghostFullRobotModel.getOneDoFJoints();
      Set<String> fingerNames = new HashSet<>();
      for (RobotSide side : RobotSide.values)
         if (robotVersion.getHandType(side) == HandType.ABILITY_HAND)
            for (AbilityHandJointName jointName : AbilityHandJointName.values)
               fingerNames.add(jointName.getJointName(side));
      int j = 0;
      ghostOneDoFJointExcludingHandsIndices = new int[oneDoFJoints.length - fingerNames.size()];
      for (int i = 0; i < oneDoFJoints.length; i++)
         if (!fingerNames.contains(oneDoFJoints[i].getName()))
            ghostOneDoFJointExcludingHandsIndices[j++] = i;
      ghostRobotGraphic = new RDXMultiBodyGraphic(syncedRobot.getRobotModel().getSimpleRobotName() + " (IK Preview Ghost)");
      ghostRobotGraphic.loadRobotModelAndGraphics(ghostRobotDefinition, ghostFullRobotModel.getElevator());
      ghostRobotGraphic.setActive(true);
      ghostRobotGraphic.create();

      if (miniGhostRobotDefinition != null)
      {
         RobotDefinition.forEachRigidBodyDefinition(miniGhostRobotDefinition.getRootBodyDefinition(),
                                                    body -> body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(material)));

         FullHumanoidRobotModel miniGhostFullRobotModel = new FullHumanoidRobotModelWrapper(miniGhostRobotDefinition, jointMap, false);
         miniGhostKST = new RDXVRMiniGhostPreview(syncedRobot.getRobotModel().getSimpleRobotName() + " KST",
                                                  miniGhostRobotDefinition,
                                                  miniGhostFullRobotModel,
                                                  vrContext,
                                                  Color.YELLOW,
                                                  0.2f);

         FullHumanoidRobotModel miniGhostRealModel = new FullHumanoidRobotModelWrapper(miniGhostRobotDefinition, jointMap, false);
         miniGhostReal = new RDXVRMiniGhostPreview(syncedRobot.getRobotModel().getSimpleRobotName() + " Real",
                                                   miniGhostRobotDefinition,
                                                   miniGhostRealModel,
                                                   vrContext,
                                                   Color.BLACK,
                                                   0.8f);
         miniGhostEnabled = true;
      }
      else
      {
         miniGhostKST = null;
         miniGhostReal = null;
         miniGhostEnabled = false;
      }

      if (recordKSTOutput)
      {
         ModelBuilder modelBuilder = new ModelBuilder();
         Pixmap pixmap = new Pixmap(1, 1, Pixmap.Format.RGBA8888);
         pixmap.setColor(Color.RED);
         pixmap.fill();
         Texture redTexture = new Texture(pixmap);
         Material redMaterial = new Material(PBRTextureAttribute.createBaseColorTexture(redTexture));
         Model circleModel = modelBuilder.createSphere(0.0001f, 0.015f, 0.015f, 20, 20, redMaterial, Usage.Position);
         recordingGraphics = new ModelInstance(circleModel);
      }

      for (RobotSide side : RobotSide.values)
      {
         handFrameGraphics.put(side, new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
         controllerFrameGraphics.put(side, new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
         wristFrameGraphics.put(side, new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
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
      motionRetargeting = new RDXVRMotionRetargeting(syncedRobot.getFullRobotModel(), ghostFullRobotModel, handDesiredControlFrames, trackerReferenceFrames,
                                                     headsetReferenceFrame, retargetingParameters);

      status = ros2ControllerHelper.subscribe(KinematicsStreamingToolboxModule.getOutputStatusTopic(syncedRobot.getRobotModel().getSimpleRobotName()));
      ros2LogMessagePublisher = ros2ControllerHelper.getROS2Node().createPublisher(ROS2LogRecord.getROS2LogTopic());
      if (createToolbox)
      {
         toolbox = new KinematicsStreamingToolboxModule(syncedRobot.getRobotModel(), kstParameters, ENABLE_YO_VARIABLE_TOOLBOX_SERVER);
      }

      if (vrContext.getVRModel() == RDXVRHardwareModel.FOCUS3)
      {
         RDXBaseUI.getInstance().getKeyBindings().register("Enable IK preview", "A button");
         RDXBaseUI.getInstance().getKeyBindings().register("Control robot", "X button");
         RDXBaseUI.getInstance().getKeyBindings().register("Start/stop recording motion", "B button");
         RDXBaseUI.getInstance().getKeyBindings().register("Mark demonstration (hold) (enable in menu)", "B button");
         RDXBaseUI.getInstance().getKeyBindings().register("Cycle demonstration task", "Y button");
      }
      else
      {
         RDXBaseUI.getInstance().getKeyBindings().register("Enable IK preview", "Right A button");
         RDXBaseUI.getInstance().getKeyBindings().register("Control robot", "Left A button");
         RDXBaseUI.getInstance().getKeyBindings().register("Start/stop recording motion", "Right B button");
         RDXBaseUI.getInstance().getKeyBindings().register("Mark demonstration (hold) (enable in menu)", "Right B button");
         RDXBaseUI.getInstance().getKeyBindings().register("Cycle demonstration task (enable in menu)", "Left B button");
         RDXBaseUI.getInstance().getKeyBindings().register("Process user motion", "Left B button");
      }

      for (RobotSide side : RobotSide.values)
      {
         if (!robotVersion.hasHandWithFingers(side))
         {
            handControlModes.put(side, RDXHandControlMode.NONE);
            handHasFingers.put(side, false);
         }
      }
      handControl = new RDXVRHandControl(vrContext, handManager, streamToController, handControlModes, robotVersion);
      multiContact = new RDXVRMultiContact(syncedRobot, ghostFullRobotModel, ros2ControllerHelper, vrContext, streamPeriod, streamToController, handControlModes);
   }

   private void handleRecordRequest()
   {
      recordRequest = !recordRequest;
      ROS2LogMessage message = new ROS2LogMessage();
      message.setRequestedState((recordRequest ? ROS2LoggerRequestedState.START : ROS2LoggerRequestedState.FINISH).toByte());
      ros2LogMessagePublisher.publish(message);
   }

   public void processVRInput()
   {
      vrContext.getController(RobotSide.LEFT).runIfConnected(controller ->
      {
         controller.setAButtonText(streamToController.get() ? "Stop control" : "Start control");
         if (demonstrationMode.get())
            controller.setBButtonText("Task %d: x%d".formatted(demonstrationTaskIndex, demonstrationCounts[demonstrationTaskIndex]));

         InputDigitalActionData aButton = controller.getAButtonActionData();
         InputDigitalActionData bButton = controller.getBButtonActionData();
         boolean leftAButtonPressed = aButton.bChanged() && !aButton.bState();
         boolean leftBButtonPressed = bButton.bChanged() && !bButton.bState();
         handleLeftControllerJoystickInput(leftAButtonPressed, leftBButtonPressed);

         controllerLastPollTimeNanos = controller.getLastPollTimeNanos();
      });

      vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
      {
         controller.setAButtonText(isKSTEnabled.get() ? "Stop preview" : "Start preview");
         if (demonstrationMode.get())
            controller.setBButtonText("%s demonstrating".formatted(performingDemonstration.get() == -1 ? "Start" : "Stop"));
         else
            controller.setBButtonText("Record motion");

         InputDigitalActionData aButton = controller.getAButtonActionData();
         InputDigitalActionData bButton = controller.getBButtonActionData();
         boolean rightAButtonPressed = aButton.bChanged() && !aButton.bState();
         boolean rightBButtonPressed = bButton.bChanged() && !bButton.bState();
         boolean rightBButtonState = bButton.bState();
         handleRightControllerJoystickInput(rightAButtonPressed, rightBButtonPressed, rightBButtonState);

         controllerLastPollTimeNanos = controller.getLastPollTimeNanos();
      });

      multiContact.processVRInput();
      handControl.processVRInput();
      
      if (isKSTEnabled.get() && toolboxInputStreamRateLimiter.run(streamPeriod) && !replayMotion.get())
      {
         KinematicsStreamingToolboxInputMessage toolboxInputMessage = new KinematicsStreamingToolboxInputMessage();
         processControllers(toolboxInputMessage);
         processTrackers(toolboxInputMessage);
         processHeadset(toolboxInputMessage);
         retargetMotion(toolboxInputMessage);
         multiContact.doCoMControl(toolboxInputMessage);

         if (controlArmsOnly.get())
         { // If option 'Control Arms Only' is active, lock pelvis and chest to current pose
            lockChest(toolboxInputMessage);
            lockPelvis(toolboxInputMessage);
         }

         if (lockPelvis.get())
         {
            lockPelvis(toolboxInputMessage);
         }

         if (isKSTEnabled.get())
            toolboxInputMessage.setStreamToController(streamToController.get());
         toolboxInputMessage.setDemonstrationTaskId(performingDemonstration.get());
         toolboxInputMessage.setTimestamp(controllerLastPollTimeNanos);

         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputToolboxConfigurationTopic(syncedRobot.getRobotModel().getSimpleRobotName()),
                                      ikSolverConfigurationMessage);
         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputCommandTopic(syncedRobot.getRobotModel().getSimpleRobotName()),
                                      toolboxInputMessage);
         outputFrequencyPlot.recordEvent();
      }
   }

   private void handleLeftControllerJoystickInput(boolean leftAButtonPressed, boolean leftBButtonPressed)
   {
      if (isKSTEnabled.get() && leftAButtonPressed)
      {
         setStreamToController(!streamToController.get(), true);
      }

      if (leftBButtonPressed)
      {
         if (demonstrationMode.get())
            demonstrationTaskIndex = (demonstrationTaskIndex + 1) % demonstrationCounts.length;
      }
   }

   private void handleRightControllerJoystickInput(boolean rightAButtonPressed, boolean rightBButtonPressed, boolean rightBButtonState)
   {
      if (rightAButtonPressed)
      {
         setKSTEnabled(!isKSTEnabled.get());
      }

      if (rightBButtonPressed)
      {
         if (demonstrationMode.get())
         {
            if (performingDemonstration.get() > -1) // demonstration completed
               ++demonstrationCounts[demonstrationTaskIndex];
            performingDemonstration.set(performingDemonstration.get() == -1 ? demonstrationTaskIndex : -1);
         }
         else
            handleRecordRequest();
      }
   }

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
      message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(100));

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
      message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(100));
      message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(100));

      toolboxInputMessage.getInputs().add().set(message);
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

   private void processTrackers(KinematicsStreamingToolboxInputMessage messageToPack)
   {
      Set<String> additionalTrackedSegments = vrContext.getAssignedTrackerRoles();
      for (VRTrackedSegmentType segmentType : VRTrackedSegmentType.TRACKER_TYPES)
      {
         if (additionalTrackedSegments.contains(segmentType.getSegmentName()))
         {
            FramePose3D desiredPose = new FramePose3D();
            FrameVector3D desiredAngularVelocity = new FrameVector3D();
            FrameVector3D desiredLinearVelocity = new FrameVector3D();

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
                  if (segmentType.isWristRelated())
                     wristFrameGraphics.get(segmentType.getSegmentSide()).setToReferenceFrame(ghostFullRobotModel.getForearm(segmentType.getSegmentSide()).getBodyFixedFrame());
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
               motionRetargeting.setDesiredVelocities(segmentType, desiredAngularVelocity, desiredLinearVelocity);
            });

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

   private void processHeadset(KinematicsStreamingToolboxInputMessage messageToPack)
   {
      vrContext.getHeadset().runIfConnected(headset->
      {
         RigidBodyBasics controlledSegment = getControlledSegment(HEAD);
         if (controlledSegment != null)
         {
            FramePose3D desiredPose = new FramePose3D();
            desiredPose.setToZero(headset.getXForwardZUpHeadsetFrame());
            desiredPose.changeFrame(ReferenceFrame.getWorldFrame());
            KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(controlledSegment,
                                                                               desiredPose,
                                                                               headset.getAngularVelocity(),
                                                                               headset.getLinearVelocity(),
                                                                               retargetingParameters.getPositionWeight(HEAD),
                                                                               retargetingParameters.getOrientationWeight(HEAD),
                                                                               retargetingParameters.getLinearRateLimitation(HEAD),
                                                                               retargetingParameters.getAngularRateLimitation(HEAD));
            messageToPack.getInputs().add().set(message);
         }
      });
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
                                                                               motionRetargeting.getDesiredAngularVelocity(segmentType),
                                                                               motionRetargeting.getDesiredLinearVelocity(segmentType),
                                                                               retargetingParameters.getPositionWeight(segmentType),
                                                                               retargetingParameters.getOrientationWeight(segmentType),
                                                                               retargetingParameters.getLinearRateLimitation(segmentType),
                                                                               retargetingParameters.getAngularRateLimitation(segmentType));
            if (segmentType.isHandRelated())
            {
               // Check arm scaling state not changed -> disabled
               if (!isKSTEnabled.get())
                  return;
               message.getControlFramePositionInEndEffector().set(ikControlFramePoses.get(segmentType.getSegmentSide()).getPosition());
               message.getControlFrameOrientationInEndEffector().set(ikControlFramePoses.get(segmentType.getSegmentSide()).getOrientation());
            }
            else
            {
               message.getControlFramePositionInEndEffector().set(motionRetargeting.getIKControlFramePosition(segmentType));
            }
            messageToPack.getInputs().add().set(message);
         }
      }
      if (motionRetargeting.isCenterOfMassAvailable())
      {   // If using ankles and waist tracker, create a CoM message for the toolbox
         KinematicsToolboxCenterOfMassMessage comMessage = new KinematicsToolboxCenterOfMassMessage();
         comMessage.setHasDesiredLinearVelocity(false);
         // Add offset from KST parameters
         FramePose3D centerOfMassDesiredWithOffset = new FramePose3D(ReferenceFrame.getWorldFrame(), motionRetargeting.getDesiredCenterOfMassXYInWorld(), new YawPitchRoll());
         centerOfMassDesiredWithOffset.changeFrame(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
         centerOfMassDesiredWithOffset.appendTranslation(kstParameters.getCenterOfMassOffset().getX(), kstParameters.getCenterOfMassOffset().getY(), 0.0);
         centerOfMassDesiredWithOffset.changeFrame(ReferenceFrame.getWorldFrame());

         comMessage.getDesiredPositionInWorld().set(centerOfMassDesiredWithOffset.getPosition());
         comMessage.getSelectionMatrix().setSelectionFrameId(toFrameId(ReferenceFrame.getWorldFrame()));
         comMessage.getSelectionMatrix().setXSelected(true);
         comMessage.getSelectionMatrix().setYSelected(true);
         comMessage.getSelectionMatrix().setZSelected(false);
         comMessage.getWeights().setXWeight(kstParameters.getCenterOfMassTrackingWeight());
         comMessage.getWeights().setYWeight(kstParameters.getCenterOfMassTrackingWeight());

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
         case HEAD -> ghostFullRobotModel.getHead();
         case CHEST -> ghostFullRobotModel.getChest();
         case WAIST -> ghostFullRobotModel.getPelvis();
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

   public void update()
   {
      if (!isKSTEnabled.get())
      {
         setStreamToController(false, false);
      }

      if (isKSTEnabled.get())
      {
         if (streamToController.get())
         {
            ghostRobotGraphic.setActive(showGhosts.get());
            robotVisualizer.setActive(showGhosts.get());
            if (showGhosts.get())
               robotVisualizer.setOpacity(0.5f);
         }
         if (miniGhostEnabled)
         {
            miniGhostKST.setActive(showGhosts.get() && showMiniGhost.get());
            miniGhostReal.setActive(showGhosts.get() && showMiniGhost.get());
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
                  ghostOneDoFJointsExcludingHands[i].setQ(latestStatus.getDesiredJointAngles().get(ghostOneDoFJointExcludingHandsIndices[i]));
                  if (miniGhostEnabled)
                  {
                     miniGhostKST.setJoint(i, latestStatus.getDesiredJointAngles().get(i));
                     OneDoFJointBasics realJoint = syncedRobot.getFullRobotModel().getOneDoFJoints()[i];
                     double realQ = realJoint.getQ();
                     miniGhostReal.setJoint(i, realQ);
                  }
               }
               ghostFullRobotModel.getElevator().updateFramesRecursively();
            }
            multiContact.update(latestStatus);

            if (miniGhostEnabled)
            {
               miniGhostKST.updateColor(latestStatus.getLeftFootInContact(), latestStatus.getRightFootInContact());
               miniGhostKST.updatePose();
               miniGhostReal.updateRootOffset(ghostFullRobotModel.getPelvis().getBodyFixedFrame().getTransformToRoot(),
                                              syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame().getTransformToRoot());
               miniGhostReal.updatePose();
            }
         }

         if (ghostRobotGraphic.isActive())
            ghostRobotGraphic.update();

         if (recordRequest && recordingGraphics != null)
         {
            FramePose3D recordingGraphicsPose = new FramePose3D(headsetReferenceFrame.getReferenceFrame());
            recordingGraphicsPose.getTranslation().add(0.3, 0.2, 0.1);
            recordingGraphicsPose.changeFrame(ReferenceFrame.getWorldFrame());
            RigidBodyTransform graphicsTransform  = new RigidBodyTransform();
            recordingGraphicsPose.get(graphicsTransform);
            LibGDXTools.toLibGDX(graphicsTransform, recordingGraphics.transform);
         }
      }
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.checkbox(labels.get("Enable Kinematics Streaming"), isKSTEnabled))
      {
         setKSTEnabled(isKSTEnabled.get());
      }
      if (ImGui.checkbox(labels.get("Control Robot"), streamToController))
      {
         setStreamToController(streamToController.get(), true);
      }
      ImGui.checkbox(labels.get("Control Arms Only"), controlArmsOnly);
      ImGui.checkbox(labels.get("Lock Pelvis"), lockPelvis);
      Set<String> connectedTrackers = vrContext.getAssignedTrackerRoles();
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

      ImGuiTools.separatorText("Visualization Options", ImGuiTools.getSmallBoldFont());
      ImGui.checkbox(labels.get("Show Robot Ghosts during Control"), showGhosts);
      if (miniGhostEnabled)
      {
         ImGui.checkbox(labels.get("Show Mini Robot Ghost"), showMiniGhost);
      }
      ImGui.checkbox(labels.get("Show Reference Frames"), showReferenceFrameGraphics);

      if (replayer != null)
      {
         ImGuiTools.separatorText("Replay Options", ImGuiTools.getSmallBoldFont());
         if (ImGui.inputText("Log Directory", logDirectory, ImGuiInputTextFlags.EnterReturnsTrue)
             || ImGui.inputText("Log File Name", logFileName, ImGuiInputTextFlags.EnterReturnsTrue))
         {
            replayer.load(new File(logDirectory.get(), logFileName.get()));
         }

         if (!replayer.isReady())
         {
            ImGui.beginDisabled();
            replayMotion.set(false);
         }
         if (ImGui.checkbox(labels.get("Enable replay"), replayMotion))
         {
            setKSTEnabled(replayMotion.get());
            replayer.reset();
            if (replayMotion.get())
            {
               replayer.pauseReplay(pauseReplay.get());
               startReplayThread();
            }
            else
            {
               stopReplayThread();
            }
         }
         if (!replayer.isReady())
            ImGui.endDisabled();

         if (!replayMotion.get())
         {
            ImGui.beginDisabled();
            pauseReplay.set(true);
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get(pauseReplay.get() ? "Resume" : "Pause")))
         {
            pauseReplay.set(!pauseReplay.get());
            replayer.pauseReplay(pauseReplay.get());
         }
         if (!replayMotion.get())
            ImGui.endDisabled();

         if (!pauseReplay.get())
            ImGui.beginDisabled();
         if (ImGui.sliderFloat("Replay speed", replaySpeed, 0.1f, 2.0f, "%.1f"))
            replayer.setReplaySpeed(replaySpeed[0]);
         if (!pauseReplay.get())
            ImGui.endDisabled();
      }

      ImGuiTools.separatorText("Utility Functions", ImGuiTools.getSmallBoldFont());
      if (ImGui.button(labels.get("Wakeup Toolbox")))
      {
         wakeUpToolbox();
      }
      if (ImGui.button(labels.get("Sleep Toolbox")))
      {
         sleepToolbox();
      }
      if (ImGui.button(labels.get("Reinitialize Toolbox Configuration")))
      {
         reinitializeToolboxRobotConfiguration();
      }
      if (ImGui.checkbox(labels.get("Demonstration Mode"), demonstrationMode))
      {
         demonstrationTaskIndex = 0;
         demonstrationCounts[0] = 0;
         demonstrationCounts[1] = 0;
      }

      ImGuiTools.separatorText("Hand Control Mode", ImGuiTools.getSmallBoldFont());
      for (RobotSide side : RobotSide.values)
      {
         ImGui.text(side.getCamelCaseName() + " Hand:");
         if (ImGui.radioButton(labels.get("None", side.ordinal()), handControlModes.get(side) == RDXHandControlMode.NONE))
         {
            handControlModes.put(side, RDXHandControlMode.NONE);
         }
         if (!handHasFingers.get(side))
         {
            ImGui.beginDisabled();
         }
         if (ImGui.radioButton(labels.get("Hand Configuration", side.ordinal()), handControlModes.get(side) == RDXHandControlMode.HAND_CONFIGURATION))
         {
            handControlModes.put(side, RDXHandControlMode.HAND_CONFIGURATION);
         }
         if (ImGui.radioButton(labels.get("Finger Streaming", side.ordinal()), handControlModes.get(side) == RDXHandControlMode.FINGER_STREAMING))
         {
            handControlModes.put(side, RDXHandControlMode.FINGER_STREAMING);
         }
         if (!handHasFingers.get(side))
         {
            ImGui.endDisabled();
         }
         if (ImGui.radioButton(labels.get("Load Bearing", side.ordinal()), handControlModes.get(side) == RDXHandControlMode.LOAD_BEARING))
         {
            handControlModes.put(side, RDXHandControlMode.LOAD_BEARING);
         }
         ImGui.separator();
      }
   }

   public void setKSTEnabled(boolean enabled)
   {
      if (enabled)
      {
         initialize();
         reinitializeToolboxRobotConfiguration();
         ghostRobotGraphic.setActive(true);
         if (miniGhostEnabled)
         {
            miniGhostKST.setActive(true);
            miniGhostReal.setActive(true);
         }
      }
      else // Disable
      {
         sleepToolbox();
         ghostRobotGraphic.setActive(false);
         if (miniGhostEnabled)
         {
            miniGhostKST.setActive(false);
            miniGhostReal.setActive(false);
         }
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
            if (miniGhostEnabled)
            {
               miniGhostKST.setActive(true);
               miniGhostReal.setActive(true);
            }
            performingDemonstration.set(-1);
         }
         sendRLStateTransitionRequest(enabled);
      }

      streamToController.set(enabled);
   }

   private void initialize()
   {
      recordRequest = false;
      trackerReferenceFrames.clear();
      motionRetargeting.reset();
      motionRetargeting.setControlArmsOnly(controlArmsOnly.get());
      motionRetargeting.setArmScaling(armScaling.get());
      motionRetargeting.setCoMTracking(comTracking.get());
      multiContact.reset();
      initialPelvisFrame = null;
      initialChestFrame = null;
   }

   private void sendRLStateTransitionRequest(boolean activate)
   {
      HighLevelStateMessage highLevelStateMessage = new HighLevelStateMessage();
      if (activate)
      {
         highLevelStateMessage.setHighLevelControllerName(HighLevelControllerName.RL_TRANSITION_STATE.toByte());
      }
      else
      {
         highLevelStateMessage.setHighLevelControllerName(HighLevelControllerName.EXIT_RL.toByte());
      }
      ros2ControllerHelper.publishToController(highLevelStateMessage);
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
         List<ArmJointName> armJointNames = Arrays.asList(syncedRobot.getFullRobotModel().getRobotSpecificJointNames().getArmJointNames());
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
      ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStreamingInitialConfigurationTopic(syncedRobot.getRobotModel().getSimpleRobotName()),
                                   initialConfigMessage);
      reinitializeToolbox();
      wakeUpToolbox();
      LogTools.warn("Reinitializing initial KST configuration");
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
         if (miniGhostKST != null)
         {
            miniGhostKST.getRenderables(renderables, pool, sceneLevels);
            miniGhostReal.getRenderables(renderables, pool, sceneLevels);
         }
      }

      if (showReferenceFrameGraphics.get())
      {
         for (RobotSide side : RobotSide.values)
         {
            controllerFrameGraphics.get(side).getRenderables(renderables, pool);
            handFrameGraphics.get(side).getRenderables(renderables, pool);
            wristFrameGraphics.get(side).getRenderables(renderables, pool);
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

      multiContact.getRenderables(renderables, pool);
      if (recordRequest && recordingGraphics != null)
         recordingGraphics.getRenderables(renderables, pool);
   }

   /**
    * Configures the hand control modes, see {@link RDXHandControlMode} for options
    */
   public void setVRHandConfiguration(RDXHandControlMode leftHandMode, RDXHandControlMode rightHandMode)
   {
      handControlModes.set(RobotSide.LEFT, leftHandMode);
      handControlModes.set(RobotSide.RIGHT, rightHandMode);
   }

   public void destroy()
   {
      if (toolbox != null)
         toolbox.closeAndDispose();
      ghostRobotGraphic.destroy();
      if (miniGhostEnabled)
      {
         miniGhostKST.destroy();
         miniGhostReal.destroy();
      }
      chestFrameGraphics.dispose();
      for (RobotSide side : RobotSide.values)
      {
         controllerFrameGraphics.get(side).dispose();
         handFrameGraphics.get(side).dispose();
         wristFrameGraphics.get(side).dispose();
      }
      if (replayThreadRunning)
      {
         stopReplayThread();
      }
   }

   public void startReplayThread()
   {
      if (replayThreadRunning)
         return; // Already running
      replayThreadRunning = true;
      replayThread = new Thread(() ->
      {
        final long intervalNanos = 1_000_000; // 1 kHz: 1 ms = 1,000,000 nanoseconds
        while (replayThreadRunning)
        {
           long start = System.nanoTime();
           try
           {
              if (replayer != null && replayer.isReady() && replayMotion.get())
              {
                 if (replayer.doIncrementalReplay())
                 {
                    replayMotion.set(false);
                    replayer.reset();
                    LogTools.info("Replay completed successfully");
                 }
              }
           }
           catch (Exception e)
           {
              LogTools.error("Replay thread error: " + e.getMessage());
           }

           long elapsed = System.nanoTime() - start;
           long remaining = intervalNanos - elapsed;
           if (remaining > 0)
           {
              try
              {
                 Thread.sleep(remaining / 1_000_000, (int) (remaining % 1_000_000));
              }
              catch (InterruptedException ignored)
              {
              }
           }
        }
      });
      replayThread.setDaemon(true); // Stops with main app
      replayThread.start();
   }

   public void stopReplayThread()
   {
      replayThreadRunning = false;
      if (replayThread != null)
      {
         replayThread.interrupt();
         replayThread = null;
      }
   }

   public FullHumanoidRobotModel getGhostFullRobotModel()
   {
      return ghostFullRobotModel;
   }

   public RDXMultiBodyGraphic getGhostRobotGraphic()
   {
      return ghostRobotGraphic;
   }

   public boolean getStreamToController()
   {
      return streamToController.get();
   }

   public boolean getShowGhosts()
   {
      return showGhosts.get();
   }
}