package us.ihmc.rdx.ui.remoteCaptury;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import gnu.trove.map.hash.TLongObjectHashMap;
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
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters.InputStateEstimatorType;
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
import us.ihmc.motionRetargeting.CapturyTrackedSegmentType;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.imgui.ImGuiFrequencyPlot;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.tools.KinematicsRecordReplay;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
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
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

public class RDXCapturyKinematicsStreaming
{
   private static final double FRAME_AXIS_GRAPHICS_LENGTH = 0.2;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final RetargetingParameters retargetingParameters;
   private final DRCRobotModel robotModel;
   private RDXMultiBodyGraphic ghostRobotGraphic;
   private FullHumanoidRobotModel ghostFullRobotModel;
   private SideDependentList<RigidBodyBasics> ghostUpperArms = new SideDependentList<>();
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
   private final SideDependentList<Pose3D> ikUpperArmControlFramePoses = new SideDependentList<>();
   private final SideDependentList<Pose3D> ikForearmControlFramePoses = new SideDependentList<>();
   private final Pose3D ikChestControlFramePoses = new Pose3D();
   private final SideDependentList<RDXReferenceFrameGraphic> ikHandControlFrameGraphics = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> ikUpperArmControlFrameGraphics = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> ikForearmControlFrameGraphics = new SideDependentList<>();
   private final RDXReferenceFrameGraphic ikChestControlFrameGraphics = new RDXReferenceFrameGraphic(
         1.25 * FRAME_AXIS_GRAPHICS_LENGTH);
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

   private final HandConfiguration[] handConfigurations = {HandConfiguration.HALF_CLOSE,
                                                           HandConfiguration.CRUSH,
                                                           HandConfiguration.CLOSE};
   private int leftIndex = -1;
   private int rightIndex = -1;

   private final AtomicReference<KinematicsStreamingToolboxInputMessage> toolboxInputMessagePending = new AtomicReference<>(
         null);

   public RDXCapturyKinematicsStreaming(ROS2SyncedRobotModel syncedRobot,
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

   public void create(boolean createToolbox)
   {
      RobotDefinition ghostRobotDefinition = new RobotDefinition(syncedRobot.getRobotModel().getRobotDefinition());
      MaterialDefinition material = new MaterialDefinition(ColorDefinitions.parse("0xDEE934")
                                                                           .derive(0.0, 1.0, 1.0, 0.5));
      RobotDefinition.forEachRigidBodyDefinition(ghostRobotDefinition.getRootBodyDefinition(),
                                                 body -> body.getVisualDefinitions()
                                                             .forEach(visual -> visual.setMaterialDefinition(material)));

      ghostFullRobotModel = syncedRobot.getRobotModel().createFullRobotModel();
      ghostOneDoFJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(ghostFullRobotModel);
      ghostRobotGraphic = new RDXMultiBodyGraphic(
            syncedRobot.getRobotModel().getSimpleRobotName() + " (IK Preview Ghost)");
      ghostRobotGraphic.loadRobotModelAndGraphics(ghostRobotDefinition, ghostFullRobotModel.getElevator());
      ghostRobotGraphic.setActive(true);
      ghostRobotGraphic.create();

      for (RobotSide side : RobotSide.values)
      {
         // TODO Kinda cheating here to find the upper-arm. Parametrize me?
         RigidBodyBasics upperArm = ghostFullRobotModel.getArmJoint(side, ArmJointName.ELBOW_PITCH).getPredecessor();
         ghostUpperArms.put(side, upperArm);

         handFrameGraphics.put(side, new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
         controllerFrameGraphics.put(side, new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
         handDesiredControlFrames.put(side, new MutableReferenceFrame(ReferenceFrame.getWorldFrame()));

         ikHandControlFrameGraphics.put(side, new RDXReferenceFrameGraphic(1.25 * FRAME_AXIS_GRAPHICS_LENGTH));
         ikUpperArmControlFrameGraphics.put(side, new RDXReferenceFrameGraphic(1.25 * FRAME_AXIS_GRAPHICS_LENGTH));
         ikForearmControlFrameGraphics.put(side, new RDXReferenceFrameGraphic(1.25 * FRAME_AXIS_GRAPHICS_LENGTH));

         Pose3D ikControlFramePose = new Pose3D();
         if (side == RobotSide.LEFT)
         {
            ikControlFramePose.getPosition()
                              .setAndNegate(retargetingParameters.getTranslationFromTracker(CapturyTrackedSegmentType.LEFT_HAND));
            ikControlFramePose.getOrientation()
                              .setAndInvert(retargetingParameters.getYawPitchRollFromTracker(CapturyTrackedSegmentType.LEFT_HAND));
         }
         else
         {
            ikControlFramePose.getPosition()
                              .setAndNegate(retargetingParameters.getTranslationFromTracker(CapturyTrackedSegmentType.RIGHT_HAND));
            ikControlFramePose.getOrientation()
                              .setAndInvert(retargetingParameters.getYawPitchRollFromTracker(CapturyTrackedSegmentType.RIGHT_HAND));
         }
         ikHandControlFramePoses.put(side, ikControlFramePose);

         // TODO Figure out how to set this up with NadiaRetargetingParameters
         Pose3D ikUpperArmControlFramePose = new Pose3D();
         Pose3D ikForearmControlFramePose = new Pose3D();
         ikUpperArmControlFramePoses.put(side, ikUpperArmControlFramePose);
         ikForearmControlFramePoses.put(side, ikForearmControlFramePose);
         if(side == RobotSide.LEFT)
         {
            ikHandControlFramePoses.get(side).getPosition().set(0.0, 0.0, 0.0);
            ikHandControlFramePoses.get(side).getOrientation().setYawPitchRoll(3 * Math.PI / 2, 0, Math.PI / 2);
            ikUpperArmControlFramePoses.get(side).getPosition().set(0.0, 0.0, 0.0);
            ikUpperArmControlFramePoses.get(side).getOrientation().setYawPitchRoll(Math.PI, 0, Math.PI / 2);
            ikForearmControlFramePoses.get(side).getPosition().set(0.0, 0.0, 0.1);
            ikForearmControlFramePoses.get(side).getOrientation().setYawPitchRoll(3 * Math.PI / 2, 0, Math.PI / 2);
            ikChestControlFramePoses.getPosition().set(0.4, 0.0, 0.1);
            ikChestControlFramePoses.getOrientation().setYawPitchRoll(0.0, Math.PI, Math.PI);
         }
         else
         {
            ikHandControlFramePoses.get(side).getPosition().set(0.0, 0.0, 0.0);
            ikHandControlFramePoses.get(side).getOrientation().setYawPitchRoll(Math.PI, 0, -Math.PI / 2);
            ikUpperArmControlFramePoses.get(side).getPosition().set(0.0, 0.0, 0.0);
            ikUpperArmControlFramePoses.get(side).getOrientation().setYawPitchRoll(Math.PI, 0, -Math.PI / 2);
            ikForearmControlFramePoses.get(side).getPosition().set(0.0, 0.0, 0.1);
            ikForearmControlFramePoses.get(side).getOrientation().setYawPitchRoll(Math.PI, 0, -Math.PI / 2);
         }

      }

      status = ros2ControllerHelper.subscribe(KinematicsStreamingToolboxModule.getOutputStatusTopic(syncedRobot.getRobotModel()
                                                                                                               .getSimpleRobotName()));

      kinematicsRecorder = new KinematicsRecordReplay(sceneGraph, enabled);

      KinematicsStreamingToolboxParameters parameters = new KinematicsStreamingToolboxParameters();
      parameters.setDefault();
      parameters.setPublishingPeriod(0.015); // Publishing period in seconds.
      parameters.setDefaultChestMessageAngularWeight(1.0, 1.0, 0.5);
      // That locks the pelvis in place, lower to gain some more freedom
      parameters.setDefaultPelvisMessageLinearWeight(1.0, 1.0, 1.0);
      // Max velocity
      parameters.setDefaultLinearRateLimit(100.0);
      // Max angular velocity
      parameters.setDefaultAngularRateLimit(1000.0);
      // In case you don't send a weight with your commands
      parameters.setDefaultLinearWeight(10.0);
      parameters.setDefaultAngularWeight(0.1);

      // Pre-processor: state estimator of all the inputs
      parameters.setInputPoseLPFBreakFrequency(15.0);
      parameters.setInputPoseCorrectionDuration(0.05); // Need to send inputs at 30Hz.
      parameters.setInputStateEstimatorType(InputStateEstimatorType.FBC_STYLE);
      // Bounding box around the robot, if the input gets out, the IK stops
      parameters.setUseBBXInputFilter(true);
      parameters.setInputBBXFilterSize(2.0, 2.8, 2.5);
      parameters.setInputBBXFilterCenter(0.4, 0.0, 1.25);

      // Post-processor on the IK solution
      parameters.setOutputLPFBreakFrequency(10.0);
      parameters.setOutputJointVelocityScale(0.65);

      parameters.setMinimizeAngularMomentum(true);
      parameters.setMinimizeLinearMomentum(false);
      // Minimize angular momentum: prioritize joints that are closer to the end-effectors.
      parameters.setAngularMomentumWeight(0.125);
      //      parameters.setLinearMomentumWeight(0.25);

      parameters.getDefaultConfiguration().setEnableLeftHandTaskspace(false);
      parameters.getDefaultConfiguration().setEnableRightHandTaskspace(false);
      parameters.getDefaultConfiguration().setEnableNeckJointspace(false);
      parameters.getDefaultSolverConfiguration().setJointVelocityWeight(0.05);
      parameters.getDefaultSolverConfiguration().setEnableJointVelocityLimits(false);
      parameters.setUseStreamingPublisher(true);

      if (createToolbox)
      {
         boolean startYoVariableServer = true;
         toolbox = new KinematicsStreamingToolboxModule(robotModel,
                                                        parameters,
                                                        startYoVariableServer,
                                                        DomainFactory.PubSubImplementation.FAST_RTPS);
         ((KinematicsStreamingToolboxController) toolbox.getToolboxController()).setInitialRobotConfigurationNamedMap(
               createInitialConfiguration(robotModel));
      }

      RDXBaseUI.getInstance().getKeyBindings().register("Streaming - Enable IK (toggle)", "Right A button");
      RDXBaseUI.getInstance().getKeyBindings().register("Streaming - Control robot (toggle)", "Left A button");
   }

   private Map<String, Double> createInitialConfiguration(DRCRobotModel robotModel)
   {
      Map<String, Double> initialConfigurationMap = new HashMap<>();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      RobotInitialSetup<HumanoidFloatingRootJointRobot> defaultRobotInitialSetup = robotModel.getDefaultRobotInitialSetup(
            0.0,
            0.0);
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
         initialConfigurationMap.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL),
                                     robotSide.negateIfRightSide(0.13));
         initialConfigurationMap.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), 0.13);
         initialConfigurationMap.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), -1.0);

         initialConfigurationMap.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), -0.58);
         initialConfigurationMap.put(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), 0.55 + 0.672);
         initialConfigurationMap.put(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH), -0.64);
      }

      return initialConfigurationMap;
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
                  if (latestStatus.getCurrentToolboxState()
                      == KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_INITIALIZE_FAILURE_MISSING_RCD
                      && messageThrottler.run(1.0))
                     LogTools.warn("Status update: Toolbox failed initialization, missing RobotConfigurationData.");
                  else if (latestStatus.getCurrentToolboxState()
                           == KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_INITIALIZE_SUCCESSFUL)
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
            ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputCommandTopic(syncedRobot.getRobotModel()
                                                                                                          .getSimpleRobotName()),
                                         inputToSend);
      }
   }

   public void addIKRigidBodyInputs(List<KinematicsToolboxRigidBodyMessage> rigidBodyInputs)
   {
      if ((enabled.get() || kinematicsRecorder.isReplaying()) && toolboxInputStreamRateLimiter.run(streamPeriod))
      {
         KinematicsStreamingToolboxInputMessage inputToSend = toolboxInputMessagePending.getAndSet(null);
         if (inputToSend == null)
         {
            KinematicsStreamingToolboxInputMessage toolboxInputMessage = new KinematicsStreamingToolboxInputMessage();

            if (enabled.get())
               toolboxInputMessage.setStreamToController(streamToController.get());
            else
               toolboxInputMessage.setStreamToController(kinematicsRecorder.isReplaying());
            //         toolboxInputMessage.setTimestamp();

            toolboxInputMessagePending.set(toolboxInputMessage);
            inputToSend = toolboxInputMessagePending.getAndSet(null);
         }

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
         if (enabled.get())
         {
            inputToSend.setStreamToController(streamToController.get());
         }
         else
            inputToSend.setStreamToController(kinematicsRecorder.isReplaying());
         toolboxInputMessagePending.set(inputToSend);

         outputFrequencyPlot.recordEvent();
      }
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
      ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStateTopic(syncedRobot.getRobotModel()
                                                                                                  .getSimpleRobotName()),
                                   toolboxStateMessage);
   }

   private void wakeUpToolbox()
   {
      ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
      toolboxStateMessage.setRequestedToolboxState(ToolboxState.WAKE_UP.toByte());
      ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStateTopic(syncedRobot.getRobotModel()
                                                                                                  .getSimpleRobotName()),
                                   toolboxStateMessage);
   }

   private void sleepToolbox()
   {
      ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
      toolboxStateMessage.setRequestedToolboxState(ToolboxState.SLEEP.toByte());
      ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStateTopic(syncedRobot.getRobotModel()
                                                                                                  .getSimpleRobotName()),
                                   toolboxStateMessage);
   }

   public void getVirtualRenderables(Array<Renderable> renderables,
                                     Pool<Renderable> pool,
                                     Set<RDXSceneLevel> sceneLevels)
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
         ikUpperArmControlFrameGraphics.get(side).getRenderables(renderables, pool);
         ikForearmControlFrameGraphics.get(side).getRenderables(renderables, pool);
         ikChestControlFrameGraphics.getRenderables(renderables, pool);
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
         ikUpperArmControlFrameGraphics.get(side).dispose();
         ikForearmControlFrameGraphics.get(side).dispose();
         ikChestControlFrameGraphics.dispose();
      }
   }

   public void sendHandCommand(RobotSide robotSide, HandConfiguration desiredHandConfiguration)
   {
      ros2ControllerHelper.publish(DeprecatedAPIs::getHandConfigurationTopic,
                                   HumanoidMessageTools.createHandDesiredConfigurationMessage(robotSide,
                                                                                              desiredHandConfiguration));
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

   public enum CapturyMarker
   {
      SHOULDER, ELBOW, HAND, CHEST
   }

   public KinematicsToolboxRigidBodyMessage remoteCapturyStreaming(ReferenceFrame referenceFrame,
                                                                   int handSideInteger,
                                                                   CapturyMarker marker)
   {
      RobotSide side = handSideInteger == 0 ? RobotSide.LEFT : RobotSide.RIGHT;

      KinematicsToolboxRigidBodyMessage output = new KinematicsToolboxRigidBodyMessage();
      trackerFrameGraphics.put(CapturyTrackedSegmentType.LEFT_FOREARM.getSegmentName(),
                               new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
      trackerFrameGraphics.put(CapturyTrackedSegmentType.RIGHT_FOREARM.getSegmentName(),
                               new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));

      switch (marker)
      {
         case HAND:
         {
            CapturyTrackedSegmentType hand = CapturyTrackedSegmentType.toHand(side);
            handDesiredControlFrames.put(side, new MutableReferenceFrame(referenceFrame));
            MovingReferenceFrame endEffectorFrame = ghostFullRobotModel.getEndEffectorFrame(side, LimbName.ARM);
            controllerFrameGraphics.get(side).setToReferenceFrame(referenceFrame);
            handFrameGraphics.get(side).setToReferenceFrame(endEffectorFrame);

            KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(ghostFullRobotModel.getHand(side),
                                                                               handDesiredControlFrames.get(side)
                                                                                                       .getReferenceFrame(),
                                                                               hand.getSegmentName(),
                                                                               0.0,
                                                                               1.0);
            MessageTools.packSelectionMatrix3DMessage(false, message.getLinearSelectionMatrix());
            MessageTools.packSelectionMatrix3DMessage(true, message.getAngularSelectionMatrix());
            message.getControlFramePositionInEndEffector().set(ikHandControlFramePoses.get(side).getPosition());
            message.getControlFrameOrientationInEndEffector().set(ikHandControlFramePoses.get(side).getOrientation());
            message.setHasAngularVelocity(true);
            message.setHasLinearVelocity(true);
            output.set(message);
            break;
         }
         case SHOULDER:
         {
            CapturyTrackedSegmentType upperArm = CapturyTrackedSegmentType.toUpperArm(side);

            MutableReferenceFrame trackerDesiredControlFrame = new MutableReferenceFrame(referenceFrame);
            trackerDesiredControlFrame.getReferenceFrame().update();
            trackedSegmentDesiredFrame.put(upperArm.getSegmentName(), trackerDesiredControlFrame);

            if (!trackerFrameGraphics.containsKey(upperArm.getSegmentName()))
            {
               trackerFrameGraphics.put(upperArm.getSegmentName(),
                                        new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
            }

            trackerFrameGraphics.get(upperArm.getSegmentName())
                                .setToReferenceFrame(trackerDesiredControlFrame.getReferenceFrame());
            RigidBodyBasics controlledSegment = ghostUpperArms.get(side);

            if (controlledSegment != null)
            {
               KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(controlledSegment,
                                                                                  trackerDesiredControlFrame.getReferenceFrame(),
                                                                                  upperArm.getSegmentName(),
                                                                                  0.0,
                                                                                  0.5);

               MessageTools.packSelectionMatrix3DMessage(false, message.getLinearSelectionMatrix());
               MessageTools.packSelectionMatrix3DMessage(true, false, true, null, message.getAngularSelectionMatrix());
               Pose3D controlFramePose = ikUpperArmControlFramePoses.get(side);
               message.getControlFramePositionInEndEffector().set(controlFramePose.getPosition());
               message.getControlFrameOrientationInEndEffector().set(controlFramePose.getOrientation());
               message.setHasAngularVelocity(true);
               message.setHasLinearVelocity(true);
               output.set(message);
            }
            break;
         }
         case ELBOW:
         {
            CapturyTrackedSegmentType forearm = CapturyTrackedSegmentType.toForearm(side);

            MutableReferenceFrame trackerDesiredControlFrame = new MutableReferenceFrame(referenceFrame);
            trackerDesiredControlFrame.getReferenceFrame().update();
            trackedSegmentDesiredFrame.put(forearm.getSegmentName(), trackerDesiredControlFrame);
            trackerFrameGraphics.get(forearm.getSegmentName())
                                .setToReferenceFrame(trackedSegmentDesiredFrame.get(forearm.getSegmentName())
                                                                               .getReferenceFrame());
            RigidBodyBasics controlledSegment = ghostFullRobotModel.getForearm(side);

            if (controlledSegment != null)
            {
               KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(controlledSegment,
                                                                                  trackedSegmentDesiredFrame.get(forearm.getSegmentName())
                                                                                                            .getReferenceFrame(),
                                                                                  forearm.getSegmentName(),
                                                                                  0.0,
                                                                                  1.0);
               MessageTools.packSelectionMatrix3DMessage(false, message.getLinearSelectionMatrix());
               MessageTools.packSelectionMatrix3DMessage(true, false, true, null, message.getAngularSelectionMatrix());
               message.getControlFramePositionInEndEffector().set(ikForearmControlFramePoses.get(side).getPosition());
               message.getControlFrameOrientationInEndEffector()
                      .set(ikForearmControlFramePoses.get(side).getOrientation());
               message.setHasAngularVelocity(true);
               message.setHasLinearVelocity(true);
               output.set(message);
            }
            break;
         }
         case CHEST:
         {
            CapturyTrackedSegmentType chest = CapturyTrackedSegmentType.toChest();

            MutableReferenceFrame trackerDesiredControlFrame = new MutableReferenceFrame(referenceFrame);
            trackerDesiredControlFrame.getReferenceFrame().update();

            trackedSegmentDesiredFrame.put(chest.getSegmentName(), trackerDesiredControlFrame);

            if (!trackerFrameGraphics.containsKey(chest.getSegmentName()))
            {
               trackerFrameGraphics.put(chest.getSegmentName(),
                                        new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
            }
            trackedSegmentDesiredFrame.put(chest.getSegmentName(), trackerDesiredControlFrame);
            trackerFrameGraphics.get(chest.getSegmentName())
                                .setToReferenceFrame(trackedSegmentDesiredFrame.get(chest.getSegmentName())
                                                                               .getReferenceFrame());
            RigidBodyBasics controlledSegment = ghostFullRobotModel.getChest();

            if (controlledSegment != null)
            {
               KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(controlledSegment,
                                                                                  trackedSegmentDesiredFrame.get(chest.getSegmentName())
                                                                                                            .getReferenceFrame(),
                                                                                  chest.getSegmentName(),
                                                                                  0.0,
                                                                                  10);
               MessageTools.packSelectionMatrix3DMessage(false, message.getLinearSelectionMatrix());
               MessageTools.packSelectionMatrix3DMessage(true, true, true, null, message.getAngularSelectionMatrix());
               message.getControlFramePositionInEndEffector().set(ikChestControlFramePoses.getPosition());
               message.getControlFrameOrientationInEndEffector().set(ikChestControlFramePoses.getOrientation());
               message.setHasAngularVelocity(true);
               message.setHasLinearVelocity(true);
               output.set(message);
            }
            break;
         }
      }
      return output;
   }
}