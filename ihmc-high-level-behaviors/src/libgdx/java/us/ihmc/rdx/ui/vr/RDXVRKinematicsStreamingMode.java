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
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sharedControl.HandConfigurationListener;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.referenceFrame.FixedReferenceFrame;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.processes.RestartableProcess;
import us.ihmc.rdx.ui.processes.RestartableJavaProcess;
import us.ihmc.rdx.ui.tools.KinematicsRecordReplay;
import us.ihmc.rdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rdx.vr.RDXVRControllerModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.tools.thread.Throttler;

import java.util.Set;

public class RDXVRKinematicsStreamingMode implements HandConfigurationListener
{
   private static final int NUMBER_OF_PARTS_TO_RECORD = 5;
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final RestartableJavaProcess kinematicsStreamingToolboxProcess;
   private RDXMultiBodyGraphic ghostRobotGraphic;
   private FullHumanoidRobotModel ghostFullRobotModel;
   private OneDoFJointBasics[] ghostOneDoFJointsExcludingHands;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean enabled = new ImBoolean(true);
   private final ImBoolean advanced = new ImBoolean(false);
   private IHMCROS2Input<KinematicsToolboxOutputStatus> status;
   private final double streamPeriod = UnitConversions.hertzToSeconds(10.0);
   private final Throttler toolboxInputStreamRateLimiter = new Throttler();
   private final FramePose3D tempFramePose = new FramePose3D();
   private final ImGuiFrequencyPlot statusFrequencyPlot = new ImGuiFrequencyPlot();
   private final ImGuiFrequencyPlot outputFrequencyPlot = new ImGuiFrequencyPlot();
   private final SideDependentList<OneDoFJointBasics> wristJoints = new SideDependentList<>();
   private final SideDependentList<ImGuiPlot> wristJointAnglePlots = new SideDependentList<>();
   private PausablePeriodicThread wakeUpThread;
   private final ImBoolean wakeUpThreadRunning = new ImBoolean(false);
   private RDXReferenceFrameGraphic headsetFrameGraphic;
   private final SideDependentList<ModifiableReferenceFrame> handDesiredControlFrames = new SideDependentList<>();
   private ModifiableReferenceFrame leftForeArmDesiredControlFrame;
   private ModifiableReferenceFrame rightForeArmDesiredControlFrame;
   private ModifiableReferenceFrame chestDesiredControlFrame;
   private ModifiableReferenceFrame chestTrackerControlFrame;
   private final SideDependentList<RDXReferenceFrameGraphic> controllerFrameGraphics = new SideDependentList<>();
   private RDXReferenceFrameGraphic leftForeArmControlFrameGraphic;
   private RDXReferenceFrameGraphic rightForeArmControlFrameGraphic;
   private RDXReferenceFrameGraphic chestControlFrameGraphic;
   private final ImBoolean showReferenceFrameGraphics = new ImBoolean(true);
   private final ImBoolean streamToController = new ImBoolean(false);
   private final Throttler messageThrottler = new Throttler();
   private KinematicsRecordReplay kinematicsRecorder;
   private RDXVRAssistance vrAssistant;

   private final HandConfiguration[] handConfigurations = {HandConfiguration.HALF_CLOSE, HandConfiguration.CRUSH, HandConfiguration.CLOSE};
   private int leftIndex = -1;
   private int rightIndex = -1;
   private RDXVRControllerModel controllerModel = RDXVRControllerModel.UNKNOWN;

   public RDXVRKinematicsStreamingMode(ROS2SyncedRobotModel syncedRobot,
                                       ROS2ControllerHelper ros2ControllerHelper,
                                       RestartableJavaProcess kinematicsStreamingToolboxProcess)
   {
      this.syncedRobot = syncedRobot;
      this.robotModel = syncedRobot.getRobotModel();
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.kinematicsStreamingToolboxProcess = kinematicsStreamingToolboxProcess;
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

      double length = 0.2;
      headsetFrameGraphic = new RDXReferenceFrameGraphic(length);
      for (RobotSide side : RobotSide.values)
      {
         controllerFrameGraphics.put(side, new RDXReferenceFrameGraphic(length));
         leftForeArmControlFrameGraphic = new RDXReferenceFrameGraphic(length);
         rightForeArmControlFrameGraphic = new RDXReferenceFrameGraphic(length);
         chestControlFrameGraphic = new RDXReferenceFrameGraphic(length);
         RigidBodyTransform wristToHandControlTransform = robotModel.getUIParameters().getTransformWristToHand(side);
         ModifiableReferenceFrame handDesiredControlFrame = new ModifiableReferenceFrame( side.getCamelCaseName() + "Hand", vrContext.getController(side).getXForwardZUpControllerFrame());
         //            handDesiredControlFrame.getTransformToParent().set(robotModel.getJointMap().getHandControlFrameToWristTransform(side));
         // Atlas
         //         {
         //            handDesiredControlFrame.getTransformToParent().getTranslation().setX(-0.1);
         //            handDesiredControlFrame.getTransformToParent()
         //                                   .getRotation()
         //                                   .setYawPitchRoll(side == RobotSide.RIGHT ? 0.0 : Math.toRadians(180.0), side.negateIfLeftSide(Math.toRadians(90.0)), 0.0);
         //         }
         // Nadia
         {
            if (side == RobotSide.LEFT)
            {
               handDesiredControlFrame.getTransformToParent().getRotation().setToYawOrientation(Math.PI);
               handDesiredControlFrame.getTransformToParent().getRotation().appendRollRotation(Math.PI / 2.0);
            }
            else
            {
               handDesiredControlFrame.getTransformToParent().getRotation().setToRollOrientation(Math.PI / 2.0);
            }
         }
         handDesiredControlFrame.getReferenceFrame().update();
         handDesiredControlFrames.put(side, handDesiredControlFrame);
         ArmJointName lastWristJoint = robotModel.getJointMap().getArmJointNames()[robotModel.getJointMap().getArmJointNames().length - 1];
         wristJoints.put(side, ghostFullRobotModel.getArmJoint(side, lastWristJoint));
         wristJointAnglePlots.put(side, new ImGuiPlot(labels.get(side + " Hand Joint Angle")));
      }

      status = ros2ControllerHelper.subscribe(KinematicsStreamingToolboxModule.getOutputStatusTopic(robotModel.getSimpleRobotName()));

      wakeUpThread = new PausablePeriodicThread(getClass().getSimpleName() + "WakeUpThread", 1.0, true, this::wakeUpToolbox);

      kinematicsRecorder = new KinematicsRecordReplay(ros2ControllerHelper, enabled, NUMBER_OF_PARTS_TO_RECORD);
      vrAssistant = new RDXVRAssistance(robotModel, ros2ControllerHelper, streamToController, kinematicsRecorder.isReplayingEnabled());

      if (!kinematicsStreamingToolboxProcess.isStarted())
         kinematicsStreamingToolboxProcess.start();
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
         if(kinematicsRecorder.isReplayingEnabled().get())
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
         if (clickTriggerButton.bChanged() && !clickTriggerButton.bState() && !vrAssistant.isActive())
         {
            HandConfiguration handConfiguration = nextHandConfiguration(RobotSide.RIGHT);
            sendHandCommand(RobotSide.RIGHT, handConfiguration);
         }
      });

      vrAssistant.processInput(vrContext);

      for (RobotSide side : RobotSide.values)
      {
         vrContext.getController(side).runIfConnected(controller ->
         {
            float currentGripX = controller.getGripActionData().x();
//            if (currentGripX)
            {

            }
//            lastGripX
         });
      }

      if ((enabled.get() || kinematicsRecorder.isReplaying()) && toolboxInputStreamRateLimiter.run(streamPeriod))
      {
         KinematicsStreamingToolboxInputMessage toolboxInputMessage = new KinematicsStreamingToolboxInputMessage();
         for (RobotSide side : RobotSide.values)
         {
            vrContext.getController(side).runIfConnected(controller ->
            {
               KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
               message.setEndEffectorHashCode(ghostFullRobotModel.getHand(side).hashCode());
               String frameName = handDesiredControlFrames.get(side).getFrameName();
               tempFramePose.setToZero(handDesiredControlFrames.get(side).getReferenceFrame());
               tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
               controllerFrameGraphics.get(side).setToReferenceFrame(controller.getXForwardZUpControllerFrame());
               kinematicsRecorder.framePoseToRecord(tempFramePose, frameName);
               if (kinematicsRecorder.isReplaying())
                  kinematicsRecorder.framePoseToPack(tempFramePose); //get values of tempFramePose from replay
               else if (vrAssistant.isActive())
               {
                  if (vrAssistant.readyToPack())
                  {
                     vrAssistant.framePoseToPack(tempFramePose, frameName);
                     vrAssistant.checkForHandConfigurationUpdates(this);
                  }
                  else
                     vrAssistant.processFrameInformation(tempFramePose, frameName);
               }
               if (!vrAssistant.isActive())
               {
                  message.getDesiredPositionInWorld().set(tempFramePose.getPosition());
                  message.getDesiredOrientationInWorld().set(tempFramePose.getOrientation());
                  message.getControlFrameOrientationInEndEffector().setYawPitchRoll(0.0, side.negateIfLeftSide(Math.PI / 2.0), side.negateIfLeftSide(Math.PI / 2.0));
                  toolboxInputMessage.getInputs().add().set(message);
               }
               else if (vrAssistant.isPlaying() && vrAssistant.containsBodyPart(frameName))
               {
                  message.getDesiredPositionInWorld().set(tempFramePose.getPosition());
                  message.getDesiredOrientationInWorld().set(tempFramePose.getOrientation());
                  if(!vrAssistant.isAffordancePhase())
                     message.getControlFrameOrientationInEndEffector().setYawPitchRoll(0.0, side.negateIfLeftSide(Math.PI / 2.0), side.negateIfLeftSide(Math.PI / 2.0));
                  toolboxInputMessage.getInputs().add().set(message);
               }
            });
         }

         Set<String> trackedSegments = vrContext.getBodySegmentsWithTrackers();
         if (trackedSegments.contains("leftForeArm"))
         {
            String segment = "leftForeArm";
            vrContext.getTracker(segment).runIfConnected(tracker ->
            {
               if (leftForeArmDesiredControlFrame == null)
               {
                  leftForeArmDesiredControlFrame = new ModifiableReferenceFrame(segment,
                                                                                vrContext.getTracker(
                                                                                      segment).getXForwardZUpTrackerFrame());
                  leftForeArmDesiredControlFrame.getTransformToParent().getRotation().appendYawRotation(-Math.PI / 2.0);
               }
               leftForeArmDesiredControlFrame.changeParentFrame(vrContext.getTracker(segment).getXForwardZUpTrackerFrame());
               leftForeArmDesiredControlFrame.getReferenceFrame().update();
               leftForeArmControlFrameGraphic.setToReferenceFrame(leftForeArmDesiredControlFrame.getReferenceFrame());
               KinematicsToolboxRigidBodyMessage message = createOrientationRigidBodyMessage(
                     ghostFullRobotModel.getForearm(RobotSide.LEFT),
                     leftForeArmDesiredControlFrame,
                     0.1);
               toolboxInputMessage.getInputs().add().set(message);
            });
         }

         if (trackedSegments.contains("rightForeArm"))
         {
            String segment = "rightForeArm";
            vrContext.getTracker(segment).runIfConnected(tracker ->
            {
               if (rightForeArmDesiredControlFrame == null)
               {
                  rightForeArmDesiredControlFrame = new ModifiableReferenceFrame(segment, vrContext.getTracker(segment).getXForwardZUpTrackerFrame());
                  rightForeArmDesiredControlFrame.getTransformToParent().getRotation().appendYawRotation(Math.PI / 2.0);
               }
               rightForeArmDesiredControlFrame.changeParentFrame(vrContext.getTracker(segment).getXForwardZUpTrackerFrame());
               rightForeArmDesiredControlFrame.getReferenceFrame().update();
               rightForeArmControlFrameGraphic.setToReferenceFrame(rightForeArmDesiredControlFrame.getReferenceFrame());
               KinematicsToolboxRigidBodyMessage message = createOrientationRigidBodyMessage(ghostFullRobotModel.getForearm(RobotSide.RIGHT),
                                                                                             rightForeArmDesiredControlFrame,
                                                                                             0.1);
               toolboxInputMessage.getInputs().add().set(message);
            });
         }

         if (trackedSegments.contains("chest"))
         {
            String segment = "chest";
            vrContext.getTracker(segment).runIfConnected(tracker ->
            {
                  // start with current pose of the chest
                  if (chestTrackerControlFrame == null)
                  {
                     chestTrackerControlFrame = new ModifiableReferenceFrame(segment, vrContext.getTracker(segment).getXForwardZUpTrackerFrame());
                  }
                  // update tracker frame
                  chestTrackerControlFrame.changeParentFrame(vrContext.getTracker(segment).getXForwardZUpTrackerFrame());
                  chestTrackerControlFrame.getReferenceFrame().update();
                  chestControlFrameGraphic.setToReferenceFrame(chestTrackerControlFrame.getReferenceFrame());
                  KinematicsToolboxRigidBodyMessage message = createOrientationRigidBodyMessage(ghostFullRobotModel.getChest(), chestTrackerControlFrame,10);
                  toolboxInputMessage.getInputs().add().set(message);
            });
         }

         if (enabled.get())
         {
            if (chestDesiredControlFrame == null)
            {
               FixedReferenceFrame initialChestFrame = new FixedReferenceFrame("fixedChestFrame", ReferenceFrame.getWorldFrame(), new RigidBodyTransform(syncedRobot.getFullRobotModel().getChest().getBodyFixedFrame().getTransformToWorldFrame()));
               chestDesiredControlFrame = new ModifiableReferenceFrame("chestInitial", initialChestFrame);
            }
//            KinematicsToolboxRigidBodyMessage message = createPoseRigidBodyMessage(ghostFullRobotModel.getChest(), chestDesiredControlFrame, 10, 1);
            KinematicsToolboxRigidBodyMessage message = createPositionRigidBodyMessage(ghostFullRobotModel.getChest(), chestDesiredControlFrame, 10);
            toolboxInputMessage.getInputs().add().set(message);
         }

         if (enabled.get())
            toolboxInputMessage.setStreamToController(streamToController.get());
         else
            toolboxInputMessage.setStreamToController(kinematicsRecorder.isReplaying());
         toolboxInputMessage.setTimestamp(System.nanoTime());
         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputCommandTopic(robotModel.getSimpleRobotName()), toolboxInputMessage);
         outputFrequencyPlot.recordEvent();
      }
   }

   private KinematicsToolboxRigidBodyMessage createOrientationRigidBodyMessage(RigidBodyBasics endEffector, ModifiableReferenceFrame desiredControlFrame, double weight) {
      return createPoseRigidBodyMessage(endEffector, desiredControlFrame, 0, weight);
   }

   private KinematicsToolboxRigidBodyMessage createPositionRigidBodyMessage(RigidBodyBasics endEffector, ModifiableReferenceFrame desiredControlFrame, double weight) {
      return createPoseRigidBodyMessage(endEffector, desiredControlFrame, weight, 0);
   }

   private KinematicsToolboxRigidBodyMessage createPoseRigidBodyMessage(RigidBodyBasics endEffector, ModifiableReferenceFrame desiredControlFrame, double positionWeight, double orientationWeight) {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(endEffector.hashCode());
      String frameName = desiredControlFrame.getFrameName();
      tempFramePose.setToZero(desiredControlFrame.getReferenceFrame());
      tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      if (!frameName.equals("chestInitial"))
      {
         kinematicsRecorder.framePoseToRecord(tempFramePose, frameName);
         if (kinematicsRecorder.isReplaying())
            kinematicsRecorder.framePoseToPack(tempFramePose); //get values of tempFramePose from replay
         else if (vrAssistant.isActive())
         {
            if (vrAssistant.readyToPack())
               vrAssistant.framePoseToPack(tempFramePose, frameName);
            else
               vrAssistant.processFrameInformation(tempFramePose, frameName);
         }
         if (!vrAssistant.isActive() || vrAssistant.containsBodyPart(frameName))
         {
            message.getDesiredOrientationInWorld().set(tempFramePose.getOrientation());
            message.getDesiredPositionInWorld().set(tempFramePose.getPosition());
         }
      }
      else
      {
         message.getDesiredOrientationInWorld().set(tempFramePose.getOrientation());
         message.getDesiredPositionInWorld().set(tempFramePose.getPosition());
      }
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
      return message;
   }

   public void update(boolean ikStreamingModeEnabled)
   {
      // Safety features!
      if (!ikStreamingModeEnabled)
         streamToController.set(false);
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

            // update preview assistance
            if (vrAssistant.isActive() && vrAssistant.isPreviewActive()) // if preview is enabled
            {
               if (vrAssistant.isFirstPreview()) // first preview
               {
                  vrAssistant.saveStatusForPreview(latestStatus); // store the status
                  vrAssistant.updatePreviewModel(latestStatus); // update shared control ghost robot
               }
               else if (vrAssistant.isPreviewGraphicActive()) // replay preview
               {
                  // update IK ghost
                  KinematicsToolboxOutputStatus statusPreview = vrAssistant.getPreviewStatus();
                  ghostFullRobotModel.getRootJoint().setJointPosition(statusPreview.getDesiredRootPosition());
                  ghostFullRobotModel.getRootJoint().setJointOrientation(statusPreview.getDesiredRootOrientation());
                  for (int i = 0; i < ghostOneDoFJointsExcludingHands.length; i++)
                     ghostOneDoFJointsExcludingHands[i].setQ(statusPreview.getDesiredJointAngles().get(i));
                  ghostFullRobotModel.getElevator().updateFramesRecursively();
                  // update shared control ghost
                  vrAssistant.replayPreviewModel();
               }
            }
         }
      }
      if (ghostRobotGraphic.isActive())
         ghostRobotGraphic.update();
      if(vrAssistant.isActive() && vrAssistant.isPreviewActive()) // if graphic active update also graphic
         vrAssistant.getGhostPreviewGraphic().update();
      vrAssistant.update();
   }

   public void renderImGuiWidgets()
   {
      ImGui.separator();
      if (controllerModel == RDXVRControllerModel.FOCUS3)
         ImGui.text("X BUTTON");
      else
         ImGui.text("LEFT A BUTTON");
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Control Robot"), streamToController);
      // add widget for using shared control assistance in VR
      if (controllerModel == RDXVRControllerModel.FOCUS3)
         ImGui.text("Y BUTTON");
      else
         ImGui.text("LEFT B BUTTON");
      ImGui.sameLine();
      vrAssistant.renderWidgets(labels);

      ImGui.text("Advanced options");
      ImGui.sameLine();
      ImGui.checkbox(labels.get(""), advanced);
      if (advanced.get())
      {
         ImGui.separator();
         ghostRobotGraphic.renderImGuiWidgets();
         // add widgets for recording/replaying motion in VR
         ImGui.text("Left Joystick: Start/Stop recording");
         kinematicsRecorder.renderRecordWidgets(labels);
         ImGui.text("Left Joystick: Start/Stop replay");
         kinematicsRecorder.renderReplayWidgets(labels);
         ImGui.checkbox(labels.get("Show reference frames"), showReferenceFrameGraphics);
         // add widgets for kinematics streaming management
         kinematicsStreamingToolboxProcess.renderImGuiWidgets();
         if (ImGui.checkbox(labels.get("Kinematics streaming"), enabled))
         {
            setEnabled(enabled.get());
         }
         ImGui.sameLine();
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
         if (ImGui.checkbox(labels.get("Wake up thread"), wakeUpThreadRunning))
         {
            wakeUpThread.setRunning(wakeUpThreadRunning.get());
         }
         ImGui.text("Output:");
         ImGui.sameLine();
         outputFrequencyPlot.renderImGuiWidgets();
         ImGui.text("Status:");
         ImGui.sameLine();
         statusFrequencyPlot.renderImGuiWidgets();
         for (RobotSide side : RobotSide.values)
         {
            wristJointAnglePlots.get(side).render(wristJoints.get(side).getQ());
         }
      }
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
      else
      {
         vrAssistant.setEnabled(false);
         chestDesiredControlFrame = null;
      }
   }

   public boolean isEnabled()
   {
      return enabled.get();
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
      if (status.hasReceivedFirstMessage())
      {
         ghostRobotGraphic.getRenderables(renderables, pool,sceneLevels);
         if (vrAssistant.isActive() && vrAssistant.isPreviewActive())
         {
            vrAssistant.getGhostPreviewGraphic().getRenderables(renderables, pool, sceneLevels);
            for (var spline : vrAssistant.getSplinePreviewGraphic().entrySet())
               spline.getValue().getRenderables(renderables, pool);
            for (var stdDeviationRegion : vrAssistant.getStdDeviationGraphic().entrySet())
               stdDeviationRegion.getValue().getRenderables(renderables, pool);
         }
         vrAssistant.getMenuPanel().getRenderables(renderables, pool);
      }
      if (showReferenceFrameGraphics.get())
      {
//         headsetFrameGraphic.getRenderables(renderables, pool);
         for (RobotSide side : RobotSide.values)
         {
            controllerFrameGraphics.get(side).getRenderables(renderables, pool);
         }
         leftForeArmControlFrameGraphic.getRenderables(renderables, pool);
         rightForeArmControlFrameGraphic.getRenderables(renderables, pool);
         chestControlFrameGraphic.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      ghostRobotGraphic.destroy();
      vrAssistant.destroy();
      headsetFrameGraphic.dispose();
      for (RobotSide side : RobotSide.values)
      {
         controllerFrameGraphics.get(side).dispose();
      }
   }

   @Override
   public void onNotification() {
      var configuration = vrAssistant.getHandConfiguration();
      sendHandCommand(configuration.getLeft(), configuration.getRight());
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

   public RestartableProcess getKinematicsStreamingToolboxProcess()
   {
      return kinematicsStreamingToolboxProcess;
   }

   public RDXVRAssistance getVRAssistant()
   {
      return vrAssistant;
   }
}