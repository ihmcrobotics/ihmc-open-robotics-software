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
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
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

import java.util.HashMap;
import java.util.Set;

public class RDXVRKinematicsStreamingMode
{
   private final DRCRobotModel robotModel;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final RestartableJavaProcess kinematicsStreamingToolboxProcess;
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
   private final SideDependentList<OneDoFJointBasics> wristJoints = new SideDependentList<>();
   private final SideDependentList<ImGuiPlot> wristJointAnglePlots = new SideDependentList<>();
   private PausablePeriodicThread wakeUpThread;
   private final ImBoolean wakeUpThreadRunning = new ImBoolean(false);
   private RDXReferenceFrameGraphic headsetFrameGraphic;
   private final SideDependentList<ModifiableReferenceFrame> handDesiredControlFrames = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> controllerFrameGraphics = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> handControlFrameGraphics = new SideDependentList<>();
   private final HashMap<String, ModifiableReferenceFrame> trackedSegmentDesiredFrame = new HashMap<>();
   private final HashMap<String, RDXReferenceFrameGraphic> trackedSegmentGraphics = new HashMap<>();
   private final ImBoolean showReferenceFrameGraphics = new ImBoolean(true);
   private final ImBoolean streamToController = new ImBoolean(false);
   private final Throttler messageThrottler = new Throttler();
   private KinematicsRecordReplay kinematicsRecorder;
   private RDXVRSharedControl sharedControlAssistant;

   private final HandConfiguration[] handConfigurations = {HandConfiguration.OPEN, HandConfiguration.HALF_CLOSE, HandConfiguration.CRUSH};
   private int leftIndex = -1;
   private int rightIndex = -1;
   private RDXVRControllerModel controllerModel = RDXVRControllerModel.UNKNOWN;

   public RDXVRKinematicsStreamingMode(DRCRobotModel robotModel,
                                       ROS2ControllerHelper ros2ControllerHelper,
                                       RestartableJavaProcess kinematicsStreamingToolboxProcess)
   {
      this.robotModel = robotModel;
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
         handControlFrameGraphics.put(side, new RDXReferenceFrameGraphic(length));
         RigidBodyTransform wristToHandControlTransform = robotModel.getUIParameters().getTransformWristToHand(side);
         ModifiableReferenceFrame handDesiredControlFrame = new ModifiableReferenceFrame(vrContext.getController(side).getXForwardZUpControllerFrame());
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

      kinematicsRecorder = new KinematicsRecordReplay(ros2ControllerHelper, enabled, 2);
      sharedControlAssistant = new RDXVRSharedControl(robotModel, ros2ControllerHelper, streamToController, kinematicsRecorder.isReplayingEnabled());
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

         // Check if left B button is pressed in order to trigger shared control assistance
         InputDigitalActionData bButton = controller.getBButtonActionData();
         sharedControlAssistant.processInput(bButton);
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
         for (RobotSide side : RobotSide.values)
         {
            vrContext.getController(side).runIfConnected(controller ->
            {  //TODO edit this part to include other robot parts (e.g., feet, elbows, chest?)
               KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
               message.setEndEffectorHashCode(ghostFullRobotModel.getHand(side).hashCode());
               tempFramePose.setToZero(handDesiredControlFrames.get(side).getReferenceFrame());
               tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
               controllerFrameGraphics.get(side).setToReferenceFrame(controller.getXForwardZUpControllerFrame());
               handControlFrameGraphics.get(side).setToReferenceFrame(handDesiredControlFrames.get(side).getReferenceFrame());
               kinematicsRecorder.framePoseToRecord(tempFramePose);
               if (kinematicsRecorder.isReplaying())
                  kinematicsRecorder.framePoseToPack(tempFramePose); //get values of tempFramePose from replay
               else if (sharedControlAssistant.isActive())
               {
                  if(sharedControlAssistant.readyToPack())
                     sharedControlAssistant.framePoseToPack(tempFramePose, side.getCamelCaseName() + "Hand");
                  else
                     sharedControlAssistant.processFrameInformation(tempFramePose, side.getCamelCaseName() + "Hand");
               }
               message.getDesiredPositionInWorld().set(tempFramePose.getPosition());
               message.getDesiredOrientationInWorld().set(tempFramePose.getOrientation());
               message.getControlFrameOrientationInEndEffector().setYawPitchRoll(0.0,
                                                                                 side.negateIfLeftSide(Math.PI / 2.0),
                                                                                 side.negateIfLeftSide(Math.PI / 2.0));
               toolboxInputMessage.getInputs().add().set(message);
            });
         }

         Set<String> trackedSegments = vrContext.getBodySegmentsWithTrackers();
         for (TrackedSegmentType segmentType : TrackedSegmentType.values()) {
            if (trackedSegments.contains(segmentType.segmentName)) {
               handleTrackedSegment(vrContext, segmentType);
            }
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

         if (enabled.get())
            toolboxInputMessage.setStreamToController(streamToController.get());
         else
            toolboxInputMessage.setStreamToController(kinematicsRecorder.isReplaying());
         toolboxInputMessage.setTimestamp(System.nanoTime());
         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputCommandTopic(robotModel.getSimpleRobotName()), toolboxInputMessage);
         outputFrequencyPlot.recordEvent();
      }
   }

   private void handleTrackedSegment(RDXVRContext vrContext, TrackedSegmentType segmentType) {
      vrContext.getTracker(segmentType.segmentName).runIfConnected(tracker ->
      {
         if (!trackedSegmentDesiredFrame.containsKey(segmentType.segmentName))
         {
            trackedSegmentDesiredFrame.put(segmentType.segmentName, new ModifiableReferenceFrame(segmentType.segmentName, vrContext.getTracker(segmentType.segmentName).getXForwardZUpTrackerFrame()));
            trackedSegmentDesiredFrame.get(segmentType.segmentName).getTransformToParent().getRotation().append(segmentType.trackerToRigidBodyRotation);
         }
         trackedSegmentDesiredFrame.get(segmentType.segmentName).changeParentFrame(vrContext.getTracker(segmentType.segmentName).getXForwardZUpTrackerFrame());
         trackedSegmentDesiredFrame.get(segmentType.segmentName).getReferenceFrame().update();
         trackedSegmentGraphics.get(segmentType.segmentName).setToReferenceFrame(trackedSegmentDesiredFrame.get(segmentType.segmentName).getReferenceFrame());
         switch (segmentType) {
            case LEFT_FOREARM:
               leftForeArmControlFrameGraphic.setToReferenceFrame(controlFrame.getReferenceFrame());
               break;
            case RIGHT_FOREARM:
               rightForeArmControlFrameGraphic.setToReferenceFrame(controlFrame.getReferenceFrame());
               break;
            case CHEST:
               chestControlFrameGraphic.setToReferenceFrame(controlFrame.getReferenceFrame());
               break;
         }

         KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(ghostFullRobotModel.getForearm(RobotSide.LEFT),
                                                                            trackedSegmentDesiredFrame.get(segmentType.segmentName),
                                                                            segmentType.segmentName,
                                                                            segmentType.positionWeight,
                                                                            segmentType.orientationWeight);
         toolboxInputMessage.getInputs().add().set(message);
      });
   }

   private KinematicsToolboxRigidBodyMessage createRigidBodyMessage(RigidBodyBasics endEffector, ModifiableReferenceFrame desiredControlFrame, String frameName, double positionWeight, double orientationWeight) {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(endEffector.hashCode());
      tempFramePose.setToZero(desiredControlFrame.getReferenceFrame());
      tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      // record motion if in recording mode
      kinematicsRecorder.framePoseToRecord(tempFramePose, frameName);
         if (kinematicsRecorder.isReplaying())
            kinematicsRecorder.framePoseToPack(tempFramePose); //get values of tempFramePose from replay

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
            if (sharedControlAssistant.isActive() && sharedControlAssistant.isPreviewActive()) // if preview is enabled
            {
               if (sharedControlAssistant.isFirstPreview()) // first preview
               {
                  sharedControlAssistant.saveStatusForPreview(latestStatus); // store the status
                  sharedControlAssistant.updatePreviewModel(latestStatus); // update shared control ghost robot
               }
               else if (sharedControlAssistant.isPreviewGraphicActive()) // replay preview
               {
                  // update IK ghost
                  KinematicsToolboxOutputStatus statusPreview = sharedControlAssistant.getPreviewStatus();
                  ghostFullRobotModel.getRootJoint().setJointPosition(statusPreview.getDesiredRootPosition());
                  ghostFullRobotModel.getRootJoint().setJointOrientation(statusPreview.getDesiredRootOrientation());
                  for (int i = 0; i < ghostOneDoFJointsExcludingHands.length; i++)
                     ghostOneDoFJointsExcludingHands[i].setQ(statusPreview.getDesiredJointAngles().get(i));
                  ghostFullRobotModel.getElevator().updateFramesRecursively();
                  // update shared control ghost
                  sharedControlAssistant.replayPreviewModel();
               }
            }
         }
      }
      ghostRobotGraphic.update();
      if(sharedControlAssistant.isActive() && sharedControlAssistant.isPreviewActive()) // if graphic active update also graphic
         sharedControlAssistant.getGhostPreviewGraphic().update();
   }

   public void renderImGuiWidgets()
   {
      if (controllerModel == RDXVRControllerModel.FOCUS3)
      {
         ImGui.text("Toggle IK tracking enabled: A button");
         ImGui.text("Toggle stream to controller: X button");
      }
      else
      {
         ImGui.text("Toggle IK tracking enabled: Right A button");
         ImGui.text("Toggle stream to controller: Left A button");
      }

      kinematicsStreamingToolboxProcess.renderImGuiWidgets();
      ghostRobotGraphic.renderImGuiWidgets();
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
      // add widgets for recording/replaying motion in VR
      ImGui.text("Start/Stop recording: Press Left Joystick");
      kinematicsRecorder.renderRecordWidgets(labels);
      ImGui.text("Start/Stop replay: Press Left Joystick (cannot stream/record if replay)");
      kinematicsRecorder.renderReplayWidgets(labels);
      // add widget for using shared control assistance in VR
      if (controllerModel == RDXVRControllerModel.FOCUS3)
         ImGui.text("Toggle shared control assistance: Y button");
      else
         ImGui.text("Toggle shared control assistance: Left B button");
      sharedControlAssistant.renderWidgets(labels);
      if (ImGui.checkbox(labels.get("Wake up thread"), wakeUpThreadRunning))
      {
         wakeUpThread.setRunning(wakeUpThreadRunning.get());
      }
      ImGui.checkbox(labels.get("Streaming to controller"), streamToController);
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
         if (sharedControlAssistant.isActive() && sharedControlAssistant.isPreviewActive())
         {
            sharedControlAssistant.getGhostPreviewGraphic().getRenderables(renderables, pool, sceneLevels);
            for (var spline : sharedControlAssistant.getSplinePreviewGraphic().entrySet())
               spline.getValue().getRenderables(renderables, pool);
            for (var stdDeviationRegion : sharedControlAssistant.getStdDeviationGraphic().entrySet())
               stdDeviationRegion.getValue().getRenderables(renderables, pool);
         }
      }
      if (showReferenceFrameGraphics.get())
      {
//         headsetFrameGraphic.getRenderables(renderables, pool);
         for (RobotSide side : RobotSide.values)
         {
            controllerFrameGraphics.get(side).getRenderables(renderables, pool);
            handControlFrameGraphics.get(side).getRenderables(renderables, pool);
         }
      }
   }

   public void destroy()
   {
      ghostRobotGraphic.destroy();
      sharedControlAssistant.destroy();
      headsetFrameGraphic.dispose();
      for (RobotSide side : RobotSide.values)
      {
         controllerFrameGraphics.get(side).dispose();
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

   public RestartableProcess getKinematicsStreamingToolboxProcess()
   {
      return kinematicsStreamingToolboxProcess;
   }
}
