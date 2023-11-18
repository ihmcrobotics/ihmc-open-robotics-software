package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.lwjgl.openvr.InputDigitalActionData;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.communication.ros2.ROS2ControllerPublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.rdx.imgui.ImGuiFrequencyPlot;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.vr.TrackedSegmentType;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRControllerModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public class RDXIKStreamer
{
   private static final double FRAME_AXIS_GRAPHICS_LENGTH = 0.2;
   private final DRCRobotModel robotModel;
   private final ROS2ControllerPublishSubscribeAPI ros2ControllerHelper;
   private final SideDependentList<RDXInteractableHand> interactableHands;
   private RDXMultiBodyGraphic ghostRobotGraphic;
   private FullHumanoidRobotModel ghostFullRobotModel;
   private OneDoFJointBasics[] ghostOneDoFJointsExcludingHands;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean enabled = new ImBoolean(false);
   private IHMCROS2Input<KinematicsToolboxOutputStatus> status;
   private final double streamPeriod = UnitConversions.hertzToSeconds(30.0);
   private final Throttler toolboxInputStreamRateLimiter = new Throttler();
   private final FramePose3D tempFramePose = new FramePose3D();
   private final ImGuiFrequencyPlot statusFrequencyPlot = new ImGuiFrequencyPlot();
   private final ImGuiFrequencyPlot outputFrequencyPlot = new ImGuiFrequencyPlot();
   private final ImBoolean wakeUpThreadRunning = new ImBoolean(false);
   private final SideDependentList<ReferenceFrame> vrControllerControlFrames = new SideDependentList<>();
   private final SideDependentList<MutableReferenceFrame> handDesiredControlFrames = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> controllerFrameGraphics = new SideDependentList<>();
   private final Map<String, MutableReferenceFrame> trackedSegmentDesiredFrame = new HashMap<>();
   private final Map<String, RDXReferenceFrameGraphic> trackerFrameGraphics = new HashMap<>();
   private double userHeightChangeRate = 0.0;
   private final ImBoolean showReferenceFrameGraphics = new ImBoolean(true);
   private final ImBoolean streamToController = new ImBoolean(false);
   private final Throttler messageThrottler = new Throttler();

   private final HandConfiguration[] handConfigurations = {HandConfiguration.HALF_CLOSE, HandConfiguration.CRUSH, HandConfiguration.CLOSE};
   private int leftIndex = -1;
   private int rightIndex = -1;
   private RDXVRControllerModel controllerModel = RDXVRControllerModel.UNKNOWN;
   private Set<String> additionalTrackedSegments;

   public RDXIKStreamer(DRCRobotModel robotModel,
                        ROS2ControllerPublishSubscribeAPI ros2ControllerHelper,
                        SideDependentList<RDXInteractableHand> interactableHands)
   {
      this.robotModel = robotModel;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.interactableHands = interactableHands;
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

      for (RobotSide side : RobotSide.values)
      {
         controllerFrameGraphics.put(side, new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
         RigidBodyTransform handControlFrameToVRControllerFrame = new RigidBodyTransform();
         if (side == RobotSide.LEFT)
         {
            handControlFrameToVRControllerFrame.getRotation().setToYawOrientation(Math.PI);
            handControlFrameToVRControllerFrame.getRotation().appendRollRotation(Math.PI / 2.0);
         }
         else
         {
            handControlFrameToVRControllerFrame.getRotation().setToRollOrientation(Math.PI / 2.0);
         }
         ReferenceFrame vrControllerControlFrame
               = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(vrContext.getController(side).getXForwardZUpControllerFrame(),
                                                                                          handControlFrameToVRControllerFrame);
         vrControllerControlFrames.put(side, vrControllerControlFrame);
      }

      status = ros2ControllerHelper.subscribe(KinematicsStreamingToolboxModule.getOutputStatusTopic(robotModel.getSimpleRobotName()));
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      if (controllerModel == RDXVRControllerModel.UNKNOWN)
      {
         controllerModel = vrContext.getControllerModel();
      }

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

         // use right joystick values to control pelvis height of robot indirectly by teleporting operator up/down
         // NOTE. intentionally not controlling pelvis height directly but teleporting the whole user to make them more comfortable
         userHeightChangeRate = -controller.getJoystickActionData().y();
         vrContext.teleport(teleportIHMCZUpToIHMCZUpWorld -> teleportIHMCZUpToIHMCZUpWorld.getTranslation().addZ(userHeightChangeRate * 0.01));
      });

      additionalTrackedSegments = vrContext.getBodySegmentsWithTrackers();
   }

   private void handleTrackedSegment(RDXVRContext vrContext,
                                     KinematicsStreamingToolboxInputMessage toolboxInputMessage,
                                     TrackedSegmentType segmentType,
                                     Set<String> additionalTrackedSegments)
   {
      if (additionalTrackedSegments.contains(segmentType.getSegmentName()))
      {
         vrContext.getTracker(segmentType.getSegmentName()).runIfConnected(tracker ->
         {
            if (!trackerFrameGraphics.containsKey(segmentType.getSegmentName()))
            {
               trackerFrameGraphics.put(segmentType.getSegmentName(), new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
            }
            if (!trackedSegmentDesiredFrame.containsKey(segmentType.getSegmentName()))
            {
               trackedSegmentDesiredFrame.put(segmentType.getSegmentName(),
                                              new MutableReferenceFrame(segmentType.getSegmentName(),
                                                                        vrContext.getTracker(segmentType.getSegmentName()).getXForwardZUpTrackerFrame()));
               trackedSegmentDesiredFrame.get(segmentType.getSegmentName())
                                         .getTransformToParent()
                                         .getRotation()
                                         .append(segmentType.getTrackerToRigidBodyRotation());
            }
            trackedSegmentDesiredFrame.get(segmentType.getSegmentName())
                                      .setParentFrame(vrContext.getTracker(segmentType.getSegmentName()).getXForwardZUpTrackerFrame());
            trackedSegmentDesiredFrame.get(segmentType.getSegmentName()).getReferenceFrame().update();
            trackerFrameGraphics.get(segmentType.getSegmentName())
                                .setToReferenceFrame(trackedSegmentDesiredFrame.get(segmentType.getSegmentName()).getReferenceFrame());
            RigidBodyBasics controlledSegment = switch (segmentType)
            {
               case LEFT_FOREARM -> ghostFullRobotModel.getForearm(RobotSide.LEFT);
               case RIGHT_FOREARM -> ghostFullRobotModel.getForearm(RobotSide.RIGHT);
               case CHEST -> ghostFullRobotModel.getChest();
               default -> throw new IllegalStateException("Unexpected VR-tracked segment: " + segmentType);
            };
            KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(controlledSegment,
                                                                               trackedSegmentDesiredFrame.get(segmentType.getSegmentName()).getReferenceFrame(),
                                                                               segmentType.getSegmentName(),
                                                                               segmentType.getPositionWeight(),
                                                                               segmentType.getOrientationWeight());
            toolboxInputMessage.getInputs().add().set(message);
         });
      }
      else if (segmentType.getSegmentName().contains("Hand"))
      {
         vrContext.getController(segmentType.getSegmentSide()).runIfConnected(controller ->
         {
           controllerFrameGraphics.get(segmentType.getSegmentSide())
                                  .setToReferenceFrame(controller.getXForwardZUpControllerFrame());
           ReferenceFrame controlFrame;
           if (!interactableHands.get(segmentType.getSegmentSide()).isDeleted())
              controlFrame = interactableHands.get(segmentType.getSegmentSide()).getControlReferenceFrame();
           else
              controlFrame = vrControllerControlFrames.get(segmentType.getSegmentSide());
           KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(ghostFullRobotModel.getHand(segmentType.getSegmentSide()),
                                                                              controlFrame,
                                                                              segmentType.getSegmentName(),
                                                                              segmentType.getPositionWeight(),
                                                                              segmentType.getOrientationWeight());
           // TODO. this transform should go in TrackedSegmentType. and also the one in the create() method. Actually check if they cancel out
           message.getControlFrameOrientationInEndEffector()
                  .setYawPitchRoll(0.0,
                                   segmentType.getSegmentSide().negateIfLeftSide(Math.PI / 2.0),
                                   segmentType.getSegmentSide().negateIfLeftSide(Math.PI / 2.0));
           toolboxInputMessage.getInputs().add().set(message);
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

      message.getDesiredOrientationInWorld().set(tempFramePose.getOrientation());
      message.getDesiredPositionInWorld().set(tempFramePose.getPosition());
      if (!Double.isNaN(positionWeight))
      {
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
      }
      if (!Double.isNaN(orientationWeight))
      {
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
      }
      return message;
   }

   public void update(boolean ikStreamingModeEnabled)
   {
      // Safety features!
      if (!ikStreamingModeEnabled)
         streamToController.set(false);
      setEnabled(ikStreamingModeEnabled);
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
         }
      }
      ghostRobotGraphic.update();

      if (enabled.get() && toolboxInputStreamRateLimiter.run(streamPeriod))
      {
         KinematicsStreamingToolboxInputMessage toolboxInputMessage = new KinematicsStreamingToolboxInputMessage();
//         for (TrackedSegmentType segmentType : TrackedSegmentType.values())
//            handleTrackedSegment(vrContext, toolboxInputMessage, segmentType, additionalTrackedSegments);

         for (RobotSide side : RobotSide.values)
         {
            if (!interactableHands.get(side).isDeleted())
            {
               ReferenceFrame controlFrame = interactableHands.get(side).getControlReferenceFrame();
               controllerFrameGraphics.get(side).setToReferenceFrame(controlFrame);

               TrackedSegmentType trackedSegmentHand = side == RobotSide.LEFT ? TrackedSegmentType.LEFT_HAND : TrackedSegmentType.RIGHT_HAND;

               KinematicsToolboxRigidBodyMessage message = createRigidBodyMessage(ghostFullRobotModel.getHand(side),
                                                                                  controlFrame,
                                                                                  "Hand",
                                                                                  trackedSegmentHand.getPositionWeight(),
                                                                                  trackedSegmentHand.getOrientationWeight());
               // TODO. this transform should go in TrackedSegmentType. and also the one in the create() method. Actually check if they cancel out
//               message.getControlFrameOrientationInEndEffector()
//                      .setYawPitchRoll(0.0, side.negateIfLeftSide(Math.PI / 2.0), side.negateIfLeftSide(Math.PI / 2.0));
               toolboxInputMessage.getInputs().add().set(message);
            }
         }

         toolboxInputMessage.setStreamToController(streamToController.get());
         toolboxInputMessage.setTimestamp(System.nanoTime());
         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputCommandTopic(robotModel.getSimpleRobotName()), toolboxInputMessage);
         outputFrequencyPlot.recordEvent();
      }
   }

   public void renderImGuiWidgets()
   {
      if (controllerModel == RDXVRControllerModel.FOCUS3)
         ImGui.text("X Button");
      else
         ImGui.text("Left A Button");
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Control/Stop Robot"), streamToController);

      // add widget for using shared control assistance in VR
      if (controllerModel == RDXVRControllerModel.FOCUS3)
         ImGui.text("Y Button");
      else
         ImGui.text("Left B Button");
      ImGui.sameLine();

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
      if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
         if (status.hasReceivedFirstMessage())
         {
            ghostRobotGraphic.getRenderables(renderables, pool, sceneLevels);
         }
         if (showReferenceFrameGraphics.get())
         {
            for (RobotSide side : RobotSide.values)
               controllerFrameGraphics.get(side).getRenderables(renderables, pool);
            for (RDXReferenceFrameGraphic trackerGraphic : trackerFrameGraphics.values())
               trackerGraphic.getRenderables(renderables, pool);
         }
      }
   }

   public void destroy()
   {
      ghostRobotGraphic.destroy();
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
}
