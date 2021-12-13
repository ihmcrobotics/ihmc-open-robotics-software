package us.ihmc.gdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import controller_msgs.msg.dds.ToolboxStateMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.graphics.GDXReferenceFrameGraphic;
import us.ihmc.gdx.ui.graphics.GDXRobotModelGraphic;
import us.ihmc.gdx.ui.missionControl.processes.KinematicsStreamingToolboxProcess;
import us.ihmc.gdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.tools.thread.Throttler;

import java.util.Map;

public class GDXVRKinematicsStreamingMode
{
   private final DRCRobotModel robotModel;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final KinematicsStreamingToolboxProcess kinematicsStreamingToolboxProcess;
   private GDXRobotModelGraphic ghostRobotGraphic;
   private FullHumanoidRobotModel ghostFullRobotModel;
   private OneDoFJointBasics[] ghostOneDoFJointsExcludingHands;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean enabled = new ImBoolean(false);
   private ROS2Input<KinematicsToolboxOutputStatus> status;
   private final double streamPeriod = UnitConversions.hertzToSeconds(10.0);
   private final Throttler toolboxInputStreamRateLimiter = new Throttler();
   private final FramePose3D tempFramePose = new FramePose3D();
   private final ImGuiFrequencyPlot statusFrequencyPlot = new ImGuiFrequencyPlot();
   private final ImGuiFrequencyPlot outputFrequencyPlot = new ImGuiFrequencyPlot();
   private final SideDependentList<OneDoFJointBasics> wristJoints = new SideDependentList<>();
   private final SideDependentList<ImGuiPlot> wristJointAnglePlots = new SideDependentList<>();
   private PausablePeriodicThread wakeUpThread;
   private ImBoolean wakeUpThreadRunning = new ImBoolean(false);
   private GDXReferenceFrameGraphic headsetFrameGraphic;
   private SideDependentList<ReferenceFrame> handDesiredControlFrames = new SideDependentList<>();
   private SideDependentList<GDXReferenceFrameGraphic> controllerFrameGraphics = new SideDependentList<>();
   private SideDependentList<GDXReferenceFrameGraphic> handControlFrameGraphics = new SideDependentList<>();
   private ImBoolean showReferenceFrameGraphics = new ImBoolean(true);
   private boolean streamToController;

   public GDXVRKinematicsStreamingMode(DRCRobotModel robotModel, Map<String, Double> initialConfiguration, ROS2ControllerHelper ros2ControllerHelper)
   {
      this.robotModel = robotModel;
      this.ros2ControllerHelper = ros2ControllerHelper;
      kinematicsStreamingToolboxProcess = new KinematicsStreamingToolboxProcess(robotModel, initialConfiguration);
   }

   public void create(GDXVRContext vrContext)
   {
      RobotDefinition ghostRobotDefinition = new RobotDefinition(robotModel.getRobotDefinition());
      MaterialDefinition material = new MaterialDefinition(ColorDefinitions.parse("0x4B61D1").derive(0.0, 1.0, 1.0, 0.5));
      RobotDefinition.forEachRigidBodyDefinition(ghostRobotDefinition.getRootBodyDefinition(),
                                                 body -> body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(material)));

      ghostFullRobotModel = robotModel.createFullRobotModel();
      ghostOneDoFJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(ghostFullRobotModel);
      ghostRobotGraphic = new GDXRobotModelGraphic(robotModel.getSimpleRobotName() + " (IK Preview Ghost)");
      ghostRobotGraphic.loadRobotModelAndGraphics(ghostRobotDefinition, ghostFullRobotModel.getElevator(), robotModel);
      ghostRobotGraphic.setActive(true);
      ghostRobotGraphic.create();

      double length = 0.2;
      headsetFrameGraphic = new GDXReferenceFrameGraphic(length);
      for (RobotSide side : RobotSide.values)
      {
         controllerFrameGraphics.put(side, new GDXReferenceFrameGraphic(length));
         handControlFrameGraphics.put(side, new GDXReferenceFrameGraphic(length));
         RigidBodyTransform wristToHandControlTransform = robotModel.getUIParameters().getTransformWristToHand(side);
         RigidBodyTransform handControlToControllerTransform = new RigidBodyTransform();
         handControlToControllerTransform.getTranslation().setX(-0.1);
         handControlToControllerTransform.getRotation().setYawPitchRoll(side == RobotSide.RIGHT ? 0.0 : Math.toRadians(180.0),
                                                                        side.negateIfLeftSide(Math.toRadians(90.0)),
                                                                        0.0);
         handDesiredControlFrames.put(side,
                                      ReferenceFrameMissingTools
                                            .constructFrameWithUnchangingTransformToParent(vrContext.getController(side).getXForwardZUpControllerFrame(),
                                                                                           handControlToControllerTransform));
         wristJoints.put(side, ghostFullRobotModel.getArmJoint(side, ArmJointName.SECOND_WRIST_PITCH));
         wristJointAnglePlots.put(side, new ImGuiPlot(labels.get(side + " Hand Joint Angle")));
      }

      status = ros2ControllerHelper.subscribe(KinematicsStreamingToolboxModule.getOutputStatusTopic(robotModel.getSimpleRobotName()));

      wakeUpThread = new PausablePeriodicThread(getClass().getSimpleName() + "WakeUpThread", 1.0, true, this::wakeUpToolbox);
   }

   public void processVRInput(GDXVRContext vrContext)
   {
      for (RobotSide side : RobotSide.values)
      {
         vrContext.getController(side).runIfConnected(controller ->
         {
            if (side == RobotSide.LEFT)
            {
               InputDigitalActionData aButton = controller.getAButtonActionData();
               if (aButton.bChanged() && !aButton.bState())
               {
                  streamToController = !streamToController;
               }
            }

            float currentGripX = controller.getGripActionData().x();
//            if (currentGripX)
            {

            }
//            lastGripX
         });
      }

      if (enabled.get() && toolboxInputStreamRateLimiter.run(streamPeriod))
      {
         KinematicsStreamingToolboxInputMessage toolboxInputMessage = new KinematicsStreamingToolboxInputMessage();
         for (RobotSide side : RobotSide.values)
         {
            vrContext.getController(side).runIfConnected(controller ->
            {
               KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
               message.setEndEffectorHashCode(ghostFullRobotModel.getHand(side).hashCode());
               tempFramePose.setToZero(handDesiredControlFrames.get(side));
               tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
               controllerFrameGraphics.get(side).setToReferenceFrame(controller.getXForwardZUpControllerFrame());
               handControlFrameGraphics.get(side).setToReferenceFrame(handDesiredControlFrames.get(side));
               message.getDesiredPositionInWorld().set(tempFramePose.getPosition());
               message.getDesiredOrientationInWorld().set(tempFramePose.getOrientation());
               message.getControlFrameOrientationInEndEffector().setYawPitchRoll(0.0,
                                                                                 side.negateIfLeftSide(Math.PI / 2.0),
                                                                                 side.negateIfLeftSide(Math.PI / 2.0));
               toolboxInputMessage.getInputs().add().set(message);
            });
         }
//         vrContext.getHeadset().runIfConnected(headset ->
//         {
//            KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
//            message.setEndEffectorHashCode(ghostFullRobotModel.getHead().hashCode());
//            tempFramePose.setToZero(headset.getXForwardZUpHeadsetFrame());
//            tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
//            headsetFrameGraphic.setToReferenceFrame(headset.getXForwardZUpHeadsetFrame());
//            message.getDesiredPositionInWorld().set(tempFramePose.getPosition());
//            message.getDesiredOrientationInWorld().set(tempFramePose.getOrientation());
//            message.getControlFramePositionInEndEffector().set(0.1, 0.0, 0.0);
//            message.getControlFrameOrientationInEndEffector().setYawPitchRoll(Math.PI / 2.0, 0.0, Math.PI / 2.0);
//            boolean xSelected = false;
//            boolean ySelected = false;
//            boolean zSelected = true;
//            message.getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(xSelected,
//                                                                                               ySelected,
//                                                                                               zSelected,
//                                                                                               ReferenceFrame.getWorldFrame()));
//            toolboxInputMessage.getInputs().add().set(message);
//         });
         toolboxInputMessage.setStreamToController(streamToController);
         toolboxInputMessage.setTimestamp(System.nanoTime());
         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputCommandTopic(robotModel.getSimpleRobotName()), toolboxInputMessage);
         outputFrequencyPlot.recordEvent();
      }
   }

   public void update()
   {
      if (status.getMessageNotification().poll())
      {
         KinematicsToolboxOutputStatus latestStatus = status.getMessageNotification().read();
         statusFrequencyPlot.recordEvent();
         if (latestStatus.getJointNameHash() == -1)
         {
            if (latestStatus.getCurrentToolboxState() == KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_INITIALIZE_FAILURE_MISSING_RCD)
               LogTools.warn("Status update: Toolbox failed initialization, missing RobotConfigurationData.");
            else if (latestStatus.getCurrentToolboxState() == KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_INITIALIZE_SUCCESSFUL)
               LogTools.info("Status update: Toolbox initialized successfully.");
         }
         else
         {
            ghostFullRobotModel.getRootJoint().setJointPosition(latestStatus.getDesiredRootTranslation());
            ghostFullRobotModel.getRootJoint().setJointOrientation(latestStatus.getDesiredRootOrientation());
            for (int i = 0; i < ghostOneDoFJointsExcludingHands.length; i++)
            {
               ghostOneDoFJointsExcludingHands[i].setQ(latestStatus.getDesiredJointAngles().get(i));
            }
            ghostFullRobotModel.getElevator().updateFramesRecursively();
         }
      }
      ghostRobotGraphic.update();
   }

   public void renderImGuiWidgets()
   {
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
      if (ImGui.checkbox(labels.get("Wake up thread"), wakeUpThreadRunning))
      {
         wakeUpThread.setRunning(wakeUpThreadRunning.get());
      }
      ImGui.text("Streaming to controller: " + streamToController);
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
         wakeUpToolbox();
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

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      ghostRobotGraphic.getRenderables(renderables, pool);
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
      headsetFrameGraphic.dispose();
      for (RobotSide side : RobotSide.values)
      {
         controllerFrameGraphics.get(side).dispose();
      }
   }

   public KinematicsStreamingToolboxProcess getKinematicsStreamingToolboxProcess()
   {
      return kinematicsStreamingToolboxProcess;
   }
}
