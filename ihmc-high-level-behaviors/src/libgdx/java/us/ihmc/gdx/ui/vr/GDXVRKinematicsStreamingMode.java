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
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.graphics.GDXRobotModelGraphic;
import us.ihmc.gdx.ui.missionControl.processes.KinematicsStreamingToolboxProcess;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

public class GDXVRKinematicsStreamingMode
{
   private final DRCRobotModel robotModel;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final KinematicsStreamingToolboxProcess kinematicsStreamingToolboxProcess;
   private GDXRobotModelGraphic ghostRobotGraphic;
   private FullHumanoidRobotModel ghostFullRobotModel;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean enabled = new ImBoolean(false);
   private ROS2Input<KinematicsToolboxOutputStatus> status;
   private final double streamPeriod = UnitConversions.hertzToSeconds(10.0);
   private final Throttler toolboxInputStreamRateLimiter = new Throttler();
   private final FramePose3D tempFramePose = new FramePose3D();

   public GDXVRKinematicsStreamingMode(DRCRobotModel robotModel, ROS2ControllerHelper ros2ControllerHelper)
   {
      this.robotModel = robotModel;
      this.ros2ControllerHelper = ros2ControllerHelper;
      kinematicsStreamingToolboxProcess = new KinematicsStreamingToolboxProcess(robotModel);
   }

   public void create()
   {
      RobotDefinition ghostRobotDefinition = new RobotDefinition(robotModel.getRobotDefinition());
      MaterialDefinition material = new MaterialDefinition(ColorDefinitions.parse("0x4B61D1").derive(0.0, 1.0, 1.0, 0.5));
      RobotDefinition.forEachRigidBodyDefinition(ghostRobotDefinition.getRootBodyDefinition(),
                                                 body -> body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(material)));

      ghostFullRobotModel = robotModel.createFullRobotModel();
      ghostRobotGraphic = new GDXRobotModelGraphic(robotModel.getSimpleRobotName());
      ghostRobotGraphic.loadRobotModelAndGraphics(ghostRobotDefinition, ghostFullRobotModel.getElevator(), robotModel);
      ghostRobotGraphic.create();

      status = ros2ControllerHelper.subscribe(KinematicsStreamingToolboxModule.getOutputStatusTopic(robotModel.getSimpleRobotName()));
   }

   public void processVRInput(GDXVRContext vrContext)
   {
      for (RobotSide side : RobotSide.values)
      {
         vrContext.getController(side).runIfConnected(controller ->
         {

         });
      }

      if (toolboxInputStreamRateLimiter.run(streamPeriod))
      {
         KinematicsStreamingToolboxInputMessage toolboxInputMessage = new KinematicsStreamingToolboxInputMessage();
         for (RobotSide side : RobotSide.values)
         {
            vrContext.getController(side).runIfConnected(controller ->
            {
               KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
               message.setEndEffectorHashCode(ghostFullRobotModel.getHand(side).hashCode());
               tempFramePose.setToZero(controller.getXForwardZUpControllerFrame());
               tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
               message.getDesiredPositionInWorld().set(tempFramePose.getPosition());
               message.getDesiredOrientationInWorld().set(tempFramePose.getOrientation());
               message.getControlFrameOrientationInEndEffector().setYawPitchRoll(0.0,
                                                                                 side.negateIfLeftSide(Math.PI / 2.0),
                                                                                 side.negateIfLeftSide(Math.PI / 2.0));
               toolboxInputMessage.getInputs().add().set(message);
            });
         }
         vrContext.getHeadset().runIfConnected(headset ->
         {
            KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
            message.setEndEffectorHashCode(ghostFullRobotModel.getHead().hashCode());
            tempFramePose.setToZero(headset.getXForwardZUpHeadsetFrame());
            tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
            message.getDesiredPositionInWorld().set(tempFramePose.getPosition());
            message.getDesiredOrientationInWorld().set(tempFramePose.getOrientation());
            message.getControlFramePositionInEndEffector().set(0.1, 0.0, 0.0);
            message.getControlFrameOrientationInEndEffector().setYawPitchRoll(Math.PI / 2.0, 0.0, Math.PI / 2.0);
            boolean xSelected = false;
            boolean ySelected = false;
            boolean zSelected = true;
            message.getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(xSelected,
                                                                                               ySelected,
                                                                                               zSelected,
                                                                                               ReferenceFrame.getWorldFrame()));
            toolboxInputMessage.getInputs().add().set(message);
         });
         toolboxInputMessage.setStreamToController(false);
         toolboxInputMessage.setTimestamp(System.nanoTime());
         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputCommandTopic(robotModel.getSimpleRobotName()), toolboxInputMessage);
      }
   }

   public void update()
   {
      if (status.getMessageNotification().poll())
      {
         KinematicsToolboxOutputStatus latestStatus = status.getMessageNotification().read();
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
            OneDoFJointBasics[] oneDoFJoints = ghostFullRobotModel.getOneDoFJoints();
            for (int i = 0; i < oneDoFJoints.length; i++)
            {
               oneDoFJoints[i].setQ(latestStatus.getDesiredJointAngles().get(i));
            }
         }
      }
      ghostRobotGraphic.update();
   }

   public void renderImGuiWidgets()
   {
      kinematicsStreamingToolboxProcess.renderImGuiWidgets();
      if (ImGui.checkbox(labels.get("Kinematics streaming"), enabled))
      {
         ToolboxState requestedState = enabled.get() ? ToolboxState.WAKE_UP : ToolboxState.SLEEP;
         ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
         toolboxStateMessage.setRequestedToolboxState(requestedState.toByte());
         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStateTopic(robotModel.getSimpleRobotName()), toolboxStateMessage);
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      ghostRobotGraphic.getRenderables(renderables, pool);
   }

   public void destroy()
   {
      ghostRobotGraphic.destroy();
   }

   public KinematicsStreamingToolboxProcess getKinematicsStreamingToolboxProcess()
   {
      return kinematicsStreamingToolboxProcess;
   }
}
