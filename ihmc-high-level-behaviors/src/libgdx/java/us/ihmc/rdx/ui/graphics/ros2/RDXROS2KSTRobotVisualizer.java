package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import toolbox_msgs.KinematicsToolboxOutputStatus;
import toolbox_msgs.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.ToolboxAPIs;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;

import java.util.Set;

public class RDXROS2KSTRobotVisualizer extends RDXROS2SingleTopicVisualizer<KinematicsToolboxOutputStatus>
{
   private final ROS2Node ros2Node;
   private final DRCRobotModel robotModel;
   private final FullHumanoidRobotModel ghostFullRobotModel;
   private final OneDoFJointBasics[] ghostOneDoFJointsExcludingHands;
   private final ROS2Topic<KinematicsToolboxOutputStatus> topic;
   private final TypedNotification<KinematicsToolboxOutputStatus> statusSubscription = new TypedNotification<>();
   private ROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;
   private RDXMultiBodyGraphic multiBodyGraphic;
   private String text;

   public RDXROS2KSTRobotVisualizer(ROS2Node ros2Node, DRCRobotModel robotModel)
   {
      super("Kinematics Streaming Solution");

      this.ros2Node = ros2Node;
      this.robotModel = robotModel;

      ghostFullRobotModel = robotModel.createFullRobotModel();
      ghostOneDoFJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(ghostFullRobotModel);
      topic = KinematicsStreamingToolboxModule.getOutputStatusTopic(robotModel.getSimpleRobotName());
   }

   @Override
   public void create()
   {
      super.create();

      multiBodyGraphic = new RDXMultiBodyGraphic(robotModel.getSimpleRobotName() + " (IK Preview Ghost)");
      RobotDefinition ghostRobotDefinition = new RobotDefinition(robotModel.getRobotDefinition());
      MaterialDefinition material = new MaterialDefinition(ColorDefinitions.parse("0xDEE934").derive(0.0, 1.0, 1.0, 0.5));
      RobotDefinition.forEachRigidBodyDefinition(ghostRobotDefinition.getRootBodyDefinition(),
                                                 body -> body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(material)));
      multiBodyGraphic.loadRobotModelAndGraphics(ghostRobotDefinition, ghostFullRobotModel.getElevator());
      multiBodyGraphic.setActive(true);
      multiBodyGraphic.create();

      ros2Node.createSubscription(topic, reader ->
      {
         getFrequency().ping();
         KinematicsToolboxOutputStatus status = reader.read();
         if (status != null)
            statusSubscription.set(status);
      });

      toolboxStatePublisher = ros2Node.createPublisher(ToolboxAPIs.KINEMATICS_STREAMING_TOOLBOX.withRobot(robotModel.getSimpleRobotName())
                                                                                               .withInput()
                                                                                               .withType(ToolboxStateMessage.class));
   }

   @Override
   public void update()
   {
      if (multiBodyGraphic.isRobotLoaded())
      {
         super.update();

         if (statusSubscription.poll())
         {
            KinematicsToolboxOutputStatus status = statusSubscription.read();
            text = null;
            if (status.getJointNameHash() == -1)
            {
               if (status.getCurrentToolboxState() == KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_INITIALIZE_FAILURE_MISSING_RCD)
                  text = "Status update: Toolbox failed initialization, missing RobotConfigurationData.";
               else if (status.getCurrentToolboxState() == KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_INITIALIZE_SUCCESSFUL)
                  text = "Status update: Toolbox initialized successfully.";
            }
            else
            {
               ghostFullRobotModel.getRootJoint().setJointPosition(status.getDesiredRootPosition().getPoint());
               ghostFullRobotModel.getRootJoint().setJointOrientation(status.getDesiredRootOrientation().getQuaternion());
               for (int i = 0; i < ghostOneDoFJointsExcludingHands.length; i++)
                  ghostOneDoFJointsExcludingHands[i].setQ(status.getDesiredJointAngles().get(i));
               ghostFullRobotModel.getElevator().updateFramesRecursively();
            }
         }

         multiBodyGraphic.update();
      }
   }

   @Override
   public ROS2Topic<KinematicsToolboxOutputStatus> getTopic()
   {
      return topic;
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (text != null)
         ImGui.textColored(ImGuiTools.DARK_RED, text);

      renderButton(ToolboxState.SLEEP);
      ImGui.sameLine();
      ImGui.text("->");
      ImGui.sameLine();
      renderButton(ToolboxState.REINITIALIZE);
      ImGui.sameLine();
      ImGui.text("->");
      ImGui.sameLine();
      renderButton(ToolboxState.WAKE_UP);

      multiBodyGraphic.renderImGuiWidgets();
   }

   private void renderButton(ToolboxState state)
   {
      if (ImGui.button(state.name()))
      {
         ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
         toolboxStateMessage.setRequestedToolboxState(state.toByte());
         toolboxStatePublisher.publish(toolboxStateMessage);
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      multiBodyGraphic.setActive(isActive());
      multiBodyGraphic.getRenderables(renderables, pool, sceneLevels);
   }

   public void destroy()
   {
      super.destroy();
      multiBodyGraphic.destroy();
   }
}
