package us.ihmc.rdx.behaviorTree.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiInputTextFlags;
import imgui.type.ImString;
import toolbox_msgs.KinematicsToolboxOutputStatus;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.behaviors.behaviorTree.action.actions.MimicActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.MimicActionDefinition.MimicActionType;
import us.ihmc.behaviors.behaviorTree.action.actions.MimicActionState;
import us.ihmc.communication.ROS2Input;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.behaviorTree.RDXROS2BehaviorTree;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;

public class RDXMimicAction extends RDXActionNode<MimicActionState, MimicActionDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper waitTimeExitPolicyWidget;
   private final ImString catalogName = new ImString("", 255);
   private final ROS2Input<KinematicsToolboxOutputStatus> status;
   private final FullHumanoidRobotModel ghostFullRobotModel;
   private final OneDoFJointBasics[] ghostOneDoFJointsExcludingHands;
   private final RDXMultiBodyGraphic ghostRobotGraphic;
   private boolean hasKinematicsStatus = false;

   public RDXMimicAction(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new MimicActionState(id, rootNode.getState()), rootNode);
      waitTimeExitPolicyWidget = new ImDoubleWrapper(definition::getWaitTimeExitPolicy,
                                                     definition::setWaitTimeExitPolicy,
                                                     imDouble -> ImGui.inputDouble(labels.get("Wait Time Exit Policy"), imDouble));
      catalogName.set(definition.getMimicFileName());

      ghostFullRobotModel = syncedRobot.getRobotModel().createFullRobotModel();
      ghostOneDoFJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(ghostFullRobotModel);
      ghostRobotGraphic = new RDXMultiBodyGraphic(syncedRobot.getRobotModel().getSimpleRobotName() + " (Mimic Ghost)");
      RobotDefinition ghostRobotDefinition = new RobotDefinition(syncedRobot.getRobotModel().getRobotDefinition());
      MaterialDefinition material = new MaterialDefinition(ColorDefinitions.parse("0xDEE934").derive(0.0, 1.0, 1.0, 0.5));
      RobotDefinition.forEachRigidBodyDefinition(ghostRobotDefinition.getRootBodyDefinition(),
                                                 body -> body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(material)));
      ghostRobotGraphic.loadRobotModelAndGraphics(ghostRobotDefinition, ghostFullRobotModel.getElevator());
      ghostRobotGraphic.setActive(true);
      ghostRobotGraphic.create();

      if (rootNode.getTree() instanceof RDXROS2BehaviorTree ros2BehaviorTree)
      {
         status = ros2BehaviorTree.getROS2ControllerHelper().subscribe(KinematicsStreamingToolboxModule.getOutputStatusTopic(syncedRobot.getRobotModel().getSimpleRobotName()));
      }
      else
      {
         status = null;
         LogTools.warn("RDXMimicAction: Behavior tree is not ROS2-backed. Ghost model KST subscription disabled.");
      }
   }

   @Override
   public void update()
   {
      super.update();

      String definitionFileName = definition.getMimicFileName();
      if (!definitionFileName.equals(catalogName.get()))
         catalogName.set(definitionFileName);

      if (status != null && status.getMessageNotification().poll())
      {
         KinematicsToolboxOutputStatus latestStatus = status.getMessageNotification().read();
         if (latestStatus.getJointNameHash() != -1)
         {
            hasKinematicsStatus = true;
            ghostFullRobotModel.getRootJoint().setJointPosition(latestStatus.getDesiredRootPosition().getPoint());
            ghostFullRobotModel.getRootJoint().setJointOrientation(latestStatus.getDesiredRootOrientation().getQuaternion());
            for (int i = 0; i < ghostOneDoFJointsExcludingHands.length; i++)
               ghostOneDoFJointsExcludingHands[i].setQ(latestStatus.getDesiredJointAngles().get(i));
            ghostFullRobotModel.getElevator().updateFramesRecursively();
         }
      }

      ghostRobotGraphic.update();
   }

   @Override
   public void renderTreeViewRow()
   {
      super.renderRowBeginning();
      super.renderEditableName();
      ImGui.sameLine();
      ImGui.textDisabled(getLeafTypeTitle());
      renderRowEnd();
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      MimicActionType currentActionType = definition.getMimicActionType().getValue();
      if (ImGui.beginCombo(labels.get("Mimic Action Type"), currentActionType.name()))
      {
         for (MimicActionType value : MimicActionType.values)
         {
            if (ImGui.selectable(value.name(), value == currentActionType))
               definition.getMimicActionType().setValue(value);
         }
         ImGui.endCombo();
      }

      if (definition.getMimicActionType().getValue() == MimicActionType.EXIT_POLICY)
      {
         ImGui.pushItemWidth(80.0f);
         waitTimeExitPolicyWidget.renderImGuiWidget();
         ImGui.popItemWidth();
      }

      if (definition.getMimicActionType().getValue() == MimicActionType.EXECUTE_POLICY)
      {
         if (ImGui.inputText(labels.get("Catalog Name"), catalogName, ImGuiInputTextFlags.EnterReturnsTrue))
            definition.setMimicFileName(catalogName.get().trim());
      }
   }

   @Override
   public String getLeafTypeTitle()
   {
      MimicActionType actionType = definition.getMimicActionType().getValue();
      if (actionType == MimicActionType.EXECUTE_POLICY && !definition.getMimicFileName().isBlank())
         return "Execute " + definition.getMimicFileName();
      return actionType.name();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (getState().getIsExecuting() && hasKinematicsStatus)
         ghostRobotGraphic.getRenderables(renderables, pool, baseUI.getPrimaryScene().getSceneLevelsToRender());
   }
}
