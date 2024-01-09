package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WalkActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalRigidBodyTransform;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class WalkActionDefinition extends ActionNodeDefinition
{
   private final FootstepPlanActionDefinitionBasics footstepPlanDefinitionBasics;
   private final CRDTUnidirectionalRigidBodyTransform goalToParentTransform;
   private final SideDependentList<CRDTUnidirectionalRigidBodyTransform> goalFootstepToGoalTransforms;

   public WalkActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      footstepPlanDefinitionBasics = new FootstepPlanActionDefinitionBasics(crdtInfo);
      goalToParentTransform = new CRDTUnidirectionalRigidBodyTransform(ROS2ActorDesignation.OPERATOR, crdtInfo);
      goalFootstepToGoalTransforms = new SideDependentList<>(() -> new CRDTUnidirectionalRigidBodyTransform(ROS2ActorDesignation.OPERATOR, crdtInfo));
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      footstepPlanDefinitionBasics.saveToFile(jsonNode);
      JSONTools.toJSON(jsonNode, goalToParentTransform.getValueReadOnly());
      for (RobotSide side : RobotSide.values)
      {
         ObjectNode goalFootNode = jsonNode.putObject(side.getCamelCaseName() + "GoalFootTransform");
         JSONTools.toJSON(goalFootNode, goalFootstepToGoalTransforms.get(side).getValueReadOnly());
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      footstepPlanDefinitionBasics.loadFromFile(jsonNode);
      JSONTools.toEuclid(jsonNode, goalToParentTransform.getValue());
      for (RobotSide side : RobotSide.values)
      {
         JsonNode goalFootNode = jsonNode.get(side.getCamelCaseName() + "GoalFootTransform");
         JSONTools.toEuclid(goalFootNode, goalFootstepToGoalTransforms.get(side).getValue());
      }
   }

   public void toMessage(WalkActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      footstepPlanDefinitionBasics.toMessage(message.getDefinitionBasics());
      goalToParentTransform.toMessage(message.getTransformToParent());
      goalFootstepToGoalTransforms.get(RobotSide.LEFT).toMessage(message.getLeftGoalFootTransformToGizmo());
      goalFootstepToGoalTransforms.get(RobotSide.RIGHT).toMessage(message.getRightGoalFootTransformToGizmo());
   }

   public void fromMessage(WalkActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      footstepPlanDefinitionBasics.fromMessage(message.getDefinitionBasics());
      goalToParentTransform.fromMessage(message.getTransformToParent());
      goalFootstepToGoalTransforms.get(RobotSide.LEFT).fromMessage(message.getLeftGoalFootTransformToGizmo());
      goalFootstepToGoalTransforms.get(RobotSide.RIGHT).fromMessage(message.getRightGoalFootTransformToGizmo());
   }

   public FootstepPlanActionDefinitionBasics getBasics()
   {
      return footstepPlanDefinitionBasics;
   }

   public CRDTUnidirectionalRigidBodyTransform getGoalToParentTransform()
   {
      return goalToParentTransform;
   }

   public CRDTUnidirectionalRigidBodyTransform getGoalFootstepToGoalTransform(RobotSide side)
   {
      return goalFootstepToGoalTransforms.get(side);
   }
}
