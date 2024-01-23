package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage;
import behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class FootstepPlanActionDefinition extends ActionNodeDefinition
{
   private final CRDTUnidirectionalDouble swingDuration;
   private final CRDTUnidirectionalDouble transferDuration;
   private final CRDTUnidirectionalString parentFrameName;
   private final CRDTUnidirectionalBoolean isManuallyPlaced;
   private final CRDTUnidirectionalRecyclingArrayList<FootstepPlanActionFootstepDefinition> footsteps;
   private final CRDTUnidirectionalRigidBodyTransform goalToParentTransform;
   private final SideDependentList<CRDTUnidirectionalRigidBodyTransform> goalFootstepToGoalTransforms;

   public FootstepPlanActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      swingDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 1.2);
      transferDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 0.8);
      parentFrameName = new CRDTUnidirectionalString(ROS2ActorDesignation.OPERATOR, crdtInfo, ReferenceFrame.getWorldFrame().getName());
      isManuallyPlaced = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.OPERATOR, crdtInfo, false);
      footsteps = new CRDTUnidirectionalRecyclingArrayList<>(ROS2ActorDesignation.OPERATOR,
                                                             crdtInfo,
                                                             () -> new RecyclingArrayList<>(() -> new FootstepPlanActionFootstepDefinition(crdtInfo)));
      goalToParentTransform = new CRDTUnidirectionalRigidBodyTransform(ROS2ActorDesignation.OPERATOR, crdtInfo);
      goalFootstepToGoalTransforms = new SideDependentList<>(() -> new CRDTUnidirectionalRigidBodyTransform(ROS2ActorDesignation.OPERATOR, crdtInfo));
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("swingDuration", swingDuration.getValue());
      jsonNode.put("transferDuration", transferDuration.getValue());
      jsonNode.put("parentFrame", parentFrameName.getValue());
      jsonNode.put("isManuallyPlaced", isManuallyPlaced.getValue());

      ArrayNode foostepsArrayNode = jsonNode.putArray("footsteps");
      for (int i = 0; i < footsteps.getSize(); i++)
      {
         ObjectNode footstepNode = foostepsArrayNode.addObject();
         footsteps.getValueReadOnly(i).saveToFile(footstepNode);
      }
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

      swingDuration.setValue(jsonNode.get("swingDuration").asDouble());
      transferDuration.setValue(jsonNode.get("transferDuration").asDouble());
      parentFrameName.setValue(jsonNode.get("parentFrame").textValue());
      isManuallyPlaced.setValue(jsonNode.get("isManuallyPlaced").booleanValue());

      footsteps.getValue().clear();
      JSONTools.forEachArrayElement(jsonNode, "footsteps", footstepNode -> footsteps.getValue().add().loadFromFile(footstepNode));
      JSONTools.toEuclid(jsonNode, goalToParentTransform.getValue());
      for (RobotSide side : RobotSide.values)
      {
         JsonNode goalFootNode = jsonNode.get(side.getCamelCaseName() + "GoalFootTransform");
         JSONTools.toEuclid(goalFootNode, goalFootstepToGoalTransforms.get(side).getValue());
      }
   }

   public void toMessage(FootstepPlanActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setSwingDuration(swingDuration.toMessage());
      message.setTransferDuration(transferDuration.toMessage());
      message.setParentFrameName(parentFrameName.toMessage());
      message.setIsManuallyPlaced(isManuallyPlaced.toMessage());

      message.getFootsteps().clear();
      for (int i = 0; i < footsteps.getSize(); i++)
      {
         footsteps.getValueReadOnly(i).toMessage(message.getFootsteps().add());
      }
      goalToParentTransform.toMessage(message.getGoalTransformToParent());
      goalFootstepToGoalTransforms.get(RobotSide.LEFT).toMessage(message.getLeftGoalFootTransformToGizmo());
      goalFootstepToGoalTransforms.get(RobotSide.RIGHT).toMessage(message.getRightGoalFootTransformToGizmo());
   }

   public void fromMessage(FootstepPlanActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      swingDuration.fromMessage(message.getSwingDuration());
      transferDuration.fromMessage(message.getTransferDuration());
      parentFrameName.fromMessage(message.getParentFrameNameAsString());
      isManuallyPlaced.fromMessage(message.getIsManuallyPlaced());

      footsteps.fromMessage(writableList ->
      {
         writableList.clear();
         for (FootstepPlanActionFootstepDefinitionMessage footstepMessage : message.getFootsteps())
         {
            writableList.add().fromMessage(footstepMessage);
         }
      });
      goalToParentTransform.fromMessage(message.getGoalTransformToParent());
      goalFootstepToGoalTransforms.get(RobotSide.LEFT).fromMessage(message.getLeftGoalFootTransformToGizmo());
      goalFootstepToGoalTransforms.get(RobotSide.RIGHT).fromMessage(message.getRightGoalFootTransformToGizmo());
   }

   public double getSwingDuration()
   {
      return swingDuration.getValue();
   }

   public void setSwingDuration(double swingDuration)
   {
      this.swingDuration.setValue(swingDuration);
   }

   public double getTransferDuration()
   {
      return transferDuration.getValue();
   }

   public void setTransferDuration(double transferDuration)
   {
      this.transferDuration.setValue(transferDuration);
   }

   public String getParentFrameName()
   {
      return parentFrameName.getValue();
   }

   public void setParentFrameName(String parentFrameName)
   {
      this.parentFrameName.setValue(parentFrameName);
   }

   public boolean getIsManuallyPlaced()
   {
      return isManuallyPlaced.getValue();
   }

   public void setInManuallyPlaced(boolean isManuallyPlaced)
   {
      this.isManuallyPlaced.setValue(isManuallyPlaced);
   }

   public CRDTUnidirectionalString getCRDTParentFrameName()
   {
      return parentFrameName;
   }

   public CRDTUnidirectionalRecyclingArrayList<FootstepPlanActionFootstepDefinition> getFootsteps()
   {
      return footsteps;
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
