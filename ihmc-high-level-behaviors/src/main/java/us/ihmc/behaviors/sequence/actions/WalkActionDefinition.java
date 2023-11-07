package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WalkActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalDouble;
import us.ihmc.communication.crdt.CRDTUnidirectionalRigidBodyTransform;
import us.ihmc.communication.crdt.CRDTUnidirectionalString;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.JSONTools;

public class WalkActionDefinition extends ActionNodeDefinition
{
   private final CRDTUnidirectionalDouble swingDuration;
   private final CRDTUnidirectionalDouble transferDuration;
   private final CRDTUnidirectionalString parentFrameName;
   private final CRDTUnidirectionalRigidBodyTransform goalToParentTransform;
   private final SideDependentList<CRDTUnidirectionalRigidBodyTransform> goalFootstepToGoalTransforms;

   public WalkActionDefinition(CRDTInfo crdtInfo)
   {
      super(crdtInfo);

      swingDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 1.2);
      transferDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 0.8);
      parentFrameName = new CRDTUnidirectionalString(ROS2ActorDesignation.OPERATOR, crdtInfo, null);
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
      JSONTools.toJSON(jsonNode, goalToParentTransform.getValue());
      for (RobotSide side : RobotSide.values)
      {
         ObjectNode goalFootNode = jsonNode.putObject(side.getCamelCaseName() + "GoalFootTransform");
         JSONTools.toJSON(goalFootNode, goalFootstepToGoalTransforms.get(side).getValue());
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      swingDuration.setValue(jsonNode.get("swingDuration").asDouble());
      transferDuration.setValue(jsonNode.get("transferDuration").asDouble());
      parentFrameName.setValue(jsonNode.get("parentFrame").textValue());
      goalToParentTransform.setValue(goalToParentTransform -> JSONTools.toEuclid(jsonNode, goalToParentTransform));
      for (RobotSide side : RobotSide.values)
      {
         goalFootstepToGoalTransforms.get(side).setValue(goalFootstepToGoalTransform ->
         {
            JsonNode goalFootNode = jsonNode.get(side.getCamelCaseName() + "GoalFootTransform");
            JSONTools.toEuclid(goalFootNode, goalFootstepToGoalTransform);
         });
      }
   }

   public void toMessage(WalkActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setSwingDuration(swingDuration.toMessage());
      message.setTransferDuration(transferDuration.toMessage());
      message.setParentFrameName(parentFrameName.toMessage());
      goalToParentTransform.toMessage(message.getTransformToParent());
      goalFootstepToGoalTransforms.get(RobotSide.LEFT).toMessage(message.getLeftGoalFootTransformToGizmo());
      goalFootstepToGoalTransforms.get(RobotSide.RIGHT).toMessage(message.getRightGoalFootTransformToGizmo());
   }

   public void fromMessage(WalkActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      swingDuration.fromMessage(message.getSwingDuration());
      transferDuration.fromMessage(message.getTransferDuration());
      parentFrameName.fromMessage(message.getParentFrameNameAsString());
      goalToParentTransform.fromMessage(message.getTransformToParent());
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

   public CRDTUnidirectionalRigidBodyTransform getGoalFootstepToGoalTransform(RobotSide side)
   {
      return goalFootstepToGoalTransforms.get(side);
   }

   public String getParentFrameName()
   {
      return parentFrameName.getValue();
   }

   public void setParentFrameName(String parentFrameName)
   {
      this.parentFrameName.setValue(parentFrameName);
   }

   public CRDTUnidirectionalRigidBodyTransform getGoalToParentTransform()
   {
      return goalToParentTransform;
   }
}
