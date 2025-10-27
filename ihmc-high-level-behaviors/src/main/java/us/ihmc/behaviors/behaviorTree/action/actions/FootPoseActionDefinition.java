package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.FootPoseActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.communication.crdt.*;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;
import us.ihmc.tools.io.JSONTools;

public class FootPoseActionDefinition extends ActionNodeDefinition implements SidedObject
{
   private final CRDTBidirectionalEnumField<RobotSide> side;
   private final CRDTBidirectionalDouble trajectoryDuration;
   private final CRDTBidirectionalString parentFrameName;
   private final CRDTBidirectionalRigidBodyTransform footToParentTransform;

   // On disk fields
   private RobotSide onDiskSide;
   private double onDiskTrajectoryDuration;
   private String onDiskParentFrameName;
   private final RigidBodyTransform onDiskFootToParentTransform = new RigidBodyTransform();

   public FootPoseActionDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      side = new CRDTBidirectionalEnumField<>(this, RobotSide.LEFT);
      trajectoryDuration = new CRDTBidirectionalDouble(this, 4.0);
      parentFrameName = new CRDTBidirectionalString(this, ReferenceFrame.getWorldFrame().getName());
      footToParentTransform = new CRDTBidirectionalRigidBodyTransform(this);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("side", side.getValue().getLowerCaseName());
      jsonNode.put("trajectoryDuration", trajectoryDuration.getValue());
      jsonNode.put("parentFrame", parentFrameName.getValue());
      JSONTools.toJSON(jsonNode, footToParentTransform.getValueReadOnly());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      side.setValue(RobotSide.getSideFromString(jsonNode.get("side").asText()));
      trajectoryDuration.setValue(jsonNode.get("trajectoryDuration").asDouble());
      parentFrameName.setValue(jsonNode.get("parentFrame").textValue());
      JSONTools.toEuclid(jsonNode, footToParentTransform.getValueAndModify());
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskSide = side.getValue();
      onDiskTrajectoryDuration = trajectoryDuration.getValue();
      onDiskParentFrameName = parentFrameName.getValue();
      onDiskFootToParentTransform.set(footToParentTransform.getValueReadOnly());
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      if (isUndoAvailable())
      {
         side.setValue(onDiskSide);
         trajectoryDuration.setValue(onDiskTrajectoryDuration);
         parentFrameName.setValue(onDiskParentFrameName);
         footToParentTransform.getValueAndModify().set(onDiskFootToParentTransform);
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= side.getValue() == onDiskSide;
      unchanged &= trajectoryDuration.getValue() == onDiskTrajectoryDuration;
      unchanged &= parentFrameName.getValue().equals(onDiskParentFrameName);
      unchanged &= footToParentTransform.getValueReadOnly().equals(onDiskFootToParentTransform);

      return !unchanged;
   }

   public void toMessage(FootPoseActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setRobotSide(side.toMessage().toByte());
      message.setParentFrameName(parentFrameName.toMessage());
      footToParentTransform.toMessage(message.getTransformToParent());
      message.setTrajectoryDuration(trajectoryDuration.toMessage());
   }

   public void fromMessage(FootPoseActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      side.fromMessage(RobotSide.fromByte(message.getRobotSide()));
      parentFrameName.fromMessage(message.getParentFrameNameAsString());
      footToParentTransform.fromMessage(message.getTransformToParent());
      trajectoryDuration.fromMessage(message.getTrajectoryDuration());
   }

   public RotationMatrixBasics getRotation()
   {
      return footToParentTransform.getValueAndModify().getRotation();
   }

   @Override
   public RobotSide getSide()
   {
      return side.getValue();
   }

   public void setSide(RobotSide side)
   {
      this.side.setValue(side);
   }

   public double getTrajectoryDuration()
   {
      return trajectoryDuration.getValue();
   }

   public void setTrajectoryDuration(double trajectoryDuration)
   {
      this.trajectoryDuration.setValue(trajectoryDuration);
   }

   public String getParentFrameName()
   {
      return parentFrameName.getValue();
   }

   public void setParentFrameName(String parentFrameName)
   {
      this.parentFrameName.setValue(parentFrameName);
   }

   public CRDTBidirectionalString getCRDTParentFrameName()
   {
      return parentFrameName;
   }

   public CRDTBidirectionalRigidBodyTransform getFootToParentTransform()
   {
      return footToParentTransform;
   }
}
