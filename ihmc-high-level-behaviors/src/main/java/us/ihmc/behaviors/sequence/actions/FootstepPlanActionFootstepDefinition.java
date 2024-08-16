package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalEnumField;
import us.ihmc.communication.crdt.CRDTUnidirectionalRigidBodyTransform;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;
import us.ihmc.tools.io.JSONTools;

public class FootstepPlanActionFootstepDefinition implements SidedObject
{
   private final CRDTUnidirectionalEnumField<RobotSide> side;
   private final CRDTUnidirectionalRigidBodyTransform soleToPlanFrameTransform;

   // On disk fields
   private RobotSide onDiskSide;
   private final RigidBodyTransform onDiskSoleToPlanFrameTransform = new RigidBodyTransform();

   public FootstepPlanActionFootstepDefinition(CRDTInfo crdtInfo)
   {
      side = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.OPERATOR, crdtInfo, RobotSide.LEFT);
      soleToPlanFrameTransform = new CRDTUnidirectionalRigidBodyTransform(ROS2ActorDesignation.OPERATOR, crdtInfo);
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

   public CRDTUnidirectionalRigidBodyTransform getSoleToPlanFrameTransform()
   {
      return soleToPlanFrameTransform;
   }

   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("side", side.getValue().getLowerCaseName());
      JSONTools.toJSON(jsonNode, soleToPlanFrameTransform.getValueReadOnly());
   }

   public void loadFromFile(JsonNode jsonNode)
   {
      side.setValue(RobotSide.getSideFromString(jsonNode.get("side").asText()));
      JSONTools.toEuclid(jsonNode, soleToPlanFrameTransform.getValue());
   }

   public void setOnDiskFields()
   {
      onDiskSide = side.getValue();
      onDiskSoleToPlanFrameTransform.set(soleToPlanFrameTransform.getValueReadOnly());
   }

   public void undoAllNontopologicalChanges()
   {
      side.setValue(onDiskSide);
      soleToPlanFrameTransform.getValue().set(onDiskSoleToPlanFrameTransform);
   }

   public boolean hasChanges()
   {
      boolean unchanged = true;

      unchanged &= side.getValue() == onDiskSide;
      unchanged &= soleToPlanFrameTransform.getValueReadOnly().equals(onDiskSoleToPlanFrameTransform);

      return !unchanged;
   }

   public void toMessage(FootstepPlanActionFootstepDefinitionMessage message)
   {
      message.setRobotSide(side.toMessage().toByte());
      soleToPlanFrameTransform.toMessage(message.getSolePose());
   }

   public void fromMessage(FootstepPlanActionFootstepDefinitionMessage message)
   {
      side.fromMessage(RobotSide.fromByte(message.getRobotSide()));
      soleToPlanFrameTransform.fromMessage(message.getSolePose());
   }
}
