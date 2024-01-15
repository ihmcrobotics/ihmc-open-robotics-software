package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandPoseActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class HandPoseActionDefinition extends ActionNodeDefinition implements SidedObject
{
   private final CRDTUnidirectionalEnumField<RobotSide> side;
   private final CRDTUnidirectionalDouble trajectoryDuration;
   private final CRDTUnidirectionalBoolean holdPoseInWorldLater;
   private final CRDTUnidirectionalBoolean jointspaceOnly;
   private final CRDTUnidirectionalString palmParentFrameName;
   private final CRDTUnidirectionalRigidBodyTransform palmTransformToParent;

   public HandPoseActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      side = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.OPERATOR, crdtInfo, RobotSide.LEFT);
      trajectoryDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 4.0);
      holdPoseInWorldLater = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.OPERATOR, crdtInfo, false);
      jointspaceOnly = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.OPERATOR, crdtInfo, false);
      palmParentFrameName = new CRDTUnidirectionalString(ROS2ActorDesignation.OPERATOR, crdtInfo, ReferenceFrame.getWorldFrame().getName());
      palmTransformToParent = new CRDTUnidirectionalRigidBodyTransform(ROS2ActorDesignation.OPERATOR, crdtInfo);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("parentFrame", palmParentFrameName.getValue());
      JSONTools.toJSON(jsonNode, palmTransformToParent.getValueReadOnly());
      jsonNode.put("side", side.getValue().getLowerCaseName());
      jsonNode.put("trajectoryDuration", trajectoryDuration.getValue());
      jsonNode.put("holdPoseInWorldLater", holdPoseInWorldLater.getValue());
      jsonNode.put("jointSpaceControl", jointspaceOnly.getValue());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      side.setValue(RobotSide.getSideFromString(jsonNode.get("side").asText()));
      trajectoryDuration.setValue(jsonNode.get("trajectoryDuration").asDouble());
      palmParentFrameName.setValue(jsonNode.get("parentFrame").textValue());
      JSONTools.toEuclid(jsonNode, palmTransformToParent.getValue());
      holdPoseInWorldLater.setValue(jsonNode.get("holdPoseInWorldLater").asBoolean());
      jointspaceOnly.setValue(jsonNode.get("jointSpaceControl").asBoolean());
   }

   public void toMessage(HandPoseActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setParentFrameName(palmParentFrameName.toMessage());
      palmTransformToParent.toMessage(message.getTransformToParent());
      message.setRobotSide(side.toMessage().toByte());
      message.setTrajectoryDuration(trajectoryDuration.toMessage());
      message.setHoldPoseInWorld(holdPoseInWorldLater.toMessage());
      message.setJointSpaceControl(jointspaceOnly.toMessage());
   }

   public void fromMessage(HandPoseActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      palmParentFrameName.fromMessage(message.getParentFrameNameAsString());
      palmTransformToParent.fromMessage(message.getTransformToParent());
      side.fromMessage(RobotSide.fromByte(message.getRobotSide()));
      trajectoryDuration.fromMessage(message.getTrajectoryDuration());
      holdPoseInWorldLater.fromMessage(message.getHoldPoseInWorld());
      jointspaceOnly.fromMessage(message.getJointSpaceControl());
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

   public boolean getHoldPoseInWorldLater()
   {
      return holdPoseInWorldLater.getValue();
   }

   public void setHoldPoseInWorldLater(boolean holdPoseInWorldLater)
   {
      this.holdPoseInWorldLater.setValue(holdPoseInWorldLater);
   }

   public boolean getJointspaceOnly()
   {
      return jointspaceOnly.getValue();
   }

   public void setJointspaceOnly(boolean jointspaceOnly)
   {
      this.jointspaceOnly.setValue(jointspaceOnly);
   }

   public String getPalmParentFrameName()
   {
      return palmParentFrameName.getValue();
   }

   public void setPalmParentFrameName(String palmParentFrameName)
   {
      this.palmParentFrameName.setValue(palmParentFrameName);
   }

   public CRDTUnidirectionalString getCRDTPalmParentFrameName()
   {
      return palmParentFrameName;
   }

   public CRDTUnidirectionalRigidBodyTransform getPalmTransformToParent()
   {
      return palmTransformToParent;
   }
}
