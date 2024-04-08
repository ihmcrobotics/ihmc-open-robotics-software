package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class WholeBodyBimanipulationActionDefinition extends ActionNodeDefinition
{
   public static final int MAX_NUMBER_OF_JOINTS = 23;
   private final CRDTUnidirectionalString parentFrameName;
   private final CRDTUnidirectionalBoolean holdPoseInWorldLater;
   private final SideDependentList<CRDTUnidirectionalRigidBodyTransform> handToParentTransforms = new SideDependentList<>();
   private final CRDTUnidirectionalDouble trajectoryDuration;

   private double onDiskTrajectoryDuration;
   private boolean onDiskHoldPoseInWorldLater;
   private String onDiskParentFrameName;
   private final SideDependentList<RigidBodyTransform> onDiskHandToParentTransform = new SideDependentList<RigidBodyTransform>();

   public WholeBodyBimanipulationActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      holdPoseInWorldLater = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.OPERATOR, crdtInfo, false);
      parentFrameName = new CRDTUnidirectionalString(ROS2ActorDesignation.OPERATOR, crdtInfo, ReferenceFrame.getWorldFrame().getName());
      for (RobotSide side : RobotSide.values)
      {
         CRDTUnidirectionalRigidBodyTransform handToParentTransform = new CRDTUnidirectionalRigidBodyTransform(ROS2ActorDesignation.OPERATOR, crdtInfo);
         handToParentTransforms.put(side, handToParentTransform);
         onDiskHandToParentTransform.put(side, new RigidBodyTransform());
      }
      trajectoryDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 1.0);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("trajectoryDuration", trajectoryDuration.getValue());
      jsonNode.put("holdPoseInWorldLater", holdPoseInWorldLater.getValue());
      jsonNode.put("parentFrame", parentFrameName.getValue());
      for (RobotSide side : RobotSide.values)
      {
         ObjectNode goalFootNode = jsonNode.putObject(side.getCamelCaseName() + "HandPoseGoal");
         JSONTools.toJSON(goalFootNode, handToParentTransforms.get(side).getValueReadOnly());
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      trajectoryDuration.setValue(jsonNode.get("trajectoryDuration").asDouble());
      holdPoseInWorldLater.setValue(jsonNode.get("holdPoseInWorldLater").asBoolean());
      parentFrameName.setValue(jsonNode.get("parentFrame").textValue());
      for (RobotSide side : RobotSide.values)
      {
         ObjectNode goalFootNode = (ObjectNode) jsonNode.get(side.getCamelCaseName() + "HandPoseGoal");
         JSONTools.toEuclid(goalFootNode, handToParentTransforms.get(side).getValue());
      }
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskTrajectoryDuration = trajectoryDuration.getValue();
      onDiskHoldPoseInWorldLater = holdPoseInWorldLater.getValue();
      onDiskParentFrameName = parentFrameName.getValue();
      for (RobotSide side : RobotSide.values)
      {
         onDiskHandToParentTransform.get(side).set(handToParentTransforms.get(side).getValueReadOnly());
      }
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      trajectoryDuration.setValue(onDiskTrajectoryDuration);
      holdPoseInWorldLater.setValue(onDiskHoldPoseInWorldLater);
      parentFrameName.setValue(onDiskParentFrameName);
      for (RobotSide side : RobotSide.values)
      {
         handToParentTransforms.get(side).getValue().set(onDiskHandToParentTransform.get(side));
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= trajectoryDuration.getValue() == onDiskTrajectoryDuration;
      unchanged &= holdPoseInWorldLater.getValue() == onDiskHoldPoseInWorldLater;
      unchanged &= parentFrameName.getValue().equals(onDiskParentFrameName);
      for (RobotSide side : RobotSide.values)
      {
         unchanged &= handToParentTransforms.get(side).getValueReadOnly().equals(onDiskHandToParentTransform.get(side));
      }

      return !unchanged;
   }

   public void toMessage(WholeBodyBimanipulationActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setTrajectoryDuration(trajectoryDuration.toMessage());
      message.setHoldPoseInWorld(holdPoseInWorldLater.toMessage());
      message.setParentFrameName(parentFrameName.toMessage());
      handToParentTransforms.get(RobotSide.LEFT).toMessage(message.getLeftHandTransformToParent());
      handToParentTransforms.get(RobotSide.RIGHT).toMessage(message.getRightHandTransformToParent());
   }

   public void fromMessage(WholeBodyBimanipulationActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      trajectoryDuration.fromMessage(message.getTrajectoryDuration());
      holdPoseInWorldLater.fromMessage(message.getHoldPoseInWorld());
      parentFrameName.fromMessage(message.getParentFrameNameAsString());
      handToParentTransforms.get(RobotSide.LEFT).fromMessage(message.getLeftHandTransformToParent());
      handToParentTransforms.get(RobotSide.RIGHT).fromMessage(message.getRightHandTransformToParent());
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

   public String getParentFrameName()
   {
      return parentFrameName.getValue();
   }

   public void setParentFrameName(String parentFrameName)
   {
      this.parentFrameName.setValue(parentFrameName);
   }

   public CRDTUnidirectionalString getCRDTParentFrameName()
   {
      return parentFrameName;
   }

   public CRDTUnidirectionalRigidBodyTransform getHandToParentTransform(RobotSide side)
   {
      return handToParentTransforms.get(side);
   }
}
