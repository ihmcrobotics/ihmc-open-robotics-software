package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionDefinition;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.tools.io.JSONTools;

public class ChestOrientationActionDefinition extends BehaviorActionDefinition
{
   private double trajectoryDuration = 4.0;
   private boolean holdPoseInWorldLater = false;
   private String parentFrameName;
   private final RigidBodyTransform chestToParentTransform = new RigidBodyTransform();

   public ChestOrientationActionDefinition()
   {
      super("Chest orientation");
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("trajectoryDuration", trajectoryDuration);
      jsonNode.put("holdPoseInWorldLater", holdPoseInWorldLater);
      jsonNode.put("parentFrame", parentFrameName);
      JSONTools.toJSON(jsonNode, chestToParentTransform);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
      holdPoseInWorldLater = jsonNode.get("holdPoseInWorldLater").asBoolean();
      parentFrameName = jsonNode.get("parentFrame").textValue();
      JSONTools.toEuclid(jsonNode, chestToParentTransform);
   }

   public void toMessage(ChestOrientationActionDefinitionMessage message)
   {
      super.toMessage(message.getActionDefinition());

      message.setTrajectoryDuration(trajectoryDuration);
      message.setHoldPoseInWorld(holdPoseInWorldLater);
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(parentFrameName);
      MessageTools.toMessage(chestToParentTransform, message.getChestTransformToParent());
   }

   public void fromMessage(ChestOrientationActionDefinitionMessage message)
   {
      super.fromMessage(message.getActionDefinition());

      trajectoryDuration = message.getTrajectoryDuration();
      holdPoseInWorldLater = message.getHoldPoseInWorld();
      parentFrameName = message.getParentFrame().getString(0);
      MessageTools.toEuclid(message.getChestTransformToParent(), chestToParentTransform);
   }

   public void setYaw(double yaw)
   {
      RotationMatrixBasics rotation = chestToParentTransform.getRotation();
      chestToParentTransform.getRotation().setYawPitchRoll(yaw, rotation.getPitch(), rotation.getRoll());
   }

   public void setPitch(double pitch)
   {
      RotationMatrixBasics rotation = chestToParentTransform.getRotation();
      chestToParentTransform.getRotation().setYawPitchRoll(rotation.getYaw(), pitch, rotation.getRoll());
   }

   public void setRoll(double roll)
   {
      RotationMatrixBasics rotation = chestToParentTransform.getRotation();
      chestToParentTransform.getRotation().setYawPitchRoll(rotation.getYaw(), rotation.getPitch(), roll);
   }

   public RotationMatrixBasics getRotation()
   {
      return chestToParentTransform.getRotation();
   }

   public double getTrajectoryDuration()
   {
      return trajectoryDuration;
   }

   public void setTrajectoryDuration(double trajectoryDuration)
   {
      this.trajectoryDuration = trajectoryDuration;
   }

   public boolean getHoldPoseInWorldLater()
   {
      return holdPoseInWorldLater;
   }

   public void setHoldPoseInWorldLater(boolean holdPoseInWorldLater)
   {
      this.holdPoseInWorldLater = holdPoseInWorldLater;
   }

   public String getParentFrameName()
   {
      return parentFrameName;
   }

   public void setParentFrameName(String parentFrameName)
   {
      this.parentFrameName = parentFrameName;
   }

   public RigidBodyTransform getChestToParentTransform()
   {
      return chestToParentTransform;
   }
}
