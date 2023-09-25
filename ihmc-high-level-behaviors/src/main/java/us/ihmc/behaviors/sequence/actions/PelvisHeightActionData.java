package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.BodyPartPoseActionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.FrameBasedBehaviorActionData;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.tools.io.JSONTools;

public class PelvisHeightActionData extends FrameBasedBehaviorActionData
{
   private String description = "Pelvis height";
   private double trajectoryDuration = 4.0;

   @Override
   public String getDescription()
   {
      return description;
   }

   @Override
   public void setDescription(String description)
   {
      this.description = description;
   }

   public double getHeight()
   {
      return getTransformToParent().getTranslationZ();
   }

   public void setHeight(double height)
   {
      getTransformToParent().getTranslation().set(getTransformToParent().getTranslationX(), getTransformToParent().getTranslationY(), height);
   }

   public double getTrajectoryDuration()
   {
      return trajectoryDuration;
   }

   public void setTrajectoryDuration(double trajectoryDuration)
   {
      this.trajectoryDuration = trajectoryDuration;
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("trajectoryDuration", trajectoryDuration);
      jsonNode.put("parentFrame", getConditionalReferenceFrame().getConditionallyValidParentFrameName());
      JSONTools.toJSON(jsonNode, getTransformToParent());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
      getConditionalReferenceFrame().setParentFrameName(jsonNode.get("parentFrame").textValue());
      JSONTools.toEuclid(jsonNode, getTransformToParent());
   }

   public void toMessage(BodyPartPoseActionMessage message)
   {
      message.setTrajectoryDuration(trajectoryDuration);
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(getConditionalReferenceFrame().getConditionallyValidParentFrameName());
      MessageTools.toMessage(getTransformToParent(), message.getTransformToParent());
   }

   public void fromMessage(BodyPartPoseActionMessage message)
   {
      trajectoryDuration = message.getTrajectoryDuration();
      getConditionalReferenceFrame().setParentFrameName(message.getParentFrame().getString(0));
      MessageTools.toEuclid(message.getTransformToParent(), getTransformToParent());
   }
}
