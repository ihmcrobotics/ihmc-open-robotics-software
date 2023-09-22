package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepActionMessage;
import behavior_msgs.msg.dds.FootstepPlanActionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.FrameBasedBehaviorActionData;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.tools.io.JSONTools;

public class FootstepPlanActionData extends FrameBasedBehaviorActionData
{
   private String description = "Footstep plan";
   private double swingDuration = 1.2;
   private double transferDuration = 0.8;
   private final RecyclingArrayList<FootstepActionData> footsteps = new RecyclingArrayList<>(FootstepActionData::new);

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

   public double getSwingDuration()
   {
      return swingDuration;
   }

   public void setSwingDuration(double swingDuration)
   {
      this.swingDuration = swingDuration;
   }

   public double getTransferDuration()
   {
      return transferDuration;
   }

   public void setTransferDuration(double transferDuration)
   {
      this.transferDuration = transferDuration;
   }

   public RecyclingArrayList<FootstepActionData> getFootsteps()
   {
      return footsteps;
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("swingDuration", swingDuration);
      jsonNode.put("transferDuration", transferDuration);
      jsonNode.put("parentFrame", getConditionalReferenceFrame().getParentFrameName());
      ArrayNode foostepsArrayNode = jsonNode.putArray("footsteps");
      for (FootstepActionData footstep : footsteps)
      {
         ObjectNode footstepNode = foostepsArrayNode.addObject();
         footstep.saveToFile(footstepNode);
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      swingDuration = jsonNode.get("swingDuration").asDouble();
      transferDuration = jsonNode.get("transferDuration").asDouble();
      getConditionalReferenceFrame().setParentFrameName(jsonNode.get("parentFrame").textValue());
      footsteps.clear();
      JSONTools.forEachArrayElement(jsonNode, "footsteps", footstepNode -> footsteps.add().loadFromFile(footstepNode));
   }

   public void toMessage(FootstepPlanActionMessage message)
   {
      message.setSwingDuration(swingDuration);
      message.setTransferDuration(transferDuration);
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(getConditionalReferenceFrame().getParentFrameName());
      message.getFootsteps().clear();
      for (FootstepActionData footstep : footsteps)
      {
         footstep.toMessage(message.getFootsteps().add());
      }
   }

   public void fromMessage(FootstepPlanActionMessage message)
   {
      swingDuration = message.getSwingDuration();
      transferDuration = message.getTransferDuration();
      getConditionalReferenceFrame().setParentFrameName(message.getParentFrame().getString(0));
      footsteps.clear();
      for (FootstepActionMessage footstep : message.getFootsteps())
      {
         footsteps.add().fromMessage(footstep);
      }
   }
}
