package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepActionDefinitionMessage;
import behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.FrameBasedBehaviorActionDefinition;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.tools.io.JSONTools;

public class FootstepPlanActionDefinition extends FrameBasedBehaviorActionDefinition
{
   private String description = "Footstep plan";
   private double swingDuration = 1.2;
   private double transferDuration = 0.8;
   private final RecyclingArrayList<FootstepActionDefinition> footsteps = new RecyclingArrayList<>(FootstepActionDefinition::new);

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("parentFrame", getConditionalReferenceFrame().getConditionallyValidParentFrameName());
      JSONTools.toJSON(jsonNode, getTransformToParent());
      jsonNode.put("swingDuration", swingDuration);
      jsonNode.put("transferDuration", transferDuration);

      ArrayNode foostepsArrayNode = jsonNode.putArray("footsteps");
      for (FootstepActionDefinition footstep : footsteps)
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

   public void toMessage(FootstepPlanActionDefinitionMessage message)
   {
      message.setSwingDuration(swingDuration);
      message.setTransferDuration(transferDuration);
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(getConditionalReferenceFrame().getConditionallyValidParentFrameName());
      message.getFootsteps().clear();
      for (FootstepActionDefinition footstep : footsteps)
      {
         footstep.toMessage(message.getFootsteps().add());
      }
   }

   public void fromMessage(FootstepPlanActionDefinitionMessage message)
   {
      getConditionalReferenceFrame().setParentFrameName(message.getParentFrame().getString(0));
      MessageTools.toEuclid(message.getTransformToParent(), getTransformToParent());
      swingDuration = message.getSwingDuration();
      transferDuration = message.getTransferDuration();

      footsteps.clear();
      for (FootstepActionDefinitionMessage footstep : message.getFootsteps())
      {
         footsteps.add().fromMessage(footstep);
      }
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

   public RecyclingArrayList<FootstepActionDefinition> getFootsteps()
   {
      return footsteps;
   }

   @Override
   public void setDescription(String description)
   {
      this.description = description;
   }

   @Override
   public String getDescription()
   {
      return description;
   }
}
