package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage;
import behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionDefinition;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.JSONTools;

public class FootstepPlanActionDefinition extends BehaviorActionDefinition
{
   private double swingDuration = 1.2;
   private double transferDuration = 0.8;
   private String parentFrameName;
   private final RecyclingArrayList<FootstepPlanActionFootstepDefinition> footsteps = new RecyclingArrayList<>(FootstepPlanActionFootstepDefinition::new);

   public FootstepPlanActionDefinition()
   {
      super("Footstep plan");
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("swingDuration", swingDuration);
      jsonNode.put("transferDuration", transferDuration);
      jsonNode.put("parentFrame", parentFrameName);

      ArrayNode foostepsArrayNode = jsonNode.putArray("footsteps");
      for (FootstepPlanActionFootstepDefinition footstep : footsteps)
      {
         ObjectNode footstepNode = foostepsArrayNode.addObject();
         footstep.saveToFile(footstepNode);
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      swingDuration = jsonNode.get("swingDuration").asDouble();
      transferDuration = jsonNode.get("transferDuration").asDouble();
      parentFrameName = jsonNode.get("parentFrame").textValue();

      footsteps.clear();
      JSONTools.forEachArrayElement(jsonNode, "footsteps", footstepNode -> footsteps.add().loadFromFile(footstepNode));
   }

   public void toMessage(FootstepPlanActionDefinitionMessage message)
   {
      super.toMessage(message.getActionDefinition());

      message.setSwingDuration(swingDuration);
      message.setTransferDuration(transferDuration);
      message.setParentFrameName(parentFrameName);

      message.getFootsteps().clear();
      for (FootstepPlanActionFootstepDefinition footstep : footsteps)
      {
         FootstepPlanActionFootstepDefinitionMessage footstepMessage = message.getFootsteps().add();
         footstepMessage.setRobotSide(footstep.getSide().toByte());
         footstepMessage.getSolePose().set(footstep.getSoleToPlanFrameTransform());
      }
   }

   public void fromMessage(FootstepPlanActionDefinitionMessage message)
   {
      super.fromMessage(message.getActionDefinition());

      swingDuration = message.getSwingDuration();
      transferDuration = message.getTransferDuration();
      parentFrameName = message.getParentFrameNameAsString();

      footsteps.clear();
      for (FootstepPlanActionFootstepDefinitionMessage footstepMessage : message.getFootsteps())
      {
         FootstepPlanActionFootstepDefinition footstep = footsteps.add();
         footstep.setSide(RobotSide.fromByte(footstepMessage.getRobotSide()));
         footstep.getSoleToPlanFrameTransform().set(footstepMessage.getSolePose());
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

   public String getParentFrameName()
   {
      return parentFrameName;
   }

   public void setParentFrameName(String parentFrameName)
   {
      this.parentFrameName = parentFrameName;
   }

   public RecyclingArrayList<FootstepPlanActionFootstepDefinition> getFootsteps()
   {
      return footsteps;
   }
}
