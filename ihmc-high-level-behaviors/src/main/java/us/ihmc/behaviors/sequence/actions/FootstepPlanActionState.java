package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepActionDefinitionMessage;
import behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.tools.io.JSONTools;

public class FootstepPlanActionState extends BehaviorActionState<FootstepPlanActionDefinitionMessage>
{
   private final FootstepPlanActionDefinition definition = new FootstepPlanActionDefinition();
   private final RecyclingArrayList<FootstepActionState> footsteps = new RecyclingArrayList<>(() -> new FootstepActionState(this));

   @Override
   public void update()
   {
      for (FootstepActionState footstep : footsteps)
      {
         footstep.update();
      }
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      definition.saveToFile(jsonNode);

      ArrayNode foostepsArrayNode = jsonNode.putArray("footsteps");
      for (FootstepActionState footstep : footsteps)
      {
         ObjectNode footstepNode = foostepsArrayNode.addObject();
         footstep.saveToFile(footstepNode);
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      footsteps.clear();
      JSONTools.forEachArrayElement(jsonNode, "footsteps", footstepNode -> footsteps.add().loadFromFile(footstepNode));
   }

   @Override
   public void toMessage(FootstepPlanActionDefinitionMessage message)
   {
      definition.toMessage(message);

      message.getFootsteps().clear();
      for (FootstepActionState footstep : footsteps)
      {
         footstep.toMessage(message.getFootsteps().add());
      }
   }

   @Override
   public void fromMessage(FootstepPlanActionDefinitionMessage message)
   {
      definition.fromMessage(message);

      footsteps.clear();
      for (FootstepActionDefinitionMessage footstep : message.getFootsteps())
      {
         footsteps.add().fromMessage(footstep);
      }
   }

   public RecyclingArrayList<FootstepActionState> getFootsteps()
   {
      return footsteps;
   }

   @Override
   public FootstepPlanActionDefinition getDefinition()
   {
      return definition;
   }
}
