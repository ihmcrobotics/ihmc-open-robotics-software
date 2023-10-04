package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepActionDefinitionMessage;
import behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.JSONTools;

public class FootstepPlanActionState extends BehaviorActionState<FootstepPlanActionDefinitionMessage>
{
   private final FootstepPlanActionDefinition definition = new FootstepPlanActionDefinition();
   private final RecyclingArrayList<FootstepPlanActionFootstepState> footsteps;

   public FootstepPlanActionState(ReferenceFrameLibrary referenceFrameLibrary)
   {
      footsteps = new RecyclingArrayList<>(() -> new FootstepPlanActionFootstepState(referenceFrameLibrary, this));
   }

   @Override
   public void update()
   {
      for (FootstepPlanActionFootstepState footstep : footsteps)
      {
         footstep.update();
      }
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      ArrayNode foostepsArrayNode = jsonNode.putArray("footsteps");
      for (FootstepPlanActionFootstepState footstep : footsteps)
      {
         ObjectNode footstepNode = foostepsArrayNode.addObject();
         footstep.getDefinition().saveToFile(footstepNode);
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      footsteps.clear();
      JSONTools.forEachArrayElement(jsonNode, "footsteps", footstepNode -> footsteps.add().getDefinition().loadFromFile(footstepNode));
   }

   @Override
   public void toMessage(FootstepPlanActionDefinitionMessage message)
   {
      definition.toMessage(message);

      message.getFootsteps().clear();
      for (FootstepPlanActionFootstepState footstep : footsteps)
      {
         footstep.getDefinition().toMessage(message.getFootsteps().add());
      }
   }

   @Override
   public void fromMessage(FootstepPlanActionDefinitionMessage message)
   {
      definition.fromMessage(message);

      footsteps.clear();
      for (FootstepActionDefinitionMessage footstep : message.getFootsteps())
      {
         footsteps.add().getDefinition().fromMessage(footstep);
      }
   }

   public RecyclingArrayList<FootstepPlanActionFootstepState> getFootsteps()
   {
      return footsteps;
   }

   @Override
   public FootstepPlanActionDefinition getDefinition()
   {
      return definition;
   }
}
