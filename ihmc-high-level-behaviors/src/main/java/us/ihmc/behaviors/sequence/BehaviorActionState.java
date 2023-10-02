package us.ihmc.behaviors.sequence;

// TODO: Include toMessage and fromMessage
public interface BehaviorActionState
{
   /** The action's unique ID. */
   long getID();

   BehaviorActionDefinition getDefinition();
}
