package us.ihmc.behaviors.sequence;

import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

// TODO: Include toMessage and fromMessage
public abstract class BehaviorActionState implements BehaviorActionDefinitionSupplier
{
   /** The action's unique ID. */
   private final long id;

   public BehaviorActionState()
   {
      // TODO: Make parameter
      id = BehaviorActionSequence.NEXT_ID.getAndIncrement();
   }

   public void update(ReferenceFrameLibrary referenceFrameLibrary)
   {

   }

   /** The action's unique ID. */
   public long getID()
   {
      return id;
   }

   public abstract BehaviorActionDefinition getDefinition();
}
