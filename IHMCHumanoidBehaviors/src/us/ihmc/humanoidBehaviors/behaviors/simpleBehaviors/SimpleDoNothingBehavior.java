package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;

public class SimpleDoNothingBehavior extends AbstractBehavior
{
   public SimpleDoNothingBehavior(CommunicationBridgeInterface outgoingCommunicationBridge)
   {
      super(outgoingCommunicationBridge);
   }

   @Override
   public void doControl()
   {
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   public boolean hasInputBeenSet()
   {
      return true;
   }
}
