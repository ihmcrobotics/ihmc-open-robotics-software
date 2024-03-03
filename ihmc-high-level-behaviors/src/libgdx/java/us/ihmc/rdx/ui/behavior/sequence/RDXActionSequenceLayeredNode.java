package us.ihmc.rdx.ui.behavior.sequence;

public class RDXActionSequenceLayeredNode
{
   private int numberOfConcurrentActions;

   public RDXActionSequenceLayeredNode()
   {
      reset();
   }

   public void reset()
   {
      numberOfConcurrentActions = 0;
   }

   public void setNumberOfConcurrentActions(int numberOfConcurrentActions)
   {
      this.numberOfConcurrentActions = numberOfConcurrentActions;
   }

   public int getNumberOfConcurrentActions()
   {
      return numberOfConcurrentActions;
   }
}
