package us.ihmc.humanoidBehaviors.upDownExploration;

/**
 * TODO Decide whether to remove. Might not be needed.
 */
public class UpDownSequence
{
   public enum UpDown
   {
      UP,
      DOWN;

      public UpDown opposite()
      {
         return this == UP ? DOWN : UP;
      }
   }

   private UpDown next;

   public UpDownSequence(UpDown firstUpDown)
   {
      next = firstUpDown;
   }

   public UpDown peek()
   {
      return next;
   }

   public UpDown poll()
   {
      UpDown polledValue = next;
      next = next.opposite();
      return polledValue;
   }
}
