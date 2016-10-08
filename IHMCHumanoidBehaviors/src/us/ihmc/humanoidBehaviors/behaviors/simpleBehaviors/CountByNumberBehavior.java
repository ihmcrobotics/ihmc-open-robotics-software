package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;

public class CountByNumberBehavior extends AbstractBehavior
{

   boolean numbersSet = false;
   boolean mathComplete = false;
   int timesCounted = 0;
   double numberToCountBy;
   int timeToCount;
   double result = 0;

   public CountByNumberBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge)
   {
      super(outgoingCommunicationBridge);
   }

   public CountByNumberBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, double numberToCountBy, int timeToCount)
   {
      super(outgoingCommunicationBridge);

      setInput(numberToCountBy, timeToCount);
   }

   public void setInput(double numberToCountBy, int timeToCount)
   {
      this.numberToCountBy = numberToCountBy;
      this.timeToCount = timeToCount;
      numbersSet = true;
   }

   @Override
   public void doControl()
   {
      if (hasInputBeenSet() && !mathComplete)
      {
         result += numberToCountBy;
         timesCounted++;
         if (timesCounted >= timeToCount)
         {
            mathComplete = true;
         }
      }
   }

   public double getResult()
   {
      if (mathComplete)
         return result;
      else
         return Double.MIN_VALUE;
   }

   @Override
   public boolean isDone()
   {
      return mathComplete;
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      super.doPostBehaviorCleanup();
      numbersSet = false;
      mathComplete = false;
      timesCounted = 0;
      result = 0;
   }

   public boolean hasInputBeenSet()
   {
      return numbersSet;
   }
}
