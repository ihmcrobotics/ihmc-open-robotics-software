package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;

public class CountByNumberBehavior extends AbstractBehavior
{

   boolean numbersSet = false;
   boolean mathComplete = false;
   int timesCounted = 0;
   double numberToCountBy;
   int timeToCount;
   double result = 0;

   public CountByNumberBehavior(CommunicationBridgeInterface outgoingCommunicationBridge)
   {
      super(outgoingCommunicationBridge);
   }

   public CountByNumberBehavior(CommunicationBridgeInterface outgoingCommunicationBridge, double numberToCountBy, int timeToCount)
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
   public void onBehaviorExited()
   {
      numbersSet = false;
      mathComplete = false;
      timesCounted = 0;
      result = 0;
   }

   public boolean hasInputBeenSet()
   {
      return numbersSet;
   }

   @Override
   public void onBehaviorEntered()
   {
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }
}
