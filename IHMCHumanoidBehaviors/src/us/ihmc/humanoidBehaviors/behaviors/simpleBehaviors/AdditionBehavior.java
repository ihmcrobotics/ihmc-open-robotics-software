package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;

public class AdditionBehavior extends AbstractBehavior
{

   boolean numbersSet = false;
   boolean mathComplete = false;
   double value1;
   double value2;
   double result;

   public AdditionBehavior(CommunicationBridgeInterface outgoingCommunicationBridge)
   {
      super(outgoingCommunicationBridge);
   }

   public AdditionBehavior(CommunicationBridgeInterface outgoingCommunicationBridge, double value1, double value2)
   {
      super(outgoingCommunicationBridge);

      setInput(value1, value2);
   }

   public void setInput(double value1, double value2)
   {
      this.value1 = value1;
      this.value2 = value2;
      numbersSet = true;
   }

   @Override
   public void doControl()
   {
      if (hasInputBeenSet() && !mathComplete)
      {
         result = value1 + value2;
         mathComplete = true;
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
