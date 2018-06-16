package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoBoolean;

public class WaitForUserValidationBehavior extends AbstractBehavior
{

   private YoBoolean validClicked;
   private YoBoolean validAcknoledged;

   ExecutorService executorService = Executors.newFixedThreadPool(2);

   public WaitForUserValidationBehavior(String robotName, Ros2Node ros2Node, YoBoolean validClicked, YoBoolean validAcknoledged)
   {
      super(robotName, ros2Node);
      this.validAcknoledged = validAcknoledged;
      this.validClicked = validClicked;

   }

   public void reset()
   {
      validAcknoledged.set(false);
   }

   @Override
   public void doControl()
   {
      if (validClicked.getBooleanValue())
      {
         validAcknoledged.set(true);
      }
   }

   @Override
   public boolean isDone()
   {
      // return true;
      return validAcknoledged.getBooleanValue();
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

   @Override
   public void onBehaviorExited()
   {
   }
}
