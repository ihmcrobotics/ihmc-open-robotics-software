package us.ihmc.humanoidBehaviors.behaviors.coactiveElements;


import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.PickUpBallBehaviorStateMachine;

public class PickUpBallBehaviorCoactiveElementBehaviorSide extends PickUpBallBehaviorCoactiveElement
{
   private PickUpBallBehaviorStateMachine pickUpBallBehavior;
//   private HSVRange currentHSVRange = null;

   public void setPickUpBallBehavior(PickUpBallBehaviorStateMachine pickUpBallBehavior)
   {
      this.pickUpBallBehavior = pickUpBallBehavior;
   }

   @Override public void initializeUserInterfaceSide()
   {
   }

   @Override public void updateUserInterfaceSide()
   {
   }

   @Override
   public void initializeMachineSide()
   {
      machineSideCount.set(100);
   }

   @Override
   public void updateMachineSide()
   {
      if (abortAcknowledged.getBooleanValue() && (!abortClicked.getBooleanValue()))
      {
         abortAcknowledged.set(false);
      }

      if ((abortClicked.getBooleanValue()) && (!abortAcknowledged.getBooleanValue()))
      {
         if (pickUpBallBehavior != null)
         {
            pickUpBallBehavior.abort();
         }
         abortCount.increment();
         abortAcknowledged.set(true);
      }

//      HSVRange newHSVRange = new HSVRange(new HSVValue(minHue.getIntegerValue(), minSat.getIntegerValue(), minVal.getIntegerValue()),
//            new HSVValue(maxHue.getIntegerValue(), maxSat.getIntegerValue(), maxVal.getIntegerValue()));
//      pickUpBallBehavior.setHSVRange(newHSVRange);

      machineSideCount.increment();
   }
}
