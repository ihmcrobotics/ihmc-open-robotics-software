package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.TurnInPlaceBehavior;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class TurnInPlaceTask extends BehaviorTask
{
   private double desiredYaw;
   private TurnInPlaceBehavior turnInPlaceBehavior;

   private double transferTime;
   private double swingTime;

   public TurnInPlaceTask(double desiredYaw, TurnInPlaceBehavior turnInPlaceBehavior, double transferTime, double swingTime, DoubleYoVariable yoTime)
   {
      super(turnInPlaceBehavior, yoTime);
      this.desiredYaw = desiredYaw;
      this.turnInPlaceBehavior = turnInPlaceBehavior;
      this.transferTime = transferTime;
      this.swingTime = swingTime;
   }

   public TurnInPlaceTask(double desiredYaw, TurnInPlaceBehavior turnInPlaceBehavior, DoubleYoVariable yoTime)
   {
      this(desiredYaw, turnInPlaceBehavior, Double.NaN, Double.NaN, yoTime);
   }

   @Override
   protected void setBehaviorInput()
   {
      if (!Double.isNaN(swingTime))
      {
         turnInPlaceBehavior.setSwingTime(swingTime);
      }
      if (!Double.isNaN(transferTime))
      {
         turnInPlaceBehavior.setTransferTime(transferTime);
      }
      turnInPlaceBehavior.setTarget(desiredYaw);
   }
}
