package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.TurnInPlaceBehavior;
import us.ihmc.yoVariables.variable.YoDouble;

public class TurnInPlaceTask extends BehaviorAction
{
   private double desiredYaw;
   private TurnInPlaceBehavior turnInPlaceBehavior;

   private double transferTime;
   private double swingTime;

   public TurnInPlaceTask(double desiredYaw, TurnInPlaceBehavior turnInPlaceBehavior, YoDouble yoTime)
   {
      this(desiredYaw, turnInPlaceBehavior, Double.NaN, Double.NaN);
   }

   public TurnInPlaceTask(double desiredYaw, TurnInPlaceBehavior turnInPlaceBehavior, double transferTime, double swingTime)
   {
      super(turnInPlaceBehavior);
      this.desiredYaw = desiredYaw;
      this.turnInPlaceBehavior = turnInPlaceBehavior;
      this.transferTime = transferTime;
      this.swingTime = swingTime;
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
