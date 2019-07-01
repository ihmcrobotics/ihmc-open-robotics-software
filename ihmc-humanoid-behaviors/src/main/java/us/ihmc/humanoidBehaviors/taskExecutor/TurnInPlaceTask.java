package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.TurnInPlaceBehavior;
import us.ihmc.yoVariables.variable.YoDouble;

public class TurnInPlaceTask extends BehaviorAction
{
   private double desiredYaw;
   private TurnInPlaceBehavior turnInPlaceBehavior;

   public TurnInPlaceTask(double desiredYaw, TurnInPlaceBehavior turnInPlaceBehavior, YoDouble yoTime)
   {
      this(desiredYaw, turnInPlaceBehavior);
   }

   public TurnInPlaceTask(double desiredYaw, TurnInPlaceBehavior turnInPlaceBehavior)
   {
      super(turnInPlaceBehavior);
      this.desiredYaw = desiredYaw;
      this.turnInPlaceBehavior = turnInPlaceBehavior;

   }

   @Override
   protected void setBehaviorInput()
   {
      turnInPlaceBehavior.setTarget(desiredYaw);
   }
}
