package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.TurnInPlaceBehavior;
import us.ihmc.yoVariables.variable.YoDouble;

public class TurnInPlaceTask<E extends Enum<E>> extends BehaviorAction<E>
{
   private double desiredYaw;
   private TurnInPlaceBehavior turnInPlaceBehavior;

   private double transferTime;
   private double swingTime;

   public TurnInPlaceTask(double desiredYaw, TurnInPlaceBehavior turnInPlaceBehavior, YoDouble yoTime)
   {
      this(null,desiredYaw, turnInPlaceBehavior,yoTime);
   }

   public TurnInPlaceTask(double desiredYaw, TurnInPlaceBehavior turnInPlaceBehavior, double transferTime, double swingTime)
   {  
      this(null, desiredYaw,  turnInPlaceBehavior,  transferTime,  swingTime);
   }

   public TurnInPlaceTask(E stateEnum, double desiredYaw, TurnInPlaceBehavior turnInPlaceBehavior, YoDouble yoTime)
   {
      this(stateEnum, desiredYaw, turnInPlaceBehavior, Double.NaN, Double.NaN);
   }

   public TurnInPlaceTask(E stateEnum, double desiredYaw, TurnInPlaceBehavior turnInPlaceBehavior, double transferTime, double swingTime)
   {
      super(stateEnum, turnInPlaceBehavior);
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
