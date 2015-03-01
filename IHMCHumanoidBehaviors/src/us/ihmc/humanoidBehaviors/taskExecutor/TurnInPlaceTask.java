package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.TurnInPlaceBehavior;
import us.ihmc.humanoidBehaviors.behaviors.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.WalkToLocationBehavior.WalkingOrientation;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class TurnInPlaceTask extends BehaviorTask
{
   
   private double sleepTime;
   private double desiredYaw;
   private TurnInPlaceBehavior turnInPlaceBehavior;

   private double transferTime;
   private double swingTime;
   
   public TurnInPlaceTask(double desiredYaw, TurnInPlaceBehavior turnInPlaceBehavior, double transferTime, double swingTime ,DoubleYoVariable yoTime, double sleepTime)
   {
      super(turnInPlaceBehavior, yoTime,sleepTime);
      this.desiredYaw = desiredYaw;
      this.turnInPlaceBehavior = turnInPlaceBehavior;
   }
   
   public TurnInPlaceTask(double desiredYaw, TurnInPlaceBehavior turnInPlaceBehavior, double transferTime, double swingTime ,DoubleYoVariable yoTime)
   {
      this(desiredYaw, turnInPlaceBehavior, transferTime, swingTime, yoTime, 0.0);
   }
   
   public TurnInPlaceTask(double desiredYaw, TurnInPlaceBehavior turnInPlaceBehavior, DoubleYoVariable yoTime)
   {
      this(desiredYaw, turnInPlaceBehavior, Double.NaN, Double.NaN ,yoTime, 0.0);
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
