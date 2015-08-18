package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.SdfLoader.SDFBaseFullRobotModel;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyPacketBehavior;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class WholeBodyPacketTask extends BehaviorTask
{
   private final SDFBaseFullRobotModel desiredFullRobotModel;
   private final WholeBodyPacketBehavior wholeBodyPacketBehavior;
   private double trajectoryDuration;
   
   public WholeBodyPacketTask(SDFBaseFullRobotModel desiredFullRobotModel, DoubleYoVariable yoTime, WholeBodyPacketBehavior wholeBodyPacketBehavior,
         double trajectoryTime, double sleepTime)
   {
      super(wholeBodyPacketBehavior, yoTime, sleepTime);
      this.desiredFullRobotModel = desiredFullRobotModel;
      this.wholeBodyPacketBehavior = wholeBodyPacketBehavior;
      this.trajectoryDuration = trajectoryTime;
   }

   public WholeBodyPacketTask(SDFBaseFullRobotModel desiredFullRobotModel, DoubleYoVariable yoTime, WholeBodyPacketBehavior wholeBodyPacketBehavior,
         double trajectoryTime)
   {
      this(desiredFullRobotModel, yoTime, wholeBodyPacketBehavior, trajectoryTime, 0.0);
   }

   @Override
   protected void setBehaviorInput()
   {
      wholeBodyPacketBehavior.setInputs(desiredFullRobotModel, trajectoryDuration);
   }
}
