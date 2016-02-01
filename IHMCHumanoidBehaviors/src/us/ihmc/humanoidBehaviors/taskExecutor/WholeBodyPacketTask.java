package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyPacketBehavior;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class WholeBodyPacketTask extends BehaviorTask
{
   private final SDFFullRobotModel desiredFullRobotModel;
   private final WholeBodyPacketBehavior wholeBodyPacketBehavior;
   private double trajectoryDuration;
   
   public WholeBodyPacketTask(SDFFullRobotModel desiredFullRobotModel, DoubleYoVariable yoTime, WholeBodyPacketBehavior wholeBodyPacketBehavior,
         double trajectoryTime, double sleepTime)
   {
      super(wholeBodyPacketBehavior, yoTime, sleepTime);
      this.desiredFullRobotModel = desiredFullRobotModel;
      this.wholeBodyPacketBehavior = wholeBodyPacketBehavior;
      this.trajectoryDuration = trajectoryTime;
   }

   public WholeBodyPacketTask(SDFFullRobotModel desiredFullRobotModel, DoubleYoVariable yoTime, WholeBodyPacketBehavior wholeBodyPacketBehavior,
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
