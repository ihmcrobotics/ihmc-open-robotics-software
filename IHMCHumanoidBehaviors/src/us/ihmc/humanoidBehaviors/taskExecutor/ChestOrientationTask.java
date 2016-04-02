package us.ihmc.humanoidBehaviors.taskExecutor;

import javax.vecmath.Quat4d;

import us.ihmc.humanoidBehaviors.behaviors.primitives.ChestTrajectoryBehavior;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;

public class ChestOrientationTask extends BehaviorTask
{
   private final ChestTrajectoryMessage chestOrientationPacket;
   private final ChestTrajectoryBehavior chestOrientationBehavior;

   public ChestOrientationTask(ChestTrajectoryMessage chestTrajectoryMessage, DoubleYoVariable yoTime, ChestTrajectoryBehavior chestOrientationBehavior)
   {
      this(chestTrajectoryMessage, yoTime, chestOrientationBehavior, 0.0);
   }

   public ChestOrientationTask(ChestTrajectoryMessage chestTrajectoryMessage, DoubleYoVariable yoTime, ChestTrajectoryBehavior chestOrientationBehavior, double sleepTime)
   {
      super(chestOrientationBehavior, yoTime, sleepTime);
      this.chestOrientationBehavior = chestOrientationBehavior;
      this.chestOrientationPacket = chestTrajectoryMessage;
   }
   
   public ChestOrientationTask(FrameOrientation desiredChestOrientation, DoubleYoVariable yoTime, ChestTrajectoryBehavior chestOrientationBehavior, double trajectoryTime)
   {
      this(desiredChestOrientation, yoTime, chestOrientationBehavior, trajectoryTime, 0.0);
   }

   public ChestOrientationTask(FrameOrientation desiredChestOrientation, DoubleYoVariable yoTime, ChestTrajectoryBehavior chestOrientationBehavior, double trajectoryTime, double sleepTime)
   {
      super(chestOrientationBehavior, yoTime, sleepTime);
      this.chestOrientationBehavior = chestOrientationBehavior;
      Quat4d chestOrientation = new Quat4d();
      desiredChestOrientation.getQuaternion(chestOrientation);
      chestOrientationPacket = new ChestTrajectoryMessage(trajectoryTime, chestOrientation);
   }

   @Override
   protected void setBehaviorInput()
   {
      chestOrientationBehavior.setInput(chestOrientationPacket);
   }
}