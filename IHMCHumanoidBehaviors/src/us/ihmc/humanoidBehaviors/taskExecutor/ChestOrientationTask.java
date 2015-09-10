package us.ihmc.humanoidBehaviors.taskExecutor;

import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ChestOrientationBehavior;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;

public class ChestOrientationTask extends BehaviorTask
{
   private final ChestOrientationPacket chestOrientationPacket;
   private final ChestOrientationBehavior chestOrientationBehavior;

   public ChestOrientationTask(ChestOrientationPacket chestOrientationPacket, DoubleYoVariable yoTime, ChestOrientationBehavior chestOrientationBehavior)
   {
      this(chestOrientationPacket, yoTime, chestOrientationBehavior, 0.0);
   }

   public ChestOrientationTask(ChestOrientationPacket chestOrientationPacket, DoubleYoVariable yoTime, ChestOrientationBehavior chestOrientationBehavior, double sleepTime)
   {
      super(chestOrientationBehavior, yoTime, sleepTime);
      this.chestOrientationBehavior = chestOrientationBehavior;
      this.chestOrientationPacket = chestOrientationPacket;
   }
   
   public ChestOrientationTask(FrameOrientation desiredChestOrientation, DoubleYoVariable yoTime, ChestOrientationBehavior chestOrientationBehavior, double trajectoryTime)
   {
      this(desiredChestOrientation, yoTime, chestOrientationBehavior, trajectoryTime, 0.0);
   }

   public ChestOrientationTask(FrameOrientation desiredChestOrientation, DoubleYoVariable yoTime, ChestOrientationBehavior chestOrientationBehavior, double trajectoryTime, double sleepTime)
   {
      super(chestOrientationBehavior, yoTime, sleepTime);
      this.chestOrientationBehavior = chestOrientationBehavior;
      Quat4d chestOrientation = new Quat4d();
      desiredChestOrientation.getQuaternion(chestOrientation);
      chestOrientationPacket = new ChestOrientationPacket(chestOrientation, false, trajectoryTime);
   }

   @Override
   protected void setBehaviorInput()
   {
      chestOrientationBehavior.setInput(chestOrientationPacket);
   }
}