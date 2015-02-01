package us.ihmc.humanoidBehaviors.taskExecutor;

import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ChestOrientationBehavior;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.taskExecutor.Task;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class ChestOrientationTask implements Task
{
   private static final boolean DEBUG = false;
   private final ChestOrientationPacket chestOrientationPacket;
   private final ChestOrientationBehavior chestOrientationBehavior;

   private final DoubleYoVariable yoTime;
   private double behaviorDoneTime = Double.NaN;
   private final double sleepTime;

   public ChestOrientationTask(FrameOrientation desiredChestOrientation, DoubleYoVariable yoTime, ChestOrientationBehavior chestOrientationBehavior,
         double trajectoryTime)
   {
      this(desiredChestOrientation, yoTime, chestOrientationBehavior, trajectoryTime, 0.0);
   }

   public ChestOrientationTask(FrameOrientation desiredChestOrientation, DoubleYoVariable yoTime, ChestOrientationBehavior chestOrientationBehavior,
         double trajectoryTime, double sleepTime)
   {
      this.chestOrientationBehavior = chestOrientationBehavior;
      this.yoTime = yoTime;
      this.sleepTime = sleepTime;
      Quat4d chestOrientation = new Quat4d();
      desiredChestOrientation.getQuaternion(chestOrientation);
      chestOrientationPacket = new ChestOrientationPacket(chestOrientation, false, trajectoryTime);
   }

   @Override
   public void doTransitionIntoAction()
   {
      if (DEBUG)
         System.out.println("Started chest orientation");
      chestOrientationBehavior.initialize();
      chestOrientationBehavior.setInput(chestOrientationPacket);
   }

   @Override
   public void doAction()
   {
      chestOrientationBehavior.doControl();
      if (Double.isNaN(behaviorDoneTime) && chestOrientationBehavior.isDone())
      {
         behaviorDoneTime = yoTime.getDoubleValue();
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      if (DEBUG)
         System.out.println("Finished chest orientation");
      chestOrientationBehavior.finalize();
   }

   @Override
   public boolean isDone()
   {
      boolean sleepTimeAchieved = yoTime.getDoubleValue() > behaviorDoneTime + sleepTime;
      return chestOrientationBehavior.isDone() && sleepTimeAchieved;
   }
}