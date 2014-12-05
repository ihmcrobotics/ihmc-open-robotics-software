package us.ihmc.humanoidBehaviors.behaviors.diagnostic;

import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ChestOrientationBehavior;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.taskExecutor.Task;

public class ChestOrientationTask implements Task
{
   private static final boolean DEBUG = false;
   private final ChestOrientationPacket chestOrientationPacket;
   private final ChestOrientationBehavior chestOrientationBehavior;

   public ChestOrientationTask(FrameOrientation desiredChestOrientation, ChestOrientationBehavior chestOrientationBehavior, double trajectoryTime)
   {
      this.chestOrientationBehavior = chestOrientationBehavior;
      Quat4d chestOrientation = new Quat4d();
      desiredChestOrientation.getQuaternion(chestOrientation);
      chestOrientationPacket = new ChestOrientationPacket(chestOrientation, trajectoryTime);
   }

   @Override
   public void doTransitionIntoAction()
   {
      chestOrientationBehavior.initialize();
      chestOrientationBehavior.setInput(chestOrientationPacket);
   }

   @Override
   public void doAction()
   {
      chestOrientationBehavior.doControl();
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
      return chestOrientationBehavior.isDone();
   }
}