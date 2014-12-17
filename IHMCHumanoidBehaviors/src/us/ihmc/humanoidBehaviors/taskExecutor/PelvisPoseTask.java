package us.ihmc.humanoidBehaviors.taskExecutor;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.walking.PelvisPosePacket;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisPoseBehavior;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.taskExecutor.Task;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class PelvisPoseTask implements Task
{
   private static final boolean DEBUG = false;
   private final PelvisPosePacket pelvisPosePacket;
   private final PelvisPoseBehavior pelvisPoseBehavior;

   private final DoubleYoVariable yoTime;
   private double behaviorDoneTime = Double.NaN;
   private final double sleepTime;

   public PelvisPoseTask(FrameOrientation desiredPelvisOrientation, DoubleYoVariable yoTime, PelvisPoseBehavior pelvisPoseBehavior, double trajectoryTime)
   {
      this(desiredPelvisOrientation, yoTime, pelvisPoseBehavior, trajectoryTime, 0.0);
   }

   public PelvisPoseTask(FrameOrientation desiredPelvisOrientation, DoubleYoVariable yoTime, PelvisPoseBehavior pelvisPoseBehavior, double trajectoryTime,
         double sleepTime)
   {
      this.pelvisPoseBehavior = pelvisPoseBehavior;
      this.yoTime = yoTime;
      this.sleepTime = sleepTime;
      Point3d pelvisPosition = new Point3d();
      Quat4d pelvisOrientation = new Quat4d();
      desiredPelvisOrientation.getQuaternion(pelvisOrientation);
      pelvisPosePacket = new PelvisPosePacket(pelvisPosition, pelvisOrientation, trajectoryTime);
   }

   @Override
   public void doTransitionIntoAction()
   {
      if (DEBUG)
         System.out.println("Started pelvis pose");
      pelvisPoseBehavior.initialize();
      pelvisPoseBehavior.setInput(pelvisPosePacket);
   }

   @Override
   public void doAction()
   {
      pelvisPoseBehavior.doControl();
      if (Double.isNaN(behaviorDoneTime) && pelvisPoseBehavior.isDone())
      {
         behaviorDoneTime = yoTime.getDoubleValue();
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      if (DEBUG)
         System.out.println("Finished pelvis pose");
      pelvisPoseBehavior.finalize();
   }

   @Override
   public boolean isDone()
   {
      boolean sleepTimeAchieved = yoTime.getDoubleValue() > behaviorDoneTime + sleepTime;
      return pelvisPoseBehavior.isDone() && sleepTimeAchieved;
   }
}
