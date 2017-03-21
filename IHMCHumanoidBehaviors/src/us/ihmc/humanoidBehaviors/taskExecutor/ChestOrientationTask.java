package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ChestTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ChestOrientationTask<E extends Enum<E>> extends BehaviorAction<E>
{
   private final ChestTrajectoryMessage chestOrientationPacket;
   private final ChestTrajectoryBehavior chestOrientationBehavior;

   public ChestOrientationTask(ChestTrajectoryMessage chestTrajectoryMessage, ChestTrajectoryBehavior chestOrientationBehavior)
   {
      this(null, chestTrajectoryMessage, chestOrientationBehavior);
   }

   public ChestOrientationTask(FrameOrientation desiredChestOrientation, ChestTrajectoryBehavior chestOrientationBehavior, double trajectoryTime, ReferenceFrame trajectoryFrame)
   {
      this(null, desiredChestOrientation, chestOrientationBehavior, trajectoryTime, trajectoryFrame);

   }

   public ChestOrientationTask(E stateEnum, ChestTrajectoryMessage chestTrajectoryMessage, ChestTrajectoryBehavior chestOrientationBehavior)
   {
      super(stateEnum, chestOrientationBehavior);
      this.chestOrientationBehavior = chestOrientationBehavior;
      this.chestOrientationPacket = chestTrajectoryMessage;
   }

   public ChestOrientationTask(E stateEnum, FrameOrientation desiredChestOrientation, ChestTrajectoryBehavior chestOrientationBehavior, double trajectoryTime, ReferenceFrame trajectoryFrame)
   {

      super(stateEnum, chestOrientationBehavior);
      this.chestOrientationBehavior = chestOrientationBehavior;
      Quaternion chestOrientation = new Quaternion();
      desiredChestOrientation.getQuaternion(chestOrientation);
      chestOrientationPacket = new ChestTrajectoryMessage(trajectoryTime, chestOrientation, ReferenceFrame.getWorldFrame(), trajectoryFrame);
   }

   @Override
   protected void setBehaviorInput()
   {
      chestOrientationBehavior.setInput(chestOrientationPacket);
   }
}