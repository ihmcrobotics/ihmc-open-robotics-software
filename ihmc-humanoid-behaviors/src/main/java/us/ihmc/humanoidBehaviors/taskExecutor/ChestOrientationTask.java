package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ChestTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;

public class ChestOrientationTask<E extends Enum<E>> extends BehaviorAction<E>
{
   private final ChestTrajectoryMessage chestOrientationPacket;
   private final ChestTrajectoryBehavior chestOrientationBehavior;

   public ChestOrientationTask(ChestTrajectoryMessage chestTrajectoryMessage, ChestTrajectoryBehavior chestOrientationBehavior)
   {
      this(null, chestTrajectoryMessage, chestOrientationBehavior);
   }

   public ChestOrientationTask(FrameQuaternion desiredChestOrientation, ChestTrajectoryBehavior chestOrientationBehavior, double trajectoryTime, ReferenceFrame trajectoryFrame)
   {
      this(null, desiredChestOrientation, chestOrientationBehavior, trajectoryTime, trajectoryFrame);

   }

   public ChestOrientationTask(E stateEnum, ChestTrajectoryMessage chestTrajectoryMessage, ChestTrajectoryBehavior chestOrientationBehavior)
   {
      super(stateEnum, chestOrientationBehavior);
      this.chestOrientationBehavior = chestOrientationBehavior;
      this.chestOrientationPacket = chestTrajectoryMessage;
   }

   public ChestOrientationTask(E stateEnum, FrameQuaternion desiredChestOrientation, ChestTrajectoryBehavior chestOrientationBehavior, double trajectoryTime, ReferenceFrame trajectoryFrame)
   {

      super(stateEnum, chestOrientationBehavior);
      this.chestOrientationBehavior = chestOrientationBehavior;
      Quaternion chestOrientation = new Quaternion(desiredChestOrientation);
      chestOrientationPacket = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime, chestOrientation, ReferenceFrame.getWorldFrame(), trajectoryFrame);
   }

   @Override
   protected void setBehaviorInput()
   {
      chestOrientationBehavior.setInput(chestOrientationPacket);
   }
}