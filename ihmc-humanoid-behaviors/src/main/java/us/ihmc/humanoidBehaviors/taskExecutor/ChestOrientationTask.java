package us.ihmc.humanoidBehaviors.taskExecutor;

import controller_msgs.msg.dds.ChestTrajectoryMessage;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ChestTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;

public class ChestOrientationTask extends BehaviorAction
{
   private final ChestTrajectoryMessage chestOrientationPacket;
   private final ChestTrajectoryBehavior chestOrientationBehavior;

   public ChestOrientationTask(ChestTrajectoryMessage chestTrajectoryMessage, ChestTrajectoryBehavior chestOrientationBehavior)
   {
      super(chestOrientationBehavior);
      this.chestOrientationBehavior = chestOrientationBehavior;
      this.chestOrientationPacket = chestTrajectoryMessage;
   }

   public ChestOrientationTask(FrameQuaternion desiredChestOrientation, ChestTrajectoryBehavior chestOrientationBehavior, double trajectoryTime,
                               ReferenceFrame trajectoryFrame)
   {
      super(chestOrientationBehavior);
      this.chestOrientationBehavior = chestOrientationBehavior;
      Quaternion chestOrientation = new Quaternion(desiredChestOrientation);
      chestOrientationPacket = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime, chestOrientation, ReferenceFrame.getWorldFrame(),
                                                                                 trajectoryFrame);
   }

   @Override
   protected void setBehaviorInput()
   {
      chestOrientationBehavior.setInput(chestOrientationPacket);
   }
}