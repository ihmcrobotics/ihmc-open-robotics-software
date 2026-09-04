package us.ihmc.gr00t;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;

import java.nio.ByteBuffer;

/** Packs [left wrist, right wrist, neck pitch/yaw, left hand, right hand] state. */
public final class Gr00tBimanualStatePacker implements Gr00tStatePacker
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final Gr00tHandController hands;
   private final String neckPitchJointName;
   private final String neckYawJointName;
   private final FramePose3D framePose = new FramePose3D();

   public Gr00tBimanualStatePacker(ROS2SyncedRobotModel syncedRobot,
                                   Gr00tHandController hands,
                                   String neckPitchJointName,
                                   String neckYawJointName)
   {
      this.syncedRobot = syncedRobot;
      this.hands = hands;
      this.neckPitchJointName = neckPitchJointName;
      this.neckYawJointName = neckYawJointName;
   }

   @Override
   public boolean pack(ByteBuffer state)
   {
      synchronized (syncedRobot)
      {
         if (!syncedRobot.getDataReceptionTimerSnapshot().isRunning(0.1))
            return false;

         FullHumanoidRobotModel robotModel = syncedRobot.getFullRobotModel();
         for (RobotSide side : RobotSide.values)
         {
            framePose.setToZero(robotModel.getHand(side).getBodyFixedFrame());
            framePose.changeFrame(ReferenceFrame.getWorldFrame());
            state.putFloat(framePose.getPosition().getX32());
            state.putFloat(framePose.getPosition().getY32());
            state.putFloat(framePose.getPosition().getZ32());
            state.putFloat(framePose.getOrientation().getX32());
            state.putFloat(framePose.getOrientation().getY32());
            state.putFloat(framePose.getOrientation().getZ32());
            state.putFloat(framePose.getOrientation().getS32());
         }

         state.putFloat((float) robotModel.getOneDoFJointByName(neckPitchJointName).getQ());
         state.putFloat((float) robotModel.getOneDoFJointByName(neckYawJointName).getQ());
         for (RobotSide side : RobotSide.values)
         {
            for (double handJoint : hands.getJointPositions(side))
               state.putFloat((float) handJoint);
         }
      }
      return true;
   }
}
