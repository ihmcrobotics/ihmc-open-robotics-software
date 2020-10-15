package us.ihmc.humanoidBehaviors.stairs;

import controller_msgs.msg.dds.HeadTrajectoryMessage;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.stateMachine.core.State;

public class TraverseStairsSquareUpState implements State
{
   private final BehaviorHelper helper;
   private final TraverseStairsBehaviorParameters parameters;
   private final RemoteHumanoidRobotInterface robotInterface;
   private final RemoteSyncedRobotModel syncedRobotModel;

   private static final boolean SEND_PELVIS_AND_CHEST_TRAJECTORIES = false;

   public TraverseStairsSquareUpState(BehaviorHelper helper, TraverseStairsBehaviorParameters parameters)
   {
      this.helper = helper;
      this.parameters = parameters;
      this.robotInterface = helper.getOrCreateRobotInterface();
      this.syncedRobotModel = robotInterface.newSyncedRobot();
   }

   @Override
   public void onEntry()
   {
      LogTools.info("Entering " + getClass().getSimpleName());

      double trajectoryTime = parameters.get(TraverseStairsBehaviorParameters.trajectoryTime);

      if (SEND_PELVIS_AND_CHEST_TRAJECTORIES)
      {
         // center pelvis
         robotInterface.requestPelvisGoHome(trajectoryTime);

         // pitch chest forward
         double chestPitch = parameters.get(TraverseStairsBehaviorParameters.chestPitch);

         syncedRobotModel.update();
         ReferenceFrame midFeetZUpFrame = syncedRobotModel.getReferenceFrames().getMidFeetZUpFrame();
         FrameQuaternion chestOrientation = new FrameQuaternion(midFeetZUpFrame, 0.0, chestPitch, 0.0);
         chestOrientation.changeFrame(ReferenceFrame.getWorldFrame());
         robotInterface.requestChestOrientationTrajectory(trajectoryTime, chestOrientation, ReferenceFrame.getWorldFrame(), syncedRobotModel.getReferenceFrames().getPelvisZUpFrame());

         // pitch head forward
         double headPitch = parameters.get(TraverseStairsBehaviorParameters.headPitch);

         Quaternion headOrientation = new Quaternion(0.0, headPitch, 0.0);
         HeadTrajectoryMessage headTrajectoryMessage = HumanoidMessageTools.createHeadTrajectoryMessage(1.0,
                                                                                                        headOrientation,
                                                                                                        syncedRobotModel.getReferenceFrames().getChestFrame(),
                                                                                                        syncedRobotModel.getReferenceFrames().getChestFrame());
         robotInterface.requestHeadOrientationTrajectory(headTrajectoryMessage);
      }
   }

   @Override
   public void doAction(double timeInState)
   {
   }

   @Override
   public boolean isDone(double timeInState)
   {
      double trajectoryTime = parameters.get(TraverseStairsBehaviorParameters.trajectoryTime);
      if (timeInState >= trajectoryTime)
      {
         LogTools.info(getClass().getSimpleName() + " is done");
         return true;
      }
      else
      {
         return false;
      }
   }
}
