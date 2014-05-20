package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.posePlayback.PosePlaybackController;
import us.ihmc.commonWalkingControlModules.posePlayback.PosePlaybackPacket;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;

import com.yobotics.simulationconstructionset.DoubleYoVariable;

public class PosePlaybackControllerFactory implements HighLevelBehaviorFactory
{
   private final boolean transitionRequested;
   private final PosePlaybackPacket posePlaybackPacket;

   public PosePlaybackControllerFactory(PosePlaybackPacket posePlaybackPacket, boolean transitionRequested)
   {
      this.posePlaybackPacket = posePlaybackPacket;
      this.transitionRequested = transitionRequested;
   }

   @Override
   public HighLevelBehavior createHighLevelBehavior(MomentumBasedController momentumBasedController, ICPAndMomentumBasedController icpAndMomentumBasedController)
   {
      FullRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      DoubleYoVariable yoTime = momentumBasedController.getYoTime();
      double controlDT = momentumBasedController.getControlDT();
      PosePlaybackController posePlaybackController = new PosePlaybackController(fullRobotModel, yoTime, controlDT);
      posePlaybackController.setupForPosePlayblack(posePlaybackPacket);

      return posePlaybackController;
   }

   @Override
   public boolean isTransitionToBehaviorRequested()
   {
      return transitionRequested;
   }
}
