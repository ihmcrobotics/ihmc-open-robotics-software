package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelBehavior;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.ICPAndMomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.posePlayback.PosePlaybackController;
import us.ihmc.commonWalkingControlModules.posePlayback.PosePlaybackPacket;
import us.ihmc.humanoidRobotics.model.FullRobotModel;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


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
   public HighLevelBehavior createHighLevelBehavior(VariousWalkingProviders variousWalkingProviders, VariousWalkingManagers variousWalkingManagers,
         MomentumBasedController momentumBasedController, ICPAndMomentumBasedController icpAndMomentumBasedController)
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
