package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.math.trajectories.PoseTrajectoryGenerator;

public abstract class TrajectoryBasedTaskspaceHandControlState extends TaskspaceHandControlState
{
   public TrajectoryBasedTaskspaceHandControlState(String namePrefix, HandControlState stateEnum, MomentumBasedController momentumBasedController,
         int jacobianId, RigidBody base, RigidBody endEffector, YoVariableRegistry parentRegistry)
   {
      super(namePrefix, stateEnum, momentumBasedController, jacobianId, base, endEffector, parentRegistry);
   }

   public abstract void setTrajectory(PoseTrajectoryGenerator poseTrajectoryGenerator);

   public void setTrajectoryWithAngularControlQuality(PoseTrajectoryGenerator poseTrajectoryGenerator, double percentOfTrajectoryWithOrientationBeingControlled, double trajectoryTime)
   {
      setTrajectory(poseTrajectoryGenerator);
   }

   public abstract void setHoldPositionDuration(double holdPositionDuration);

   public abstract FramePose getDesiredPose();

   public abstract ReferenceFrame getReferenceFrame();
}
