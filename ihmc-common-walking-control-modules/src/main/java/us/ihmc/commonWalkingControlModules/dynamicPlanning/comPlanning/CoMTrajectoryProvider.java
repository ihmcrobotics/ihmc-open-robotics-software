package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.robotics.math.trajectories.Trajectory3D;

import java.util.List;

public interface CoMTrajectoryProvider extends CoMTrajectoryPlannerInterface
{
   List<Trajectory3D> getVRPTrajectories();
}
