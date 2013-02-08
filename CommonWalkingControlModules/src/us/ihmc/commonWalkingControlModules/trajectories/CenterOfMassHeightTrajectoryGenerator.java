package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.robotSide.RobotSide;


public interface CenterOfMassHeightTrajectoryGenerator
{
   public abstract void initialize(RobotSide supportLeg, Footstep nextFootstep, List<PlaneContactState> contactStates);
   public void solve(CenterOfMassHeightPartialDerivativesData coMHeightPartialDerivativesDataToPack, CenterOfMassHeightInputData centerOfMassHeightInputData);
}
