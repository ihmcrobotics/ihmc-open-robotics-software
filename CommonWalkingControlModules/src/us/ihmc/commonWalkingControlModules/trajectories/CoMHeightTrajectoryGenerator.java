package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.WalkOnTheEdgesManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.robotSide.RobotSide;


public interface CoMHeightTrajectoryGenerator
{
   public abstract void attachWalkOnToesManager(WalkOnTheEdgesManager walkOnTheEdgesManager);

   public abstract void initialize(TransferToAndNextFootstepsData transferToAndNextFootstepsData, RobotSide supportLeg, Footstep nextFootstep, List<PlaneContactState> contactStates);
   
   public void solve(CoMHeightPartialDerivativesData coMHeightPartialDerivativesDataToPack, ContactStatesAndUpcomingFootstepData centerOfMassHeightInputData);

   public abstract boolean hasBeenInitializedWithNextStep();
}
