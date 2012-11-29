package us.ihmc.commonWalkingControlModules.kinematics;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;

public interface SwingLegAnglesAtEndOfStepEstimator
{

   public abstract void getEstimatedJointAnglesAtEndOfStep(LegJointPositions legJointPositionsToPack, LegJointVelocities legJointVelocitiesToPack,
         RobotSide swingSide, FramePoint desiredPositionInPelvisFrame, FrameOrientation desiredOrientationInPelvisFrame, double desiredYawInPelvisFrame,
         double timeRemaining, boolean useBodyPositionEstimation);

}