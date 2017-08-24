package us.ihmc.quadrupedRobotics.optimization.modelPredictiveControl;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.util.PreallocatedList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;

public interface QuadrupedMpcOptimizationWithLaneChange
{
   void initialize();

   /**
    * Compute optimal step adjustment and centroidal moment pivot using model predictive control.
    * @param stepAdjustmentVector output step adjustment vector
    * @param cmpPositionSetpoint output center of pressure setpoint
    * @param queuedSteps queue of ongoing and upcoming steps
    * @param currentSolePosition current sole position for each quadrant
    * @param currentContactState current contact state for each quadrant
    * @param currentComPosition current center of mass position
    * @param currentComVelocity current center of mass velocity
    * @param currentTime current time in seconds
    */
   void compute(FrameVector3D stepAdjustmentVector, FramePoint3D cmpPositionSetpoint, PreallocatedList<QuadrupedTimedStep> queuedSteps,
         QuadrantDependentList<FramePoint3D> currentSolePosition, QuadrantDependentList<ContactState> currentContactState, FramePoint3D currentComPosition,
         FrameVector3D currentComVelocity, double currentTime, QuadrupedMpcOptimizationWithLaneChangeSettings settings);
}
