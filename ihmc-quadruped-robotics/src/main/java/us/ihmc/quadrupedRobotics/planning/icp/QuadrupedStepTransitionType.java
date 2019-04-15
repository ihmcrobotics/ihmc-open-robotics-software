package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;

/**
 * Defines the contact even transition type. This is useful for converting {@link QuadrupedTimedStep} to {@link QuadrupedContactPhase}.
 */
enum QuadrupedStepTransitionType
{
   LIFT_OFF, TOUCH_DOWN
}
