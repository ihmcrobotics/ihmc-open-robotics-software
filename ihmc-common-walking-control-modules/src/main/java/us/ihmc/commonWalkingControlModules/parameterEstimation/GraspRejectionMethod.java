package us.ihmc.commonWalkingControlModules.parameterEstimation;

/**
 * How the internal (squeeze) component of a two-nub force-closure grasp is obtained before it is subtracted from
 * the inertial estimator's measurement. See {@link GraspInternalWrenchCalculator} and {@link InertialParameterManager}.
 */
public enum GraspRejectionMethod
{
   /** Stage A: the squeeze magnitude is prescribed (e.g. the testbed's commanded lambda). Lowest risk. */
   KNOWN,
   /** Stage B: the endpoint force is backed out of the arm joint torques via the arm Jacobian. Hardware-realistic. */
   JACOBIAN
}
