package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

/**
 * Defines the type of constraints that can be imposed on the dependent variables of the 
 * {@code CentroidalMotionPlanner}. These variables are typically position, velocity, centroidal
 * torque, angular velocity and orientation
 * @author Apoorv S
 *
 */
public enum DependentVariableConstraintType
{
   /**
    * The desired value of the dependent variable is included as 
    * part of the objective function. This option should be accompanied by a weight.
    * An objective with no weight defaults to {@code DependentVariableConstraintType.IGNORE}
    */
   OBJECTIVE,
   /**
    * The desired value of the dependent variable is set a hard constraint that must be satisfied
    */
   CONSTRAINT,
   /**
    * The dependent variable is ignored during the motion planning optimization. Constraints on the 
    * variable's range are still imposed
    */
   IGNORE
}
