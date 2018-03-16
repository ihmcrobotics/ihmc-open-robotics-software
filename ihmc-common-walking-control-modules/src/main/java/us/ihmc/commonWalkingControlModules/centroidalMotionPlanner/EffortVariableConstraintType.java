package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

/**
 * Defines the nature of the constraint that can be imposed on the force and force rate variables 
 * that are being optimized through the {@code CentroidalMotionPlanner}.
 * Typically the {@code OBJECTIVE} type should be used with min max constraints specified
 * @author Apoorv S
 */
public enum EffortVariableConstraintType
{
   /**
    * The desired value of the force / force rate variable is included as
    * part of the objective function. This option should be accompanied by a weight.
    * If no weight is specified then the variable is optimized through a regularization weight
    */
   OBJECTIVE,
   /**
    * The desired value of the force / force rate variable is included in the biases and is not part of the
    * optimization
    */
   EQUALITY;
}