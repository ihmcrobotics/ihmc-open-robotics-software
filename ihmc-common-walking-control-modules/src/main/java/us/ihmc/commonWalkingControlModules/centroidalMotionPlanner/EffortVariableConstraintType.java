package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

/**
 * Defines the nature of the constraint that can be imposed on the force and force rate variables 
 * that are being optimized through the {@code CentroidalMotionPlanner}.
 * Typically the {@code OBJECTIVE} type should be used with min max constraints specified
 * @author Apoorv S
 */
public enum EffortVariableConstraintType
{
   OBJECTIVE, CONSTRAINT;
}