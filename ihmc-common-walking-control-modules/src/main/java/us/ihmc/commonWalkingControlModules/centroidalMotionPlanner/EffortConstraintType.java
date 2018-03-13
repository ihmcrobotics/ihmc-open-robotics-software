package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

/**
 * Defines the nature of the constraint to be imposed on the motion QP solution
 * Typically the {@code OBJECTIVE} type should be used with min max constraints specified
 * @author Apoorv S
 */
public enum EffortConstraintType
{
   OBJECTIVE, EQUALITY;
}