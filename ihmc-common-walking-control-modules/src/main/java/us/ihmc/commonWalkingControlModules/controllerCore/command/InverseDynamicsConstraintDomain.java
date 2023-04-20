package us.ihmc.commonWalkingControlModules.controllerCore.command;

/**
 * Parameterizes which variable set a task describes.
 */
public enum InverseDynamicsConstraintDomain
{
   /**
    * The task is a function of the desired accelerations q_dd, such that the Jacobian J multiplies the accelerations: <br>
    * J q_dd
    */
   MOTION,

   /**
    * The task is a function of the rho, the generalized contact forces. The Jacobian J multiplies the rho vector: <br>
    * J rho
    */
   RHO,

   /**
    * The task is a function of the desired accelerations and rho, the generalized contact forces. The Jacobian J multiplies the combined acceleration and rho vector: <br>
    * J [q_dd^T rho^T]^T
    */
   MOTION_AND_RHO
}
