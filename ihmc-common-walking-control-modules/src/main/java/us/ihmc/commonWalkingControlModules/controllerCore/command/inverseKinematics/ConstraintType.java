package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import us.ihmc.robotics.controllers.ControllerFailureException;

public enum ConstraintType
{
   /**
    * A equality condition that is set up in the QP as part of the objective rather than as a constraint
    */
   OBJECTIVE,
   /**
    * A equality condition that is set up in the QP as a hard constraint
    */
   EQUALITY,
   /**
    * A inequality condition that is set up in the QP as a hard inequality constraint
    */
   INEQUALITY;

   public boolean isHardConstraint()
   {
      switch (this)
      {
      case OBJECTIVE:
         return false;
      case EQUALITY:
      case INEQUALITY:
      default:
         return true;
      }
   }
   
   public String toString()
   {
      switch (this)
      {
      case OBJECTIVE: return "Objective.";
      case EQUALITY: return "Equality Motion Constraint.";
      case INEQUALITY: return "Inequality Motion Constraint.";
         default: return "Unknown case";
      }
   }

   public boolean isEqualityConstraint()
   {
      switch (this)
      {
      case OBJECTIVE: return true;
      case EQUALITY: return true;
      case INEQUALITY: return false;
      default: throw new RuntimeException("Invalid command constraint type");
      }
   }
};
