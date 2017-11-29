package us.ihmc.commonWalkingControlModules.controllerCore.command;

public enum ConstraintType
{
   /**
    * A equality condition that is set up in the QP as part of the objective rather than as a
    * constraint (Ax - b = c)
    */
   OBJECTIVE,
   /**
    * A equality condition that is set up in the QP as a hard constraint (Ax - b = 0)
    */
   EQUALITY,
   /**
    * A inequality condition that is set up in the QP as a hard less than inequality constraint (Ax
    * - b < 0)
    */
   INEQUALITY;

   @Override
   public String toString()
   {
      switch (this)
      {
      case OBJECTIVE:
         return "Objective.";
      case EQUALITY:
         return "Equality Motion Constraint.";
      case INEQUALITY:
         return "Inequality Motion Constraint.";
      default:
         return "Unknown case";
      }
   }
}
