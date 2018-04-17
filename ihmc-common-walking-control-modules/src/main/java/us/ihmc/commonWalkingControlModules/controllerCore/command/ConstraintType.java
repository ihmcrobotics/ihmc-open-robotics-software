package us.ihmc.commonWalkingControlModules.controllerCore.command;

public enum ConstraintType
{
   /**
    * A equality condition that is set up in the QP as part of the objective rather than as a
    * constraint:<br>
    * min<sub>x</sub> || Ax - b ||, <br>
    * where 'b' is the desired task objective.
    */
   OBJECTIVE,
   /**
    * A equality condition that is set up in the QP as a hard constraint:<br>
    * Ax = b, <br>
    * where 'b' is the desired task objective.
    */
   EQUALITY,
   /**
    * A inequality condition that is set up in the QP as a hard lesser-or-equal inequality constraint:<br>
    * Ax <= b, <br>
    * where 'b' is the desired task objective.
    */
   LEQ_INEQUALITY,
   /**
    * A inequality condition that is set up in the QP as a hard greater-or-equal inequality constraint:<br>
    * Ax >= b, <br>
    * where 'b' is the desired task objective.
    */
   GEQ_INEQUALITY;

   @Override
   public String toString()
   {
      switch (this)
      {
      case OBJECTIVE:
         return "Objective.";
      case EQUALITY:
         return "Equality Motion Constraint.";
      case LEQ_INEQUALITY:
         return "Lesser-or-Equal Inequality Motion Constraint.";
      case GEQ_INEQUALITY:
         return "Greater-or-Equal Inequality Motion Constraint.";
      default:
         return "Unknown case";
      }
   }
}
