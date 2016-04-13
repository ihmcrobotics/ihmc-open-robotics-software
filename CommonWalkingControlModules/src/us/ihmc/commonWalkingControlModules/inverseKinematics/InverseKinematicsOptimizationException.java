package us.ihmc.commonWalkingControlModules.inverseKinematics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsSolution;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class InverseKinematicsOptimizationException extends NoConvergenceException
{
   private static final long serialVersionUID = -6508543677850525150L;

   private final InverseKinematicsSolution solution;

   public InverseKinematicsOptimizationException(NoConvergenceException exception, InverseKinematicsSolution solution)
   {
      super(exception.getIter());
      this.solution = solution;
   }

   public InverseKinematicsSolution getSolution()
   {
      return solution;
   }
}
