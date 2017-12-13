package us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput;

import org.ejml.data.DenseMatrix64F;

/**
 * Representation of an equality constraint for the ICP Optimization solver. Has the form<br>
 *    A<sub>eq</sub> x = b<sub>eq</sub>
 */
public class ICPEqualityConstraintInput
{
   /** linear matrix that multiplies the free variables in the equality constraint. */
   public DenseMatrix64F Aeq;
   /** vector that stores the desired value of the equality constraint. */
   public DenseMatrix64F beq;

   public ICPEqualityConstraintInput(int maximumNumberOfFreeVariables)
   {
      Aeq = new DenseMatrix64F(2, maximumNumberOfFreeVariables);
      beq = new DenseMatrix64F(2, 1);
   }

   /**
    * Resets the constraint values to zero.
    */
   public void reset()
   {
      Aeq.zero();
      beq.zero();
   }

   /**
    * Reshapes the constraint problem size.
    */
   public void reshape(int size)
   {
      Aeq.reshape(2, size);
   }
}
