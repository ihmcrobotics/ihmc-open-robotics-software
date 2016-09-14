package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;
import org.ojalgo.optimisation.ExpressionsBasedModel;

import us.ihmc.tools.exceptions.NoConvergenceException;

public class OJAlgoConstrainedQPSolver extends ConstrainedQPSolver
{
 
   public OJAlgoConstrainedQPSolver()
   {
      ExpressionsBasedModel foo;
   }
   
   @Override
   public int solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq, DenseMatrix64F beq, DenseMatrix64F Ain, DenseMatrix64F bin, DenseMatrix64F x,
         boolean initialize) throws NoConvergenceException
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public int solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq, DenseMatrix64F beq, DenseMatrix64F Ain, DenseMatrix64F bin, DenseMatrix64F lb,
         DenseMatrix64F ub, DenseMatrix64F x, boolean initialize) throws NoConvergenceException
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public boolean supportBoxConstraints()
   {
      // TODO Auto-generated method stub
      return false;
   }

}
