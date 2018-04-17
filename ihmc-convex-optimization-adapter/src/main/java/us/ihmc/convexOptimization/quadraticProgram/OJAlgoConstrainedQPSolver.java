package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;
import org.ojalgo.matrix.store.PrimitiveDenseStore;
import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.convex.ConvexSolver;

import us.ihmc.tools.exceptions.NoConvergenceException;

public class OJAlgoConstrainedQPSolver extends ConstrainedQPSolver
{

   public OJAlgoConstrainedQPSolver()
   {

      ExpressionsBasedModel foo;
      throw new RuntimeException("In Development... Coming soon...");
   }

   @Override
   public int solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq, DenseMatrix64F beq, DenseMatrix64F Ain, DenseMatrix64F bin, DenseMatrix64F x,
         boolean initialize) throws NoConvergenceException
   {
      PrimitiveDenseStore QDenseStore = PrimitiveDenseStore.FACTORY.columns(Q.data);
      PrimitiveDenseStore CDenseStore = PrimitiveDenseStore.FACTORY.columns(f.data);

      ConvexSolver.Builder builder = new ConvexSolver.Builder(QDenseStore, CDenseStore);

      return 2;
   }

   @Override
   public int solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq, DenseMatrix64F beq, DenseMatrix64F Ain, DenseMatrix64F bin, DenseMatrix64F lb,
         DenseMatrix64F ub, DenseMatrix64F x, boolean initialize) throws NoConvergenceException
   {
      return 0;
   }

   @Override
   public boolean supportBoxConstraints()
   {
      return false;
   }

}
