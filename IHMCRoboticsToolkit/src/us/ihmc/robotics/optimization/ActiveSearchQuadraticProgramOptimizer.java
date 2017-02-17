package us.ihmc.robotics.optimization;

import java.util.Set;

import org.ejml.alg.dense.mult.VectorVectorMult;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.MathTools;

/**
 * @author twan
 *         Date: 8/9/13
 * based on Escande, Mansard, Wieber. Hierarchical Quadratic Programming - Part 1: Fundamental Bases. Algorithm 6.
 */
public class ActiveSearchQuadraticProgramOptimizer
{
   private QuadraticProgram quadraticProgram;
   private final ActiveSearchOptimizationSettings settings;
   private final ActiveSearchSolutionInfo solutionInfo = new ActiveSearchSolutionInfo();
   private final EqualityConstraintEnforcer equalityConstraintEnforcer;

   // temp stuff
   private final DenseMatrix64F cX = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F aCopy = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F bCopy = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F cActive = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F dActive = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F xBarStar = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F ci = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F step = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F axMinusB = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F lambda = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F aCPlus = new DenseMatrix64F(1, 1);

   public ActiveSearchQuadraticProgramOptimizer(ActiveSearchOptimizationSettings settings)
   {
      this.settings = settings;
      this.equalityConstraintEnforcer = new EqualityConstraintEnforcer(settings.getLinearSolver());
   }

   public void setQuadraticProgram(QuadraticProgram quadraticProgram)
   {
      this.quadraticProgram = quadraticProgram;
   }

   private void setInitialGuess(DenseMatrix64F initialGuess)
   {
      solutionInfo.setSolution(initialGuess);
      determineActiveSet(initialGuess);
   }

   private void determineActiveSet(DenseMatrix64F x)
   {
      solutionInfo.clearActiveSet();

      DenseMatrix64F c = quadraticProgram.getC();
      DenseMatrix64F d = quadraticProgram.getD();

      cX.reshape(quadraticProgram.getInequalityConstraintSize(), 1);
      CommonOps.mult(c, x, cX);

      for (int i = 0; i < cX.getNumRows(); i++)
      {
         double cXi = cX.get(i, 0);
         double di = d.get(i, 0);

         if (MathTools.epsilonEquals(cXi, di, settings.getEpsilonConstraintActive()))
            solutionInfo.getActiveSet().add(i);
         else if (cXi >= di)
            throw new RuntimeException("x is not feasible!");
      }
   }

   public void solve(DenseMatrix64F initialGuess)
   {
      if (quadraticProgram == null)
         throw new RuntimeException("Quadratic program has not been set!");

      solutionInfo.reset(quadraticProgram.getSolutionSize());

      setInitialGuess(initialGuess);

      while (!solutionInfo.isConverged() && (solutionInfo.getIterations() < settings.getMaxIterations()))
      {
         solutionInfo.setConverged(true);

         updateActiveConstraintEquation();
         DenseMatrix64F xStar = computeOptimumForCurrentActiveSet();

         // compute step length
         DenseMatrix64F x = solutionInfo.getSolution();
         step.set(xStar);
         CommonOps.subtractEquals(step, x); // step = xStar - x
         ci.reshape(1, quadraticProgram.getSolutionSize());

         double tau = 1.0;
         int newActiveConstraintIndex = -1;
         for (int constraintIndex = 0; constraintIndex < quadraticProgram.getInequalityConstraintSize(); constraintIndex++)
         {
            if (!solutionInfo.getActiveSet().contains(constraintIndex))
            {
               // c_i
               CommonOps.extract(quadraticProgram.getC(), constraintIndex, constraintIndex + 1, 0, quadraticProgram.getSolutionSize(), ci, 0, 0);

               // d_i
               double di = quadraticProgram.getD().get(constraintIndex, 0);

               // ci * x
               double ciX = VectorVectorMult.innerProd(ci, x);

               // ci * step
               double ciStep = VectorVectorMult.innerProd(ci, step);

               // tau_i
               double tauI = -(ciX - di) / ciStep;

               // update tau
               if (tauI < tau)
               {
                  tau = tauI;
                  newActiveConstraintIndex = constraintIndex;
               }
            }
         }

         updateSolution(tau);

         // TODO: does order matter?
         decreaseActiveSetIfNecessary();
         increaseActiveSetIfNecessary(newActiveConstraintIndex);

         solutionInfo.incrementIterations();
      }
   }

   private void updateActiveConstraintEquation()
   {
      Set<Integer> activeSet = solutionInfo.getActiveSet();
      int nActiveConstraints = activeSet.size();

      DenseMatrix64F c = quadraticProgram.getC();
      DenseMatrix64F d = quadraticProgram.getD();

      cActive.reshape(nActiveConstraints, quadraticProgram.getSolutionSize());
      dActive.reshape(nActiveConstraints, 1);

      int activeConstraintIndex = 0;
      for (Integer constraintIndex : activeSet)
      {
         CommonOps.extract(c, constraintIndex, constraintIndex + 1, 0, quadraticProgram.getSolutionSize(), cActive,
               activeConstraintIndex, 0);
         dActive.set(activeConstraintIndex, 0, d.get(constraintIndex, 0));

         activeConstraintIndex++;
      }
   }

   private DenseMatrix64F computeOptimumForCurrentActiveSet()
   {
      // compute the optimum for the current active set (treat all active constraints as equalities)
      aCopy.set(quadraticProgram.getA());
      bCopy.set(quadraticProgram.getB());

      equalityConstraintEnforcer.setConstraint(cActive, dActive);
      equalityConstraintEnforcer.constrainEquation(aCopy, bCopy);

      settings.getLinearSolver().setA(aCopy);
      xBarStar.reshape(quadraticProgram.getSolutionSize(), 1);
      settings.getLinearSolver().solve(bCopy, xBarStar);

      return equalityConstraintEnforcer.constrainResult(xBarStar);
   }

   private void increaseActiveSetIfNecessary(int newActiveConstraintIndex)
   {
      // if necessary, increase the active set
      if (newActiveConstraintIndex >= 0)
      {
         solutionInfo.getActiveSet().add(newActiveConstraintIndex);
         solutionInfo.setConverged(false);
      }
   }

   private void decreaseActiveSetIfNecessary()
   {
      // if necessary, decrease the active set
      // first compute lambda (solution to dual problem)
      axMinusB.reshape(quadraticProgram.getObjectiveSize(), 1);
      CommonOps.mult(quadraticProgram.getA(), solutionInfo.getSolution(), axMinusB);
      CommonOps.subtractEquals(axMinusB, quadraticProgram.getB());
      aCPlus.reshape(quadraticProgram.getObjectiveSize(), cActive.getNumRows());
      CommonOps.mult(quadraticProgram.getA(), equalityConstraintEnforcer.getConstraintPseudoInverse(), aCPlus);
      lambda.reshape(cActive.getNumRows(), 1);
      CommonOps.multTransA(aCPlus, axMinusB, lambda);
      CommonOps.changeSign(lambda);
      // TODO: do you need to update cActive after possible changes in the active set due to increaseActiveSetIfNecessary?

      // compute nu
      double nu = Double.POSITIVE_INFINITY;
      Integer superfluousActiveConstraintIndex = null;
      int activeConstraintIndex = 0;
      for (Integer constraintIndex : solutionInfo.getActiveSet())
      {
         double lambdaI = lambda.get(activeConstraintIndex++, 0);
         if (lambdaI < nu)
         {
            nu = lambdaI;
            superfluousActiveConstraintIndex = constraintIndex;
         }
      }
      if (nu < 0.0)
      {
         solutionInfo.getActiveSet().remove(superfluousActiveConstraintIndex);
         solutionInfo.setConverged(false);
      }
   }

   private void updateSolution(double tau)
   {
      // update solution
      CommonOps.scale(tau, step);
      CommonOps.addEquals(solutionInfo.getSolution(), step);
   }

   public ActiveSearchSolutionInfo getSolutionInfo()
   {
      return solutionInfo;
   }
}
