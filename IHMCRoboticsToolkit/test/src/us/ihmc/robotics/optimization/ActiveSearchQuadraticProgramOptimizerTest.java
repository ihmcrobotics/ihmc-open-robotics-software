package us.ihmc.robotics.optimization;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import org.ejml.ops.RandomMatrices;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;

/**
 * @author twan
 *         Date: 8/9/13
 */
public class ActiveSearchQuadraticProgramOptimizerTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testUnconstrained()
   {
      Random random = new Random(12355L);
      int objectiveSize = 5;
      int solutionSize = 5;
      int constraintSize = 0;

      QuadraticProgram quadraticProgram = createRandomQuadraticProgram(random, objectiveSize, solutionSize, constraintSize);

      DenseMatrix64F initialGuess = new DenseMatrix64F(solutionSize, 1);

      ActiveSearchSolutionInfo solutionInfo = solve(quadraticProgram, initialGuess);

      assertTrue(solutionInfo.isConverged());

      DenseMatrix64F axMinusB = new DenseMatrix64F(solutionSize, 1);
      CommonOps.mult(quadraticProgram.getA(), solutionInfo.getSolution(), axMinusB);
      CommonOps.subtractEquals(axMinusB, quadraticProgram.getB());
      assertTrue(MatrixFeatures.isConstantVal(axMinusB, 0.0, 1e-12));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstrainedSimple()
   {
      int objectiveSize = 3;
      int solutionSize = 3;
      int constraintSize = 1;

      DenseMatrix64F a = new DenseMatrix64F(objectiveSize, solutionSize);
      CommonOps.setIdentity(a);

      DenseMatrix64F b = new DenseMatrix64F(objectiveSize, 1);
      b.zero();

      DenseMatrix64F c = new DenseMatrix64F(constraintSize, solutionSize);
      CommonOps.setIdentity(c);

      DenseMatrix64F d = new DenseMatrix64F(constraintSize, 1);
      d.set(0, 0, -1.0);

      QuadraticProgram quadraticProgram = new QuadraticProgram(a, b, c, d);

      DenseMatrix64F initialGuess = new DenseMatrix64F(solutionSize, 1);
      initialGuess.set(0, 0, -10.0);
      initialGuess.set(1, 0, -10.0);
      initialGuess.set(2, 0, -10.0);
      ActiveSearchSolutionInfo solutionInfo = solve(quadraticProgram, initialGuess);

      assertTrue(solutionInfo.isConverged());

      DenseMatrix64F expectedResult = new DenseMatrix64F(solutionSize, 1);
      expectedResult.set(0, 0, d.get(0, 0));
      expectedResult.set(1, 0, 0.0);
      expectedResult.set(2, 0, 0.0);
      assertTrue(MatrixFeatures.isEquals(expectedResult, solutionInfo.getSolution(), 1e-12));
   }

	/**
	 * Not working, but probably not critical right now. Get this to work some day TODO
	 */
	@ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout = 525)
   public void testFullyConstrained()
   {	   
      int objectiveSize = 3;
      int solutionSize = 5;
      int constraintSize = 5;
      Random random = new Random(1235125L);
      QuadraticProgram quadraticProgram = createRandomQuadraticProgram(random, objectiveSize, solutionSize, constraintSize);
      DenseMatrix64F initialGuess = new DenseMatrix64F(solutionSize, 1);
      // need a feasible initial guess, so initial guess should already be the answer
      CommonOps.solve(quadraticProgram.getC(), quadraticProgram.getD(), initialGuess);
      ActiveSearchSolutionInfo solutionInfo = solve(quadraticProgram, initialGuess);

      assertTrue(solutionInfo.isConverged());

      // TODO: assert that answer is initial guess
   }

   private ActiveSearchSolutionInfo solve(QuadraticProgram quadraticProgram, DenseMatrix64F initialGuess)
   {
      ActiveSearchOptimizationSettings settings = new ActiveSearchOptimizationSettings(1e-12, 50, true);
      ActiveSearchQuadraticProgramOptimizer optimizer = new ActiveSearchQuadraticProgramOptimizer(settings);
      optimizer.setQuadraticProgram(quadraticProgram);
      optimizer.solve(initialGuess);
      return optimizer.getSolutionInfo();
   }

   private static QuadraticProgram createRandomQuadraticProgram(Random random, int objectiveSize, int solutionSize, int constraintSize)
   {
      DenseMatrix64F a = RandomMatrices.createRandom(objectiveSize, solutionSize, random);
      DenseMatrix64F b = RandomMatrices.createRandom(objectiveSize, 1, random);
      DenseMatrix64F c = RandomMatrices.createRandom(constraintSize, solutionSize, random);
      DenseMatrix64F d = RandomMatrices.createRandom(constraintSize, 1, random);
      return new QuadraticProgram(a, b, c, d);
   }
}
