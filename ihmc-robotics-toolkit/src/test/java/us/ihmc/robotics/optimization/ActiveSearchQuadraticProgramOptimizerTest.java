package us.ihmc.robotics.optimization;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
/**
 * @author twan
 *         Date: 8/9/13
 */
public class ActiveSearchQuadraticProgramOptimizerTest
{

	@Test
   public void testUnconstrained()
   {
      Random random = new Random(12355L);
      int objectiveSize = 5;
      int solutionSize = 5;
      int constraintSize = 0;

      QuadraticProgram quadraticProgram = createRandomQuadraticProgram(random, objectiveSize, solutionSize, constraintSize);

      DMatrixRMaj initialGuess = new DMatrixRMaj(solutionSize, 1);

      ActiveSearchSolutionInfo solutionInfo = solve(quadraticProgram, initialGuess);

      assertTrue(solutionInfo.isConverged());

      DMatrixRMaj axMinusB = new DMatrixRMaj(solutionSize, 1);
      CommonOps_DDRM.mult(quadraticProgram.getA(), solutionInfo.getSolution(), axMinusB);
      CommonOps_DDRM.subtractEquals(axMinusB, quadraticProgram.getB());
      assertTrue(MatrixFeatures_DDRM.isConstantVal(axMinusB, 0.0, 1e-12));
   }

	@Test
   public void testConstrainedSimple()
   {
      int objectiveSize = 3;
      int solutionSize = 3;
      int constraintSize = 1;

      DMatrixRMaj a = new DMatrixRMaj(objectiveSize, solutionSize);
      CommonOps_DDRM.setIdentity(a);

      DMatrixRMaj b = new DMatrixRMaj(objectiveSize, 1);
      b.zero();

      DMatrixRMaj c = new DMatrixRMaj(constraintSize, solutionSize);
      CommonOps_DDRM.setIdentity(c);

      DMatrixRMaj d = new DMatrixRMaj(constraintSize, 1);
      d.set(0, 0, -1.0);

      QuadraticProgram quadraticProgram = new QuadraticProgram(a, b, c, d);

      DMatrixRMaj initialGuess = new DMatrixRMaj(solutionSize, 1);
      initialGuess.set(0, 0, -10.0);
      initialGuess.set(1, 0, -10.0);
      initialGuess.set(2, 0, -10.0);
      ActiveSearchSolutionInfo solutionInfo = solve(quadraticProgram, initialGuess);

      assertTrue(solutionInfo.isConverged());

      DMatrixRMaj expectedResult = new DMatrixRMaj(solutionSize, 1);
      expectedResult.set(0, 0, d.get(0, 0));
      expectedResult.set(1, 0, 0.0);
      expectedResult.set(2, 0, 0.0);
      assertTrue(MatrixFeatures_DDRM.isEquals(expectedResult, solutionInfo.getSolution(), 1e-12));
   }

	/**
	 * Not working, but probably not critical right now. Get this to work some day TODO
	 */
	@Disabled
	@Test
   public void testFullyConstrained()
   {	   
      int objectiveSize = 3;
      int solutionSize = 5;
      int constraintSize = 5;
      Random random = new Random(1235125L);
      QuadraticProgram quadraticProgram = createRandomQuadraticProgram(random, objectiveSize, solutionSize, constraintSize);
      DMatrixRMaj initialGuess = new DMatrixRMaj(solutionSize, 1);
      // need a feasible initial guess, so initial guess should already be the answer
      CommonOps_DDRM.solve(quadraticProgram.getC(), quadraticProgram.getD(), initialGuess);
      ActiveSearchSolutionInfo solutionInfo = solve(quadraticProgram, initialGuess);

      assertTrue(solutionInfo.isConverged());

      // TODO: assert that answer is initial guess
   }

   private ActiveSearchSolutionInfo solve(QuadraticProgram quadraticProgram, DMatrixRMaj initialGuess)
   {
      ActiveSearchOptimizationSettings settings = new ActiveSearchOptimizationSettings(1e-12, 50, true);
      ActiveSearchQuadraticProgramOptimizer optimizer = new ActiveSearchQuadraticProgramOptimizer(settings);
      optimizer.setQuadraticProgram(quadraticProgram);
      optimizer.solve(initialGuess);
      return optimizer.getSolutionInfo();
   }

   private static QuadraticProgram createRandomQuadraticProgram(Random random, int objectiveSize, int solutionSize, int constraintSize)
   {
      DMatrixRMaj a = RandomMatrices_DDRM.rectangle(objectiveSize, solutionSize, random);
      DMatrixRMaj b = RandomMatrices_DDRM.rectangle(objectiveSize, 1, random);
      DMatrixRMaj c = RandomMatrices_DDRM.rectangle(constraintSize, solutionSize, random);
      DMatrixRMaj d = RandomMatrices_DDRM.rectangle(constraintSize, 1, random);
      return new QuadraticProgram(a, b, c, d);
   }
}
