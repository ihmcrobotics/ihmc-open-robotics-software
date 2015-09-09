package us.ihmc.commonWalkingControlModules.kinematics;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.RandomMatrices;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class DampedLeastSquaresJacobianSolverTest
{
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
   @After
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

	@DeployableTestMethod(duration = 0.3)
	@Test(timeout = 30000)
   public void test()
   {
      Random random = new Random(176L);
      int matrixSize = 5;
      YoVariableRegistry parentRegistry = new YoVariableRegistry("test");
      DenseMatrix64F jacobianMatrix = RandomMatrices.createRandom(matrixSize, matrixSize, random);
      DenseMatrix64F taskVelocity = RandomMatrices.createRandom(matrixSize, 1, random);
      
      
      DampedLeastSquaresJacobianSolver solver = new DampedLeastSquaresJacobianSolver("testSolver", matrixSize, parentRegistry);
      solver.setAlpha(1.0);
      DenseMatrix64F jointVelocities = new DenseMatrix64F(matrixSize, 1);
      solver.setJacobian(jacobianMatrix);
      solver.solve(jointVelocities, taskVelocity);

      DenseMatrix64F taskVelocityBack = new DenseMatrix64F(matrixSize, 1);
      solver.inverseSolve(taskVelocityBack, jointVelocities);
      
      double delta = 1e-9;
      JUnitTools.assertMatrixEquals(taskVelocity, taskVelocityBack, delta);
   }

}
