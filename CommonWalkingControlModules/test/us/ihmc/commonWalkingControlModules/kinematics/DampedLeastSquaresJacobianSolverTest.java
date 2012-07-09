package us.ihmc.commonWalkingControlModules.kinematics;

import static org.junit.Assert.*;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.RandomMatrices;
import org.junit.Test;

import us.ihmc.utilities.test.JUnitTestSuiteConstructor;
import us.ihmc.utilities.test.JUnitTools;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class DampedLeastSquaresJacobianSolverTest
{

   @Test
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
      solver.solve(jointVelocities, jacobianMatrix, taskVelocity);

      DenseMatrix64F jointAccelerations = new DenseMatrix64F(matrixSize, 1);
      
      DenseMatrix64F taskVelocityBack = new DenseMatrix64F(matrixSize, 1);
      DenseMatrix64F taskAccelerationBack = new DenseMatrix64F(matrixSize, 1);
      solver.inverseSolve(taskVelocityBack, jacobianMatrix, jointVelocities);
      
      double delta = 1e-9;
      JUnitTools.assertMatrixEquals(taskVelocity, taskVelocityBack, delta);
   }

}
