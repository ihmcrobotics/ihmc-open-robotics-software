package us.ihmc.robotics.optimization;

import static org.junit.Assert.assertEquals;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.robotics.testing.JUnitTools;

public class EqualityConstraintEnforcerTest
{
   //TODO: Add test where JQ doesn't equal zero after constraint due to damped least squares.

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimpleCaseOne()
   {
      int matrixSize = 1;
      DampedLeastSquaresSolver solver = new DampedLeastSquaresSolver(matrixSize);
      EqualityConstraintEnforcer enforcer = new EqualityConstraintEnforcer(solver);
      
      // Jx=p: x1 = 2.0;
      DenseMatrix64F j = new DenseMatrix64F(new double[][]{{1.0, 0.0}});
      DenseMatrix64F p = new DenseMatrix64F(new double[][]{{2.0}});
      enforcer.setConstraint(j, p);
      
      DenseMatrix64F checkJQEqualsZeroAfterSetConstraint = enforcer.checkJQEqualsZeroAfterSetConstraint();
      JUnitTools.assertMatrixEqualsZero(checkJQEqualsZeroAfterSetConstraint, 1e-7);
//      System.out.println("checkJQEqualsZeroAfterSetConstraint = " + checkJQEqualsZeroAfterSetConstraint);
      
      // Ax = b: x1 + x2 = 5.0;
      DenseMatrix64F a = new DenseMatrix64F(new double[][]{{1.0, 1.0}});
      DenseMatrix64F b = new DenseMatrix64F(new double[][]{{5.0}});
      enforcer.constrainEquation(a, b);
      
      // Result should be new set of equations: Ax = b: x2 = 3.0;
//      System.out.println("A = " + a);
//      System.out.println("B = " + b);
      
      // An answer to this is [99.0, 3.0]'
      DenseMatrix64F xBar = new DenseMatrix64F(new double[][]{{99.0}, {3.0}});
      DenseMatrix64F xSolution = enforcer.constrainResult(xBar);
      
      // Constrained solution is [2.0, 3.0]'
//      System.out.println(xSolution);
      
      assertEquals(xSolution.get(0), 2.0, 1e-7);
      assertEquals(xSolution.get(1), 3.0, 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimpleCaseTwo()
   {
      int matrixSize = 1;
      DampedLeastSquaresSolver solver = new DampedLeastSquaresSolver(matrixSize);
      EqualityConstraintEnforcer enforcer = new EqualityConstraintEnforcer(solver);
      
      // Jx=p: x1 + x2 + x3 = 9.0;
      DenseMatrix64F j = new DenseMatrix64F(new double[][]{{1.0, 1.0, 1.0}});
      DenseMatrix64F p = new DenseMatrix64F(new double[][]{{9.0}});
      enforcer.setConstraint(j, p);
      
      DenseMatrix64F checkJQEqualsZeroAfterSetConstraint = enforcer.checkJQEqualsZeroAfterSetConstraint();
      JUnitTools.assertMatrixEqualsZero(checkJQEqualsZeroAfterSetConstraint, 1e-7);
      
      // Ax = b: x1 = 2.0, x2 = 3.0;
      DenseMatrix64F a = new DenseMatrix64F(new double[][]{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}});
      DenseMatrix64F b = new DenseMatrix64F(new double[][]{{2.0}, {3.0}});
      enforcer.constrainEquation(a, b);
      
      // Result should be new set of equations: Ax = b: 2/3 x1 -1/3 x2 -1/3 x3 = -1; -1/3 x1 + 2/3 x2 - 1/3 x3 = 0
//      System.out.println(a);
//      System.out.println(b);
      
      // A solution to this is [1 2 3]'
      DenseMatrix64F xBar = new DenseMatrix64F(new double[][]{{1.0},{2.0},{3.0}});
      DenseMatrix64F xSolution = enforcer.constrainResult(xBar);
      
      // Constrained solution is [2.0, 3.0 4.0]'
//      System.out.println(xSolution);  
      assertEquals(xSolution.get(0), 2.0, 1e-7);
      assertEquals(xSolution.get(1), 3.0, 1e-7);
      assertEquals(xSolution.get(2), 4.0, 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimpleCaseThree()
   {
      int matrixSize = 1;
      DampedLeastSquaresSolver solver = new DampedLeastSquaresSolver(matrixSize);
      EqualityConstraintEnforcer enforcer = new EqualityConstraintEnforcer(solver);
      
      // Jx=p: Ix=[2.0, 3.0, 4.0]';
      DenseMatrix64F j = new DenseMatrix64F(new double[][]{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}});
      DenseMatrix64F p = new DenseMatrix64F(new double[][]{{2.0}, {3.0}, {4.0}});
      enforcer.setConstraint(j, p);
      
      DenseMatrix64F checkJQEqualsZeroAfterSetConstraint = enforcer.checkJQEqualsZeroAfterSetConstraint();
      JUnitTools.assertMatrixEqualsZero(checkJQEqualsZeroAfterSetConstraint, 1e-7);
      
      // Ax = b: x1 + x2 + x3 = 9.0;
      DenseMatrix64F a = new DenseMatrix64F(new double[][]{{1.0, 1.0, 1.0}});
      DenseMatrix64F b = new DenseMatrix64F(new double[][]{{9.0}});
      enforcer.constrainEquation(a, b);
      
      // Result should be new set of equations: Ax = b: 0.0 x1 + 0.0 x2 + 0.0 x3 = 0.0 
//      System.out.println(a);
//      System.out.println(b);
      
      // Anything is a solution to this. 
      DenseMatrix64F xBar = new DenseMatrix64F(new double[][]{{97.0},{-3.0},{65.0}});
      DenseMatrix64F xSolution = enforcer.constrainResult(xBar);
      
      // Constrained solution is [2.0, 3.0 4.0]'
//      System.out.println(xSolution);  
      
      assertEquals(xSolution.get(0), 2.0, 1e-7);
      assertEquals(xSolution.get(1), 3.0, 1e-7);
      assertEquals(xSolution.get(2), 4.0, 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimpleOverconstrainedCaseFour()
   {
      int matrixSize = 1;
      DampedLeastSquaresSolver solver = new DampedLeastSquaresSolver(matrixSize);
      EqualityConstraintEnforcer enforcer = new EqualityConstraintEnforcer(solver);
      
      // Jx=p: x1 + x2 = 1.0; x1 + x2 = 2.0;
      DenseMatrix64F j = new DenseMatrix64F(new double[][]{{1.0, 1.0}, {1.0, 1.0}});
      DenseMatrix64F p = new DenseMatrix64F(new double[][]{{1.0}, {2.0}});
      enforcer.setConstraint(j, p);
      
      DenseMatrix64F checkJQEqualsZeroAfterSetConstraint = enforcer.checkJQEqualsZeroAfterSetConstraint();
      JUnitTools.assertMatrixEqualsZero(checkJQEqualsZeroAfterSetConstraint, 1e-7);
      
      // Ax = b: x1 = 3.0;
      DenseMatrix64F a = new DenseMatrix64F(new double[][]{{1.0, 0.0}});
      DenseMatrix64F b = new DenseMatrix64F(new double[][]{{3.0}});
      enforcer.constrainEquation(a, b);
      
      // Result should be new set of equations: Ax = b: 0.5 x1 - 0.5 x2 = 2.25;
//      System.out.println(a);
//      System.out.println(b);
      
      // [4.5 0] + alpha * [1 1] is a solution to this. Let's try [5.5 1.0]'
      DenseMatrix64F xBar = new DenseMatrix64F(new double[][]{{5.5},{1.0}});
      DenseMatrix64F xSolution = enforcer.constrainResult(xBar);
      
      // Constrained solution is [2.0, 3.0 4.0]'
//      System.out.println(xSolution);  
      
      assertEquals(xSolution.get(0), 3.0, 1e-7);
      assertEquals(xSolution.get(1), -1.5, 1e-7);
   }

}
