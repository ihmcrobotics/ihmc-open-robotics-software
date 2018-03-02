package us.ihmc.commonWalkingControlModules.controlModules.flight;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class WholeBodyAngularVelocityRegulatorQPSolverTest
{
   private final String namePrefix = getClass().getSimpleName();
   private final double defaultControllerDT = 0.01;
   private final double defaultMaxInertia = 100.0;
   private final double defaultMinInertia = 1.0;
   private final double defaultMaxInertiaRate = 100.0;
   private final double defaultRegularizationWeight = 1e-3;
   private final Random random = new Random(1124125l);
   private final ReferenceFrame controlFrame = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("DummyCenterOfMassFrame",
                                                                                                              ReferenceFrame.getWorldFrame(),
                                                                                                              new RigidBodyTransform());

   private YoVariableRegistry registry;
   private FlightWholeBodyAngularVelocityRegulatorQPSolver qpSolver;

   private void setupTest()
   {
      setupTest(defaultMaxInertiaRate, defaultMaxInertia, defaultMinInertia, defaultRegularizationWeight, defaultControllerDT);
   }

   private void setupTest(double maxInertiaRate, double maxInertia, double minInertia, double regularizationWeight, double controllerDT)
   {
      registry = new YoVariableRegistry(namePrefix);
      qpSolver = new FlightWholeBodyAngularVelocityRegulatorQPSolver(controllerDT, registry);
      qpSolver.initialize(controlFrame);
      qpSolver.setMaxInertiaRateOfChange(maxInertiaRate);
      qpSolver.setMaxPrincipalInertia(maxInertia);
      qpSolver.setMinPrincipalInertia(minInertia);
      qpSolver.setRegularizationWeight(regularizationWeight);
   }

   // For now generating diagonal inertia tensors 
   private DenseMatrix64F generateValidInertiaTensor()
   {
      double[] principalComponents = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
      return CommonOps.diag(principalComponents);
   }

   @Test
   public void testSolverRuns()
   {
      setupTest();
      DenseMatrix64F I0 = CommonOps.diag((new double[] {20.0, 25.0, 29.0}));//generateValidInertiaTensor();
      DenseMatrix64F dI = new DenseMatrix64F(3, 3);
      FrameVector3D w0 = new FrameVector3D(controlFrame, 1.0, 2.0, 3.0);
      FrameVector3D wD = new FrameVector3D(controlFrame, 3.0, 0.0, 6.7);
      qpSolver.setCurrentCentroidalInertiaTensor(I0);
      qpSolver.setCurrentVelocityEstimate(w0);
      qpSolver.setVelocityCommand(wD);
      qpSolver.compute();
      qpSolver.getDesiredInertiaRateOfChange(dI);
      PrintTools.debug(dI.toString());
      assertRealNumber(dI);
      assertSymmertry(dI);
      assertNorm(dI, defaultMaxInertiaRate);
      CommonOps.scale(defaultControllerDT, dI);
      CommonOps.addEquals(I0, dI);
      assertPositiveDefinite(I0);
      assertOffDiagonalElements(I0);
      assertTriangleInequality(I0);
   }
   
   private void assertOffDiagonalElements(DenseMatrix64F matrix)
   {
      double Ixx = matrix.get(0, 0);
      double Iyy = matrix.get(1, 1);
      double Izz = matrix.get(2, 2);
      double Ixy = matrix.get(0, 1);
      double Iyz = matrix.get(1, 2);
      double Izx = matrix.get(2, 0);
      
      assertTrue(Ixx - 2 * Iyz >0);
      assertTrue(Iyy - 2 * Izx >0);
      assertTrue(Izz - 2 * Ixy >0);
   }

   private void assertTriangleInequality(DenseMatrix64F matrix)
   {
      double Ixx = matrix.get(0, 0);
      double Iyy = matrix.get(1, 1);
      double Izz = matrix.get(2, 2);
      
      assertTrue("Triangle inequality does not hold, Ixx: " + Ixx + ", Iyy: " + Iyy + ", Izz: " + Izz, Ixx <= Iyy + Izz);
      assertTrue("Triangle inequality does not hold, Ixx: " + Ixx + ", Iyy: " + Iyy + ", Izz: " + Izz, Iyy <= Ixx + Izz);
      assertTrue("Triangle inequality does not hold, Ixx: " + Ixx + ", Iyy: " + Iyy + ", Izz: " + Izz, Izz <= Iyy + Ixx);
   }

   private void assertPositiveDefinite(DenseMatrix64F matrix)
   {
      assertTrue("Diagonal element not positive", matrix.get(0, 0) > 0);
      assertTrue("First minor not positive", matrix.get(0, 0) * matrix.get(1, 1) - matrix.get(0, 1) * matrix.get(1, 0) > 0);
      assertTrue("Determinant not positive", CommonOps.det(matrix) > 0);
   }

   private void assertNorm(DenseMatrix64F matrix, double limit)
   {
      double val = 0.0;
      for (int i = 0; i < matrix.numRows; i++)
         for (int j = 0; j < matrix.numCols; j++)
            val += Math.abs(matrix.get(i, j));

      assertTrue("Norm constraint violated, Norm: " + val + ", Should be: " + limit,val <= limit);
   }

   private void assertSymmertry(DenseMatrix64F matrix)
   {
      for (int i = 1; i < matrix.numRows; i++)
         for (int j = 0; j < i; j++)
            assertTrue("Symmetry constraint violated, " + matrix.toString(), matrix.get(i, j) == matrix.get(j, i));
   }

   private void assertRealNumber(DenseMatrix64F matrix)
   {
      for (int i = 0; i < matrix.numRows; i++)
         for (int j = 0; j < matrix.numCols; j++)
            assertTrue("Inertia tensor contains non real numbers, " + matrix.toString(), Double.isFinite(matrix.get(i, j)));
   }

}
