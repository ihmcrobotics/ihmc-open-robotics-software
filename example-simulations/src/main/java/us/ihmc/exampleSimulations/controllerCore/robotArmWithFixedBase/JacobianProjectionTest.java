package us.ihmc.exampleSimulations.controllerCore.robotArmWithFixedBase;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.linearAlgebra.DampedNullspaceCalculator;
import us.ihmc.robotics.linearAlgebra.DampedQRNullspaceCalculator;
import us.ihmc.robotics.linearAlgebra.SVDNullspaceCalculator;
import us.ihmc.robotics.screwTheory.GeometricJacobian;

import java.util.Random;

public class JacobianProjectionTest
{
   private static void testJacobianProjection()
   {
      FixedBaseRobotArm robotArm = new FixedBaseRobotArm(1e-3);

      // Set random config
      robotArm.setRandomConfiguration();
      robotArm.getElevator().updateFramesRecursively();

      // Compute hand jacobian
      GeometricJacobian geometricJacobian = new GeometricJacobian(robotArm.getElevator(), robotArm.getHand(), robotArm.getHandControlFrame());
      geometricJacobian.compute();
      DMatrixRMaj jacobian = geometricJacobian.getJacobianMatrix();
      DMatrixRMaj jacobianNullspaceProjector = new DMatrixRMaj(0);
      DMatrixRMaj qdRandom = new DMatrixRMaj(7, 1);

      Random random = new Random(3902);
      for (int i = 0; i < qdRandom.getNumRows(); i++)
      {
         qdRandom.set(i, 0, EuclidCoreRandomTools.nextDouble(random, 1.0));
      }

      DMatrixRMaj qdRandomTranspose = new DMatrixRMaj(1, 7);
      CommonOps_DDRM.transpose(qdRandom, qdRandomTranspose);

      LogTools.info("Original spatial velocity:");
      DMatrixRMaj spatialVelocity = new DMatrixRMaj(6, 1);
      CommonOps_DDRM.mult(jacobian, qdRandom, spatialVelocity);
      System.out.println(spatialVelocity);

      // Pick random joint velocity, project into nullspace of hand jacobian
      double nullspaceProjectionAlpha = 0.001;
      DampedQRNullspaceCalculator nullspaceCalculator = new DampedQRNullspaceCalculator(100, nullspaceProjectionAlpha);
      nullspaceCalculator.projectOntoNullspace(qdRandomTranspose, jacobian);
      nullspaceCalculator.computeNullspaceProjector(jacobian, jacobianNullspaceProjector);

      // Check J qd is close to zero
      DMatrixRMaj qdRandomProjected = new DMatrixRMaj(7, 1);
      CommonOps_DDRM.transpose(qdRandomTranspose, qdRandomProjected);

      DMatrixRMaj spatialVelocityProjected = new DMatrixRMaj(6, 1);
      CommonOps_DDRM.mult(jacobian, qdRandomProjected, spatialVelocityProjected);

      LogTools.info("spatial velocity projected:");
      System.out.println(spatialVelocityProjected);
   }

   private static void testSimpleNullspaceProjection()
   {
      DMatrixRMaj J = new DMatrixRMaj(2, 4);
      J.set(0, 0, 1.0);
      J.set(1, 1, 1.0);
      J.set(0, 2, Math.sqrt(0.5));
      J.set(1, 2, Math.sqrt(0.5));
      J.set(0, 3, -Math.sqrt(0.3));
      J.set(1, 3, Math.sqrt(0.5));

      SVDNullspaceCalculator calculator = new SVDNullspaceCalculator(10, false);
      calculator.setMatrix(J, 2);
      DMatrixRMaj nullspace = calculator.getNullspace();

      DMatrixRMaj v1 = new DMatrixRMaj(0);
      DMatrixRMaj v2 = new DMatrixRMaj(0);

      System.out.println(nullspace);
   }

   public static void main(String[] args)
   {
//      testJacobianProjection();
      testSimpleNullspaceProjection();
   }
}
