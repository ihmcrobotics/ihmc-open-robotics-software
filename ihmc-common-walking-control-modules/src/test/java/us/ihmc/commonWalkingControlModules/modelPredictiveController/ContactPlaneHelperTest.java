package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;

public class ContactPlaneHelperTest
{
   @Test
   public void testJacobianCalculation()
   {
      CoefficientJacobianMatrixHelper coefficientHelper = new CoefficientJacobianMatrixHelper(4, 4);
      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      ContactPlaneHelper contactPlaneHelper = new ContactPlaneHelper(4, 4, new ZeroConeRotationCalculator());

      double mu = 0.8;
      double omega = 3.0;
      ConvexPolygon2DReadOnly contact = MPCTestHelper.createDefaultContact();
      FramePose3D pose = new FramePose3D();

      rhoHelper.computeMatrices(contact, pose, 1e-5, 1e-5, mu);
      contactPlaneHelper.computeBasisVectors(contact, pose, mu);

      for (double time = 0.0; time < 1.5; time += 0.001)
      {
         coefficientHelper.computeMatrices(time, omega);
         contactPlaneHelper.computeJacobians(time, omega);

         for (int i = 0; i < 4; i++)
         {
            DMatrixRMaj jacobianExpected = new DMatrixRMaj(3, coefficientHelper.getCoefficientSize());
            CommonOps_DDRM.mult(rhoHelper.getLinearJacobianInWorldFrame(), coefficientHelper.getJacobianMatrix(i), jacobianExpected);

            EjmlUnitTests.assertEquals(jacobianExpected, contactPlaneHelper.getJacobian(i), 1e-5);
         }
      }
   }

   @Test
   public void testPositionJacobianCalculation()
   {
      CoefficientJacobianMatrixHelper coefficientHelper = new CoefficientJacobianMatrixHelper(4, 4);
      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      ContactPlaneHelper contactPlaneHelper = new ContactPlaneHelper(4, 4, new ZeroConeRotationCalculator());

      double mu = 0.8;
      double omega = 3.0;
      ConvexPolygon2DReadOnly contact = MPCTestHelper.createDefaultContact();
      FramePose3D pose = new FramePose3D();

      rhoHelper.computeMatrices(contact, pose, 1e-5, 1e-5, mu);
      contactPlaneHelper.computeBasisVectors(contact, pose, mu);

      for (double time = 0.0; time < 1.5; time += 0.001)
      {
         coefficientHelper.computeMatrices(time, omega);
         contactPlaneHelper.computeJacobians(time, omega);

         DMatrixRMaj jacobianExpected = new DMatrixRMaj(3, coefficientHelper.getCoefficientSize());
         CommonOps_DDRM.mult(rhoHelper.getLinearJacobianInWorldFrame(), coefficientHelper.getPositionJacobianMatrix(), jacobianExpected);

         EjmlUnitTests.assertEquals(jacobianExpected, contactPlaneHelper.getPositionJacobian(), 1e-5);
      }
   }
}
