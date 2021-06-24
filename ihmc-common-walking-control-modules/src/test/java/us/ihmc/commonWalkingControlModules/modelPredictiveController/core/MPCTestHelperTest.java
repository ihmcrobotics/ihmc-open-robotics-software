package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.matrixlib.MatrixTestTools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class MPCTestHelperTest
{
   private static final double gravityZ = -9.81;
   private static final double omega = 3.0;
   private static final double mu = 0.8;
   private static final double mass = 10.0;

   private static final double Ixx = 1.0;
   private static final double Iyy = 1.0;
   private static final double Izz = 1.0;

   private static final Matrix3D momentOfInertia = new Matrix3D();

   @Test
   public void testGetContactPointAccelerationJacobian()
   {
      double orientationPreviewWindowLength = 0.75;

      MPCContactPlane contactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();
      FramePose3D contactPose = new FramePose3D();

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact);



      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);
      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      indexHandler.initialize(contactProviders);

      Random random = new Random(1738L);
      for (int iter = 0; iter < 1000; iter++)
      {
         double time = RandomNumbers.nextDouble(random, 0.0, 1.0);
         int numberOfTrajectoryCoefficients = indexHandler.getRhoCoefficientsInSegment(0) + LinearMPCIndexHandler.comCoefficientsPerSegment;
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));

         contactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0) - SE3MPCIndexHandler.variablesPerOrientationTick);
         contactPlane.computeContactForce(omega, time);

         DMatrixRMaj expectedForce = new DMatrixRMaj(3 * contactPlane.getNumberOfContactPoints(), 1);
         for (int i = 0; i < contactPlane.getNumberOfContactPoints(); i++)
         {
            contactPlane.getContactPointHelper(i).getContactAcceleration().get(3 * i, expectedForce);
         }

         DMatrixRMaj constructedForce = new DMatrixRMaj(3 * contactPlane.getNumberOfContactPoints(), 1);
         CommonOps_DDRM.mult(MPCTestHelper.getContactPointAccelerationJacobian(time, omega, contactPlane), trajectoryCoefficients, constructedForce);

         MatrixTestTools.assertMatrixEquals("Failed on iter " + iter, expectedForce, constructedForce, 1e-6);


      }
   }
}
