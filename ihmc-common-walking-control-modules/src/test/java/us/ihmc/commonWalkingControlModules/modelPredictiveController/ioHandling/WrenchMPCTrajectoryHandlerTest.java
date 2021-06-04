package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.LinearMPCIndexHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.MPCTestHelper;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class WrenchMPCTrajectoryHandlerTest
{
   @Test
   public void testExtractFromVariables()
   {
      double gravityZ = -9.81;
      double nominalHeight = 1.0;
      double omega = 3.0;
      double mass = 2.7;

      double startTime = 0.0;
      double endTime = 1.5;

      YoRegistry testRegistry = new YoRegistry("test");
      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();
      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      List<MPCContactPlane> contactPlanes = new ArrayList<>();
      List<List<MPCContactPlane>> contactPlanesList = new ArrayList<>();

      FramePose3D dummyContactPose = new FramePose3D();
      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(startTime, endTime);
      contact.addContact(dummyContactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact);

      indexHandler.initialize(contactProviders);

      DMatrixRMaj coefficients = new DMatrixRMaj(indexHandler.getTotalProblemSize(), 1);

      LinearMPCTrajectoryHandler linearMPCTrajectoryHandler = new LinearMPCTrajectoryHandler(indexHandler, gravityZ, nominalHeight, testRegistry);
      WrenchMPCTrajectoryHandler wrenchMPCTrajectoryHandler = new WrenchMPCTrajectoryHandler(testRegistry);

      MPCContactPlane contactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());
      contactPlanes.add(contactPlane);
      contactPlanesList.add(contactPlanes);

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         FramePose3D contactPose = EuclidFrameRandomTools.nextFramePose3D(random, ReferenceFrame.getWorldFrame());
         contactPlane.computeBasisVectors(contactPolygon, contactPose, 0.8);
         coefficients.setData(RandomNumbers.nextDoubleArray(random, indexHandler.getTotalProblemSize(), 1));

         contactPlane.computeContactForceCoefficientMatrix(coefficients, 0);

         linearMPCTrajectoryHandler.solveForTrajectoryOutsidePreviewWindow(contactProviders);
         linearMPCTrajectoryHandler.extractSolutionForPreviewWindow(coefficients, contactProviders, contactPlanesList, contactProviders, omega);
         wrenchMPCTrajectoryHandler.extractSolutionForPreviewWindow(contactProviders, contactPlanesList, mass, omega);

         for (double time = startTime; time <= endTime; time += 1e-3)
         {
            linearMPCTrajectoryHandler.compute(time);
            wrenchMPCTrajectoryHandler.compute(time);

            FramePoint3DReadOnly comPosition = linearMPCTrajectoryHandler.getDesiredCoMPosition();
            contactPlane.computeContactForce(omega, time);
            FrameVector3D totalTorque = new FrameVector3D();
            for (int contactIdx = 0; contactIdx < contactPlane.getNumberOfContactPoints(); contactIdx++)
            {
               FrameVector3D momentArm = new FrameVector3D();
               FrameVector3D pointTorque = new FrameVector3D();
               MPCContactPoint contactPoint = contactPlane.getContactPointHelper(contactIdx);
               momentArm.sub(contactPoint.getBasisVectorOrigin(), comPosition);
               pointTorque.cross(momentArm, contactPoint.getContactAcceleration());
               pointTorque.scale(mass);

               totalTorque.add(pointTorque);
            }

            PoseReferenceFrame comFrame = new PoseReferenceFrame("comFrame", ReferenceFrame.getWorldFrame());
            comFrame.setPositionAndUpdate(comPosition);

            Wrench wrench = new Wrench(wrenchMPCTrajectoryHandler.getDesiredWrench());
            wrench.changeFrame(comFrame);

            EuclidCoreTestTools.assertVector3DGeometricallyEquals(totalTorque, wrench.getAngularPart(), 1e-5);
         }
      }
   }
}
