package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.PreviewWindowSegment;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;

public class SE3MPCIndexHandlerTest
{
   @Test
   public void testIndices()
   {
      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);

      List<PreviewWindowSegment> contactProviders = new ArrayList<>();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D contactPose = new FramePose3D();

      PreviewWindowSegment segment0 = new PreviewWindowSegment();
      PreviewWindowSegment segment1 = new PreviewWindowSegment();
      PreviewWindowSegment segment2 = new PreviewWindowSegment();

      ContactPlaneProvider contact0 = new ContactPlaneProvider();
      ContactPlaneProvider contact1 = new ContactPlaneProvider();
      ContactPlaneProvider contact2 = new ContactPlaneProvider();
      contact0.getTimeInterval().setInterval(0.0, 1.0);
      contact0.addContact(contactPose, contactPolygon);
      contact0.setStartECMPPosition(new FramePoint3D());
      contact0.setEndECMPPosition(new FramePoint3D());

      contact1.set(contact0);
      contact1.getTimeInterval().setInterval(1.0, 1.5);

      contact2.set(contact1);
      contact2.getTimeInterval().setInterval(1.5, 2.2);

      segment0.set(contact0);
      segment1.set(contact1);
      segment2.set(contact2);

      contactProviders.add(segment0);
      contactProviders.add(segment1);
      contactProviders.add(segment2);

      indexHandler.initialize(contactProviders);

      int comCoefficients = contactPolygon.getNumberOfVertices() * LinearMPCIndexHandler.coefficientsPerRho * 4;
      int totalSize = 3 * (LinearMPCIndexHandler.comCoefficientsPerSegment + comCoefficients + SE3MPCIndexHandler.variablesPerOrientationTick);
      totalSize += 6; // final orientation tick

      assertEquals(totalSize, indexHandler.getTotalProblemSize());
      assertEquals(0, indexHandler.getOrientationStartIndex(0));
      assertEquals(6, indexHandler.getComCoefficientStartIndex(0));
      assertEquals(12, indexHandler.getRhoCoefficientStartIndex(0));
      assertEquals(LinearMPCIndexHandler.comCoefficientsPerSegment + comCoefficients + SE3MPCIndexHandler.variablesPerOrientationTick, indexHandler.getOrientationStartIndex(1));
      assertEquals(LinearMPCIndexHandler.comCoefficientsPerSegment + comCoefficients + 2 * SE3MPCIndexHandler.variablesPerOrientationTick, indexHandler.getComCoefficientStartIndex(1));
      assertEquals(2 * (LinearMPCIndexHandler.comCoefficientsPerSegment + SE3MPCIndexHandler.variablesPerOrientationTick) + comCoefficients , indexHandler.getRhoCoefficientStartIndex(1));
      assertEquals(2 * (LinearMPCIndexHandler.comCoefficientsPerSegment + comCoefficients + SE3MPCIndexHandler.variablesPerOrientationTick), indexHandler.getOrientationStartIndex(2));
      assertEquals(2 * (LinearMPCIndexHandler.comCoefficientsPerSegment + comCoefficients) + 3 * SE3MPCIndexHandler.variablesPerOrientationTick, indexHandler.getComCoefficientStartIndex(2));
      assertEquals(3 * (LinearMPCIndexHandler.comCoefficientsPerSegment + SE3MPCIndexHandler.variablesPerOrientationTick) + 2 * (comCoefficients), indexHandler.getRhoCoefficientStartIndex(2));

      double timeAtStartOfSecondSegment = 0.0;
      for (int tick = 0; tick < indexHandler.getTicksInSegment(0); tick++)
         timeAtStartOfSecondSegment += indexHandler.getTickDuration(0);

      assertEquals(1.0, timeAtStartOfSecondSegment, 1e-6);

      double timeAtStartOfThirdSegment = timeAtStartOfSecondSegment;
      for (int tick = 0; tick < indexHandler.getTicksInSegment(1); tick++)
         timeAtStartOfThirdSegment += indexHandler.getTickDuration(1);

      assertEquals(1.5, timeAtStartOfThirdSegment, 1e-6);

      double timeAtEndOfThirdSegment = timeAtStartOfThirdSegment;
      for (int tick = 0; tick < indexHandler.getTicksInSegment(2); tick++)
         timeAtEndOfThirdSegment += indexHandler.getTickDuration(2);

      assertEquals(2.2, timeAtEndOfThirdSegment, 1e-6);
   }

   @Test
   public void testIndicesManually()
   {
      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      indexHandler.setOrientationDt(0.25);

      List<PreviewWindowSegment> contactProviders = new ArrayList<>();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D contactPose = new FramePose3D();

      PreviewWindowSegment segment0 = new PreviewWindowSegment();
      PreviewWindowSegment segment1 = new PreviewWindowSegment();

      ContactPlaneProvider contact0 = new ContactPlaneProvider();
      ContactPlaneProvider contact1 = new ContactPlaneProvider();
      contact0.getTimeInterval().setInterval(0.0, 1.0);
      contact0.addContact(contactPose, contactPolygon);
      contact0.setStartECMPPosition(new FramePoint3D());
      contact0.setEndECMPPosition(new FramePoint3D());

      contact1.set(contact0);
      contact1.getTimeInterval().setInterval(1.0, 1.5);

      segment0.set(contact0);
      segment1.set(contact1);

      contactProviders.add(segment0);
      contactProviders.add(segment1);

      indexHandler.initialize(contactProviders);

      int comCoefficients = contactPolygon.getNumberOfVertices() * LinearMPCIndexHandler.coefficientsPerRho * 4;
      int totalSize = 2 * (LinearMPCIndexHandler.comCoefficientsPerSegment + comCoefficients + SE3MPCIndexHandler.variablesPerOrientationTick);
      totalSize += 6; // final orientation



      assertEquals(totalSize, indexHandler.getTotalProblemSize());
      assertEquals(4, indexHandler.getTicksInSegment(0));
      assertEquals(2, indexHandler.getTicksInSegment(1));
      assertEquals(0.25, indexHandler.getTickDuration(0), 1e-5);
      assertEquals(0.25, indexHandler.getTickDuration(1), 1e-5);


      double timeAtStartOfSecondSegment = 0.0;
      for (int tick = 0; tick < indexHandler.getTicksInSegment(0); tick++)
         timeAtStartOfSecondSegment += indexHandler.getTickDuration(0);

      assertEquals(1.0, timeAtStartOfSecondSegment, 1e-6);

      double timeAtStartOfThirdSegment = timeAtStartOfSecondSegment;
      for (int tick = 0; tick < indexHandler.getTicksInSegment(1); tick++)
         timeAtStartOfThirdSegment += indexHandler.getTickDuration(1);

      assertEquals(1.5, timeAtStartOfThirdSegment, 1e-6);
   }
}
