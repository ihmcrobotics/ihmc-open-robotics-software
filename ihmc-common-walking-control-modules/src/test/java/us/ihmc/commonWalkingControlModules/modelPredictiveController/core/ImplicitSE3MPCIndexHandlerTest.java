package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;

public class ImplicitSE3MPCIndexHandlerTest
{
   @Test
   public void testIndices()
   {
      ImplicitSE3MPCIndexHandler indexHandler = new ImplicitSE3MPCIndexHandler(4);

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D contactPose = new FramePose3D();

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

      contactProviders.add(contact0);
      contactProviders.add(contact1);
      contactProviders.add(contact2);

      indexHandler.initialize(contactProviders);

      int comCoefficients = contactPolygon.getNumberOfVertices() * LinearMPCIndexHandler.coefficientsPerRho * 4;
      int totalSize = 3 * (LinearMPCIndexHandler.comCoefficientsPerSegment + comCoefficients + ImplicitSE3MPCIndexHandler.variablesPerOrientationTick);


      assertEquals(totalSize, indexHandler.getTotalProblemSize());
      assertEquals(0, indexHandler.getComCoefficientStartIndex(0, 0));
      assertEquals(2, indexHandler.getComCoefficientStartIndex(0, 1));
      assertEquals(4, indexHandler.getComCoefficientStartIndex(0, 2));

      assertEquals(6, indexHandler.getRhoCoefficientStartIndex(0));
      assertEquals(LinearMPCIndexHandler.comCoefficientsPerSegment + comCoefficients, indexHandler.getOrientationStartIndex(0));
      assertEquals(LinearMPCIndexHandler.comCoefficientsPerSegment + comCoefficients + ImplicitSE3MPCIndexHandler.variablesPerOrientationTick, indexHandler.getComCoefficientStartIndex(1));
      assertEquals(2 * LinearMPCIndexHandler.comCoefficientsPerSegment + comCoefficients + ImplicitSE3MPCIndexHandler.variablesPerOrientationTick, indexHandler.getRhoCoefficientStartIndex(1));
      assertEquals(2 * (LinearMPCIndexHandler.comCoefficientsPerSegment + comCoefficients) + ImplicitSE3MPCIndexHandler.variablesPerOrientationTick, indexHandler.getOrientationStartIndex(1));
      assertEquals(2 * (LinearMPCIndexHandler.comCoefficientsPerSegment + comCoefficients + ImplicitSE3MPCIndexHandler.variablesPerOrientationTick), indexHandler.getComCoefficientStartIndex(2));
      assertEquals(3 * LinearMPCIndexHandler.comCoefficientsPerSegment + 2 * (comCoefficients + ImplicitSE3MPCIndexHandler.variablesPerOrientationTick), indexHandler.getRhoCoefficientStartIndex(2));
      assertEquals(3 * (LinearMPCIndexHandler.comCoefficientsPerSegment + comCoefficients) + 2 * ImplicitSE3MPCIndexHandler.variablesPerOrientationTick, indexHandler.getOrientationStartIndex(2));

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

}
