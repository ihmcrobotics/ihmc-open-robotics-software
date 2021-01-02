package us.ihmc.commonWalkingControlModules.modelPredictiveController.continuous;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCTestHelper;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;

public class ContinuousMPCIndexHandlerTest
{
   @Test
   public void testIndices()
   {
      ContinuousMPCIndexHandler indexHandler = new ContinuousMPCIndexHandler(4);

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D contactPose = new FramePose3D();

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartCopPosition(new FramePoint3D());
      contact.setEndCopPosition(new FramePoint3D());

      contactProviders.add(contact);

      indexHandler.initialize(contactProviders);

      int rhoSize = 4 * 4 * 4;
      int totalSize = 6 + rhoSize + 3 * ContinuousMPCIndexHandler.orientationCoefficientsPerSegment;

      assertEquals(totalSize, indexHandler.getTotalProblemSize());
      assertEquals(0, indexHandler.getComCoefficientStartIndex(0, 0));
      assertEquals(2, indexHandler.getComCoefficientStartIndex(0, 1));
      assertEquals(4, indexHandler.getComCoefficientStartIndex(0, 2));

      assertEquals(6, indexHandler.getRhoCoefficientStartIndex(0));

      int orientationSegmentSize = ContinuousMPCIndexHandler.orientationCoefficientsPerSegment;
      assertEquals(6 + rhoSize, indexHandler.getOrientationCoefficientsStartIndex(0));
      assertEquals(6 + rhoSize, indexHandler.getYawCoefficientsStartIndex(0));
      assertEquals(6 + orientationSegmentSize + rhoSize, indexHandler.getPitchCoefficientsStartIndex(0));
      assertEquals(6 + 2 * orientationSegmentSize + rhoSize, indexHandler.getRollCoefficientsStartIndex(0));
   }

}
