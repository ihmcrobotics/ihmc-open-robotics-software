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

public class LinearMPCIndexHandlerTest
{
   @Test
   public void testIndices()
   {
      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);

      List<PreviewWindowSegment> contactProviders = new ArrayList<>();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D contactPose = new FramePose3D();

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      PreviewWindowSegment segment = new PreviewWindowSegment();
      segment.addContact(contactPose, contactPolygon);
      segment.addContactPhaseInSegment(contact, 0.0, 1.0);

      contactProviders.add(segment);

      indexHandler.initialize(contactProviders);

      int rhoSize = 4 * 4 * 4;
      int totalSize = 6 + rhoSize;

      assertEquals(totalSize, indexHandler.getTotalProblemSize());
      assertEquals(0, indexHandler.getComCoefficientStartIndex(0, 0));
      assertEquals(2, indexHandler.getComCoefficientStartIndex(0, 1));
      assertEquals(4, indexHandler.getComCoefficientStartIndex(0, 2));

      assertEquals(6, indexHandler.getRhoCoefficientStartIndex(0));
   }

}
