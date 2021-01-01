package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

public class MPCIndexHandlerTest
{
   @Test
   public void testIndices()
   {
      MPCIndexHandler indexHandler = new MPCIndexHandler(4);

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
      int totalSize = 6 + rhoSize + 18;

      assertEquals(totalSize, indexHandler.getTotalProblemSize());
      assertEquals(0, indexHandler.getComCoefficientStartIndex(0, 0));
      assertEquals(2, indexHandler.getComCoefficientStartIndex(0, 1));
      assertEquals(4, indexHandler.getComCoefficientStartIndex(0, 2));

      assertEquals(6, indexHandler.getRhoCoefficientStartIndex(0));

      assertEquals(6 + rhoSize, indexHandler.getOrientationCoefficientsStartIndex(0));
      assertEquals(6 + rhoSize, indexHandler.getYawCoefficientsStartIndex(0));
      assertEquals(12 + rhoSize, indexHandler.getPitchCoefficientsStartIndex(0));
      assertEquals(18 + rhoSize, indexHandler.getRollCoefficientsStartIndex(0));
   }

}
