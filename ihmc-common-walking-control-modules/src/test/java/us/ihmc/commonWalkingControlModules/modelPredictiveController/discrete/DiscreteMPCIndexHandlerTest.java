package us.ihmc.commonWalkingControlModules.modelPredictiveController.discrete;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.LinearMPCIndexHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCTestHelper;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;

public class DiscreteMPCIndexHandlerTest
{
   @Test
   public void testComputeOneSegment()
   {
      double orientationDt = 0.05;
      DiscreteMPCIndexHandler indexHandler = new DiscreteMPCIndexHandler(4, orientationDt);
      List<ContactPlaneProvider> contacts = new ArrayList<>();
      ConvexPolygon2DReadOnly contact = MPCTestHelper.createDefaultContact();
      FramePose3D contactPose = new FramePose3D();
      ContactPlaneProvider contactProvider = new ContactPlaneProvider();
      contactProvider.getTimeInterval().setInterval(0.0, 0.12);
      contactProvider.addContact(contactPose, contact);

      contacts.add(contactProvider);

      indexHandler.initialize(contacts, 0.4);

      assertEquals(2, indexHandler.getTotalOrientationTicks());

      contactProvider.getTimeInterval().setInterval(0.0, 0.15);
      indexHandler.initialize(contacts, 0.4);
      assertEquals(3, indexHandler.getTotalOrientationTicks());

      contactProvider.getTimeInterval().setInterval(0.0, 0.33);
      indexHandler.initialize(contacts, 0.4);
      assertEquals(6, indexHandler.getTotalOrientationTicks());

      assertEquals(orientationDt, indexHandler.getOrientationDt());

      int linearSize = 6 + indexHandler.getRhoCoefficientsInSegment(0);
      assertEquals(linearSize + 6 * DiscreteMPCIndexHandler.orientationVariablesPerTick, indexHandler.getTotalProblemSize());

      assertEquals(linearSize, indexHandler.getOrientationStart(0));
      assertEquals(0, indexHandler.getOrientationTicksBeforeSegment(0));
      assertEquals(6, indexHandler.getOrientationTicksInSegment(0));
      for (int i = 0; i < 7; i++)
         assertEquals(0, indexHandler.getSegmentForTick(i));
      for (int i = 7; i < 10; i++)
         assertEquals(-1, indexHandler.getSegmentForTick(i));
   }

   @Test
   public void testComputeTwoSegments()
   {
      double orientationDt = 0.05;
      DiscreteMPCIndexHandler indexHandler = new DiscreteMPCIndexHandler(4, orientationDt);
      List<ContactPlaneProvider> contacts = new ArrayList<>();
      ConvexPolygon2DReadOnly contact = MPCTestHelper.createDefaultContact();
      FramePose3D contactPose = new FramePose3D();
      ContactPlaneProvider contactProvider1 = new ContactPlaneProvider();
      ContactPlaneProvider contactProvider2 = new ContactPlaneProvider();
      contactProvider1.getTimeInterval().setInterval(0.0, 0.12);
      contactProvider2.getTimeInterval().setInterval(0.12, 0.23);
      contactProvider1.addContact(contactPose, contact);
      contactProvider2.addContact(contactPose, contact);

      contacts.add(contactProvider1);
      contacts.add(contactProvider2);

      indexHandler.initialize(contacts, 0.4);

      assertEquals(4, indexHandler.getTotalOrientationTicks());

      contactProvider1.getTimeInterval().setInterval(0.0, 0.15);
      contactProvider2.getTimeInterval().setInterval(0.15, 0.3);
      indexHandler.initialize(contacts, 0.6);
      assertEquals(6, indexHandler.getTotalOrientationTicks());

      contactProvider1.getTimeInterval().setInterval(0.0, 0.33);
      contactProvider2.getTimeInterval().setInterval(0.33, 0.43);
      indexHandler.initialize(contacts, 0.6);
      assertEquals(8, indexHandler.getTotalOrientationTicks());
      assertEquals(8, indexHandler.getOrientationTicksInSegment(0) + indexHandler.getOrientationTicksInSegment(1));

      assertEquals(orientationDt, indexHandler.getOrientationDt());

      int linearSize = 2 * LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(0) + indexHandler.getRhoCoefficientsInSegment(1);
      assertEquals(linearSize + 8 * DiscreteMPCIndexHandler.orientationVariablesPerTick, indexHandler.getTotalProblemSize());

      assertEquals(linearSize, indexHandler.getOrientationStart(0));
      assertEquals(0, indexHandler.getOrientationTicksBeforeSegment(0));
      assertEquals(6, indexHandler.getOrientationTicksInSegment(0));
      assertEquals(6, indexHandler.getOrientationTicksBeforeSegment(1));
      assertEquals(2, indexHandler.getOrientationTicksInSegment(1));

      for (int i = 0; i < 7; i++)
      {
         assertEquals("segment " + i, 0, indexHandler.getSegmentForTick(i));
      }
      for (int i = 7; i < 9; i++)
      {
         assertEquals(1, indexHandler.getSegmentForTick(i));
      }

      contactProvider1.getTimeInterval().setInterval(0.0, 0.33);
      contactProvider2.getTimeInterval().setInterval(0.33, 0.35);
      indexHandler.initialize(contacts, 0.6);
      assertEquals(7, indexHandler.getTotalOrientationTicks());

      assertEquals(orientationDt, indexHandler.getOrientationDt());

      linearSize = 2 * LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(0) + indexHandler.getRhoCoefficientsInSegment(1);

      assertEquals(linearSize, indexHandler.getOrientationStart(0));
      assertEquals(0, indexHandler.getOrientationTicksBeforeSegment(0));
      assertEquals(6, indexHandler.getOrientationTicksInSegment(0));
      assertEquals(6, indexHandler.getOrientationTicksBeforeSegment(1));
      assertEquals(1, indexHandler.getOrientationTicksInSegment(1));

      assertEquals(linearSize + 7 * DiscreteMPCIndexHandler.orientationVariablesPerTick, indexHandler.getTotalProblemSize());



      for (int i = 0; i < 7; i++)
      {
         assertEquals(0, indexHandler.getSegmentForTick(i));
      }
      assertEquals(1, indexHandler.getOrientationTicksInSegment(1));


      contactProvider1.getTimeInterval().setInterval(0.0, 0.33);
      contactProvider2.getTimeInterval().setInterval(0.33, 0.35);
      indexHandler.initialize(contacts, 0.3);
      assertEquals(6, indexHandler.getTotalOrientationTicks());

      assertEquals(orientationDt, indexHandler.getOrientationDt());

      linearSize = 2 * LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(0) + indexHandler.getRhoCoefficientsInSegment(1);
      assertEquals(linearSize + 6 * DiscreteMPCIndexHandler.orientationVariablesPerTick, indexHandler.getTotalProblemSize());

      assertEquals(linearSize, indexHandler.getOrientationStart(0));
      assertEquals(0, indexHandler.getOrientationTicksBeforeSegment(0));
      assertEquals(6, indexHandler.getOrientationTicksInSegment(0));
      assertEquals(6, indexHandler.getOrientationTicksBeforeSegment(1));
      assertEquals(0, indexHandler.getOrientationTicksInSegment(1));

      for (int i = 0; i < 7; i++)
      {
         assertEquals(0, indexHandler.getSegmentForTick(i));
      }
   }
}
