package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;

public class PreviewWindowCalculatorTest
{
   @Test
   public void computeSimpleIdealContact()
   {
      YoRegistry testRegistry = new YoRegistry("test");
      PreviewWindowCalculator previewWindowCalculator = new PreviewWindowCalculator(testRegistry);

      double segmentDuration = ((YoDouble) testRegistry.findVariable("nominalSegmentDuration")).getDoubleValue();

      FramePoint3D startCoP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, 0.25, 0.3);
      FramePoint3D middleCoP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.2, 0.3, 0.32);
      FramePoint3D endCoP = new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.05, -0.1, -0.32);

      FramePose3D firstPose = new FramePose3D();
      firstPose.getPosition().set(startCoP);

      FrameConvexPolygon2D contactPolygon = new FrameConvexPolygon2D();
      contactPolygon.addVertex(0.1, -0.05);
      contactPolygon.addVertex(0.1, 0.05);
      contactPolygon.addVertex(-0.1, 0.05);
      contactPolygon.addVertex(-0.1, -0.05);
      contactPolygon.update();

      FramePose3D secondPose = new FramePose3D();
      secondPose.getPosition().set(endCoP);

      ContactPlaneProvider firstContact = new ContactPlaneProvider();
      firstContact.getTimeInterval().setInterval(0.0, 2.0 * segmentDuration);
      firstContact.setStartECMPPosition(startCoP);
      firstContact.setEndECMPPosition(middleCoP);
      firstContact.setLinearECMPVelocity();
      firstContact.addContact(firstPose, contactPolygon);

      ContactPlaneProvider secondContact = new ContactPlaneProvider();
      secondContact.getTimeInterval().setInterval(2.0 * segmentDuration, 3.0 * segmentDuration);
      secondContact.setStartECMPPosition(middleCoP);
      secondContact.setEndECMPPosition(endCoP);
      secondContact.setLinearECMPVelocity();
      secondContact.addContact(firstPose, contactPolygon);
      secondContact.addContact(secondPose, contactPolygon);

      List<ContactPlaneProvider> contacts = new ArrayList<>();
      contacts.add(firstContact);
      contacts.add(secondContact);

      previewWindowCalculator.compute(contacts, 0.0);

      assertEquals(3.0 * segmentDuration, previewWindowCalculator.getPreviewWindowDuration(), 1e-5);
      assertEquals(3, previewWindowCalculator.getPlanningWindow().size());

      List<PreviewWindowSegment> previewWindow = previewWindowCalculator.getPlanningWindow();

      for (int i = 0; i < 3; i++)
         assertEquals(1, previewWindow.get(i).getNumberOfContactPhasesInSegment());

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(firstContact.getECMPStartPosition(), previewWindow.get(0).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(firstContact.getECMPEndPosition(), previewWindow.get(1).getContactPhase(0).getECMPEndPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(secondContact.getECMPStartPosition(), previewWindow.get(2).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(secondContact.getECMPEndPosition(), previewWindow.get(2).getContactPhase(0).getECMPEndPosition(), 1e-4);
   }
}
