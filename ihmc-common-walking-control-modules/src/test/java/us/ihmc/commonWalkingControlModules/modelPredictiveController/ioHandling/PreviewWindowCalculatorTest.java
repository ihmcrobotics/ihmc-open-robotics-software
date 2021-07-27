package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactState;
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

      assertEquals(0.0, previewWindow.get(0).getStartTime(), 1e-5);
      assertEquals(segmentDuration, previewWindow.get(0).getEndTime(), 1e-5);
      assertEquals(segmentDuration, previewWindow.get(1).getStartTime(), 1e-5);
      assertEquals(2.0 * segmentDuration, previewWindow.get(1).getEndTime(), 1e-5);
      assertEquals(2.0 * segmentDuration, previewWindow.get(2).getStartTime(), 1e-5);
      assertEquals(3.0 * segmentDuration, previewWindow.get(2).getEndTime(), 1e-5);

      FramePoint3D superMiddle = new FramePoint3D();
      superMiddle.interpolate(startCoP, middleCoP, 0.5);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(firstContact.getECMPStartPosition(), previewWindow.get(0).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(superMiddle, previewWindow.get(0).getContactPhase(0).getECMPEndPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(superMiddle, previewWindow.get(1).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(firstContact.getECMPEndPosition(), previewWindow.get(1).getContactPhase(0).getECMPEndPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(secondContact.getECMPStartPosition(), previewWindow.get(2).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(secondContact.getECMPEndPosition(), previewWindow.get(2).getContactPhase(0).getECMPEndPosition(), 1e-4);

      previewWindowCalculator.compute(contacts, 0.25 * segmentDuration);

      assertEquals(2.75 * segmentDuration, previewWindowCalculator.getPreviewWindowDuration(), 1e-5);
      assertEquals(3, previewWindowCalculator.getPlanningWindow().size());

      previewWindow = previewWindowCalculator.getPlanningWindow();

      for (int i = 0; i < 3; i++)
         assertEquals(1, previewWindow.get(i).getNumberOfContactPhasesInSegment());

      assertEquals(0.25 * segmentDuration, previewWindow.get(0).getStartTime(), 1e-5);
      assertEquals((0.25 + 0.375) * segmentDuration, previewWindow.get(0).getEndTime(), 1e-5);
      assertEquals((0.25 + 0.375) * segmentDuration, previewWindow.get(1).getStartTime(), 1e-5);
      assertEquals(2.0 * segmentDuration, previewWindow.get(1).getEndTime(), 1e-5);
      assertEquals(2.0 * segmentDuration, previewWindow.get(2).getStartTime(), 1e-5);
      assertEquals(3.0 * segmentDuration, previewWindow.get(2).getEndTime(), 1e-5);
   }

   @Test
   public void computeSimpleContactSlightlyShortFirstSegment()
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
      firstContact.getTimeInterval().setInterval(0.0, 1.8 * segmentDuration);
      firstContact.setStartECMPPosition(startCoP);
      firstContact.setEndECMPPosition(middleCoP);
      firstContact.setLinearECMPVelocity();
      firstContact.addContact(firstPose, contactPolygon);

      ContactPlaneProvider secondContact = new ContactPlaneProvider();
      secondContact.getTimeInterval().setInterval(1.8 * segmentDuration, 3.0 * segmentDuration);
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

      assertEquals(0.0, previewWindow.get(0).getStartTime(), 1e-5);
      assertEquals(segmentDuration, 0.9 * previewWindow.get(0).getEndTime(), 1e-5);
      assertEquals(segmentDuration, 0.9 * previewWindow.get(1).getStartTime(), 1e-5);
      assertEquals(1.8 * segmentDuration, previewWindow.get(1).getEndTime(), 1e-5);
      assertEquals(1.8 * segmentDuration, previewWindow.get(2).getStartTime(), 1e-5);
      assertEquals(3.0 * segmentDuration, previewWindow.get(2).getEndTime(), 1e-5);

      FramePoint3D superMiddle = new FramePoint3D();
      superMiddle.interpolate(startCoP, middleCoP, 0.5);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(firstContact.getECMPStartPosition(), previewWindow.get(0).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(superMiddle, previewWindow.get(0).getContactPhase(0).getECMPEndPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(superMiddle, previewWindow.get(1).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(firstContact.getECMPEndPosition(), previewWindow.get(1).getContactPhase(0).getECMPEndPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(secondContact.getECMPStartPosition(), previewWindow.get(2).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(secondContact.getECMPEndPosition(), previewWindow.get(2).getContactPhase(0).getECMPEndPosition(), 1e-4);
   }

   @Test
   public void computeSimpleContactSlightlyLongFirstSegment()
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
      firstContact.getTimeInterval().setInterval(0.0, 2.2 * segmentDuration);
      firstContact.setStartECMPPosition(startCoP);
      firstContact.setEndECMPPosition(middleCoP);
      firstContact.setLinearECMPVelocity();
      firstContact.addContact(firstPose, contactPolygon);

      ContactPlaneProvider secondContact = new ContactPlaneProvider();
      secondContact.getTimeInterval().setInterval(2.2 * segmentDuration, 3.1 * segmentDuration);
      secondContact.setStartECMPPosition(middleCoP);
      secondContact.setEndECMPPosition(endCoP);
      secondContact.setLinearECMPVelocity();
      secondContact.addContact(firstPose, contactPolygon);
      secondContact.addContact(secondPose, contactPolygon);

      List<ContactPlaneProvider> contacts = new ArrayList<>();
      contacts.add(firstContact);
      contacts.add(secondContact);

      previewWindowCalculator.compute(contacts, 0.0);

      assertEquals(3.1 * segmentDuration, previewWindowCalculator.getPreviewWindowDuration(), 1e-5);
      assertEquals(3, previewWindowCalculator.getPlanningWindow().size());

      List<PreviewWindowSegment> previewWindow = previewWindowCalculator.getPlanningWindow();

      for (int i = 0; i < 3; i++)
         assertEquals(1, previewWindow.get(i).getNumberOfContactPhasesInSegment());

      assertEquals(0.0, previewWindow.get(0).getStartTime(), 1e-5);
      assertEquals(segmentDuration, 1.1 * previewWindow.get(0).getEndTime(), 1e-5);
      assertEquals(segmentDuration, 1.1 * previewWindow.get(1).getStartTime(), 1e-5);
      assertEquals(2.2 * segmentDuration, previewWindow.get(1).getEndTime(), 1e-5);
      assertEquals(2.2 * segmentDuration, previewWindow.get(2).getStartTime(), 1e-5);
      assertEquals(3.1 * segmentDuration, previewWindow.get(2).getEndTime(), 1e-5);

      FramePoint3D superMiddle = new FramePoint3D();
      superMiddle.interpolate(startCoP, middleCoP, 0.5);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(firstContact.getECMPStartPosition(), previewWindow.get(0).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(superMiddle, previewWindow.get(0).getContactPhase(0).getECMPEndPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(superMiddle, previewWindow.get(1).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(firstContact.getECMPEndPosition(), previewWindow.get(1).getContactPhase(0).getECMPEndPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(secondContact.getECMPStartPosition(), previewWindow.get(2).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(secondContact.getECMPEndPosition(), previewWindow.get(2).getContactPhase(0).getECMPEndPosition(), 1e-4);
   }

   @Test
   public void computeOneContactOneFlight()
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
      firstContact.getTimeInterval().setInterval(0.0, 2.2 * segmentDuration);
      firstContact.setStartECMPPosition(startCoP);
      firstContact.setEndECMPPosition(middleCoP);
      firstContact.setLinearECMPVelocity();
      firstContact.addContact(firstPose, contactPolygon);

      ContactPlaneProvider secondContact = new ContactPlaneProvider();
      secondContact.getTimeInterval().setInterval(2.2 * segmentDuration, 2.45 * segmentDuration);
      secondContact.setContactState(ContactState.FLIGHT);

      List<ContactPlaneProvider> contacts = new ArrayList<>();
      contacts.add(firstContact);
      contacts.add(secondContact);

      previewWindowCalculator.compute(contacts, 0.0);

      assertEquals(2.45 * segmentDuration, previewWindowCalculator.getPreviewWindowDuration(), 1e-5);
      assertEquals(3, previewWindowCalculator.getPlanningWindow().size());

      List<PreviewWindowSegment> previewWindow = previewWindowCalculator.getPlanningWindow();

      for (int i = 0; i < 3; i++)
         assertEquals(1, previewWindow.get(i).getNumberOfContactPhasesInSegment());

      assertEquals(0.0, previewWindow.get(0).getStartTime(), 1e-5);
      assertEquals(segmentDuration, 1.1 * previewWindow.get(0).getEndTime(), 1e-5);
      assertEquals(segmentDuration, 1.1 * previewWindow.get(1).getStartTime(), 1e-5);
      assertEquals(2.2 * segmentDuration, previewWindow.get(1).getEndTime(), 1e-5);
      assertEquals(2.2 * segmentDuration, previewWindow.get(2).getStartTime(), 1e-5);
      assertEquals(2.45 * segmentDuration, previewWindow.get(2).getEndTime(), 1e-5);

      assertEquals(ContactState.FLIGHT, previewWindow.get(2).getContactState());

      FramePoint3D superMiddle = new FramePoint3D();
      superMiddle.interpolate(startCoP, middleCoP, 0.5);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(firstContact.getECMPStartPosition(), previewWindow.get(0).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(superMiddle, previewWindow.get(0).getContactPhase(0).getECMPEndPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(superMiddle, previewWindow.get(1).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(firstContact.getECMPEndPosition(), previewWindow.get(1).getContactPhase(0).getECMPEndPosition(), 1e-4);
   }

   @Test
   public void computeOneContactOneFlightThenOneContact()
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
      firstContact.getTimeInterval().setInterval(0.0, 2.2 * segmentDuration);
      firstContact.setStartECMPPosition(startCoP);
      firstContact.setEndECMPPosition(middleCoP);
      firstContact.setLinearECMPVelocity();
      firstContact.addContact(firstPose, contactPolygon);

      ContactPlaneProvider secondContact = new ContactPlaneProvider();
      secondContact.getTimeInterval().setInterval(2.2 * segmentDuration, 2.45 * segmentDuration);
      secondContact.setContactState(ContactState.FLIGHT);

      ContactPlaneProvider thirdContact = new ContactPlaneProvider();
      thirdContact.getTimeInterval().setInterval(2.45 * segmentDuration, 3.2 * segmentDuration);
      thirdContact.setStartECMPPosition(endCoP);
      thirdContact.setEndECMPPosition(endCoP);
      thirdContact.setLinearECMPVelocity();
      thirdContact.addContact(firstPose, contactPolygon);
      thirdContact.addContact(secondPose, contactPolygon);

      List<ContactPlaneProvider> contacts = new ArrayList<>();
      contacts.add(firstContact);
      contacts.add(secondContact);
      contacts.add(thirdContact);

      previewWindowCalculator.compute(contacts, 0.0);

      assertEquals(3.1 * segmentDuration, previewWindowCalculator.getPreviewWindowDuration(), 1e-5);
      assertEquals(4, previewWindowCalculator.getPlanningWindow().size());

      List<PreviewWindowSegment> previewWindow = previewWindowCalculator.getPlanningWindow();

      for (int i = 0; i < 3; i++)
         assertEquals(1, previewWindow.get(i).getNumberOfContactPhasesInSegment());

      assertEquals(0.0, previewWindow.get(0).getStartTime(), 1e-5);
      assertEquals(segmentDuration, 1.1 * previewWindow.get(0).getEndTime(), 1e-5);
      assertEquals(segmentDuration, 1.1 * previewWindow.get(1).getStartTime(), 1e-5);
      assertEquals(2.2 * segmentDuration, previewWindow.get(1).getEndTime(), 1e-5);
      assertEquals(2.2 * segmentDuration, previewWindow.get(2).getStartTime(), 1e-5);
      assertEquals(2.45 * segmentDuration, previewWindow.get(2).getEndTime(), 1e-5);
      assertEquals(2.45 * segmentDuration, previewWindow.get(3).getStartTime(), 1e-5);
      assertEquals(3.2 * segmentDuration, previewWindow.get(3).getEndTime(), 1e-5);

      assertEquals(ContactState.FLIGHT, previewWindow.get(2).getContactState());

      FramePoint3D superMiddle = new FramePoint3D();
      superMiddle.interpolate(startCoP, middleCoP, 0.5);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(firstContact.getECMPStartPosition(), previewWindow.get(0).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(superMiddle, previewWindow.get(0).getContactPhase(0).getECMPEndPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(superMiddle, previewWindow.get(1).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(firstContact.getECMPEndPosition(), previewWindow.get(1).getContactPhase(0).getECMPEndPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(thirdContact.getECMPStartPosition(), previewWindow.get(3).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(thirdContact.getECMPEndPosition(), previewWindow.get(3).getContactPhase(0).getECMPEndPosition(), 1e-4);
   }

   @Test
   public void computeTwoContactsInOnePhase()
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
      firstContact.addContact(secondPose, contactPolygon);

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

      assertEquals(0.0, previewWindow.get(0).getStartTime(), 1e-5);
      assertEquals(segmentDuration, previewWindow.get(0).getEndTime(), 1e-5);
      assertEquals(segmentDuration, previewWindow.get(1).getStartTime(), 1e-5);
      assertEquals(2.0 * segmentDuration, previewWindow.get(1).getEndTime(), 1e-5);
      assertEquals(2.0 * segmentDuration, previewWindow.get(2).getStartTime(), 1e-5);
      assertEquals(3.0 * segmentDuration, previewWindow.get(2).getEndTime(), 1e-5);

      FramePoint3D superMiddle = new FramePoint3D();
      superMiddle.interpolate(startCoP, middleCoP, 0.5);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(firstContact.getECMPStartPosition(), previewWindow.get(0).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(superMiddle, previewWindow.get(0).getContactPhase(0).getECMPEndPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(superMiddle, previewWindow.get(1).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(firstContact.getECMPEndPosition(), previewWindow.get(1).getContactPhase(0).getECMPEndPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(secondContact.getECMPStartPosition(), previewWindow.get(2).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(secondContact.getECMPEndPosition(), previewWindow.get(2).getContactPhase(0).getECMPEndPosition(), 1e-4);

      previewWindowCalculator.compute(contacts, 0.25 * segmentDuration);

      assertEquals(2.75 * segmentDuration, previewWindowCalculator.getPreviewWindowDuration(), 1e-5);
      assertEquals(3, previewWindowCalculator.getPlanningWindow().size());

      previewWindow = previewWindowCalculator.getPlanningWindow();

      assertEquals(1, previewWindow.get(0).getNumberOfContactPhasesInSegment());
      assertEquals(2, previewWindow.get(1).getNumberOfContactPhasesInSegment());
      assertEquals(1, previewWindow.get(2).getNumberOfContactPhasesInSegment());

      double expectedStart = 0.25 * segmentDuration;
      double expectedDuration = 2.75 / 3.0 * segmentDuration;
      assertEquals(expectedStart, previewWindow.get(0).getStartTime(), 1e-5);
      assertEquals(expectedStart + expectedDuration, previewWindow.get(0).getEndTime(), 1e-5);
      assertEquals(expectedStart + expectedDuration, previewWindow.get(1).getStartTime(), 1e-5);
      assertEquals(expectedStart + 2.0 * expectedDuration, previewWindow.get(1).getEndTime(), 1e-5);
      assertEquals(expectedStart + 2.0 * expectedDuration, previewWindow.get(2).getStartTime(), 1e-5);
      assertEquals(expectedStart + 3.0 * expectedDuration, previewWindow.get(2).getEndTime(), 1e-5);
   }
}
