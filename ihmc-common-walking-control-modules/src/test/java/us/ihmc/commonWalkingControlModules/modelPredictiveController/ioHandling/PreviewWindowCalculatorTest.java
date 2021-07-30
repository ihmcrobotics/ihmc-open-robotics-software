package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactState;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;

public class PreviewWindowCalculatorTest
{
   private static void setupTest(YoRegistry registry)
   {
      setupTest(registry, 0.15, 0.75);
   }

   private static void setupTest(YoRegistry registry, double nominalDuration, double totalDuration)
   {
      ((YoDouble) registry.findVariable("nominalSegmentDuration")).set(nominalDuration);
      ((YoDouble) registry.findVariable("maximumPreviewWindowDuration")).set(totalDuration);
   }

   @Test
   public void computeSimpleIdealContact()
   {
      YoRegistry testRegistry = new YoRegistry("test");
      PreviewWindowCalculator previewWindowCalculator = new PreviewWindowCalculator(testRegistry);

      setupTest(testRegistry);

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
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(firstContact.getECMPStartVelocity(), previewWindow.get(0).getContactPhase(0).getECMPStartVelocity(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(superMiddle, previewWindow.get(0).getContactPhase(0).getECMPEndPosition(), 1e-4);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(firstContact.getECMPStartVelocity(), previewWindow.get(0).getContactPhase(0).getECMPEndVelocity(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(superMiddle, previewWindow.get(1).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(firstContact.getECMPStartVelocity(), previewWindow.get(1).getContactPhase(0).getECMPStartVelocity(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(firstContact.getECMPEndPosition(), previewWindow.get(1).getContactPhase(0).getECMPEndPosition(), 1e-4);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(firstContact.getECMPStartVelocity(), previewWindow.get(1).getContactPhase(0).getECMPEndVelocity(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(secondContact.getECMPStartPosition(), previewWindow.get(2).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(secondContact.getECMPStartVelocity(), previewWindow.get(2).getContactPhase(0).getECMPStartVelocity(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(secondContact.getECMPEndPosition(), previewWindow.get(2).getContactPhase(0).getECMPEndPosition(), 1e-4);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(secondContact.getECMPStartVelocity(), previewWindow.get(2).getContactPhase(0).getECMPEndVelocity(), 1e-4);

      previewWindowCalculator.compute(contacts, 0.25 * segmentDuration);

      assertEquals(2.75 * segmentDuration, previewWindowCalculator.getPreviewWindowDuration(), 1e-5);
      assertEquals(3, previewWindowCalculator.getPlanningWindow().size());

      previewWindow = previewWindowCalculator.getPlanningWindow();

      for (int i = 0; i < 3; i++)
         assertEquals(1, previewWindow.get(i).getNumberOfContactPhasesInSegment());

      double firstGroupStart = 0.25 * segmentDuration;
      double firstGroupDuration = 2.0 * segmentDuration - firstGroupStart;
      assertEquals(firstGroupStart, previewWindow.get(0).getStartTime(), 1e-5);
      assertEquals(firstGroupStart + 0.5 * firstGroupDuration, previewWindow.get(0).getEndTime(), 1e-5);
      assertEquals(firstGroupStart + 0.5 * firstGroupDuration, previewWindow.get(1).getStartTime(), 1e-5);
      assertEquals(2.0 * segmentDuration, previewWindow.get(1).getEndTime(), 1e-5);
      assertEquals(2.0 * segmentDuration, previewWindow.get(2).getStartTime(), 1e-5);
      assertEquals(3.0 * segmentDuration, previewWindow.get(2).getEndTime(), 1e-5);
   }

   @Test
   public void computeSimpleContactSlightlyShortFirstSegment()
   {
      YoRegistry testRegistry = new YoRegistry("test");
      PreviewWindowCalculator previewWindowCalculator = new PreviewWindowCalculator(testRegistry);
      setupTest(testRegistry);

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
      assertEquals(0.9 *segmentDuration, previewWindow.get(0).getEndTime(), 1e-5);
      assertEquals(0.9 * segmentDuration, previewWindow.get(1).getStartTime(), 1e-5);
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
      setupTest(testRegistry);

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
      assertEquals(1.1 * segmentDuration, previewWindow.get(0).getEndTime(), 1e-5);
      assertEquals(1.1 * segmentDuration, previewWindow.get(1).getStartTime(), 1e-5);
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
      setupTest(testRegistry);

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
      assertEquals(1.1 * segmentDuration, previewWindow.get(0).getEndTime(), 1e-5);
      assertEquals(1.1 * segmentDuration, previewWindow.get(1).getStartTime(), 1e-5);
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
      setupTest(testRegistry);

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

      assertEquals(3.2 * segmentDuration, previewWindowCalculator.getPreviewWindowDuration(), 1e-5);
      assertEquals(4, previewWindowCalculator.getPlanningWindow().size());

      List<PreviewWindowSegment> previewWindow = previewWindowCalculator.getPlanningWindow();

      for (int i = 0; i < 3; i++)
         assertEquals(1, previewWindow.get(i).getNumberOfContactPhasesInSegment());

      assertEquals(0.0, previewWindow.get(0).getStartTime(), 1e-5);
      assertEquals(1.1 * segmentDuration, previewWindow.get(0).getEndTime(), 1e-5);
      assertEquals(1.1 * segmentDuration, previewWindow.get(1).getStartTime(), 1e-5);
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
   public void computeTwoContactOneFlightThenOneContact()
   {
      YoRegistry testRegistry = new YoRegistry("test");
      PreviewWindowCalculator previewWindowCalculator = new PreviewWindowCalculator(testRegistry);
      setupTest(testRegistry, 0.25, 0.5);

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
      firstContact.getTimeInterval().setInterval(0.0, 0.125);
      firstContact.setStartECMPPosition(startCoP);
      firstContact.setEndECMPPosition(middleCoP);
      firstContact.setLinearECMPVelocity();
      firstContact.addContact(firstPose, contactPolygon);

      ContactPlaneProvider secondContact = new ContactPlaneProvider();
      secondContact.getTimeInterval().setInterval(0.125, 0.25);
      secondContact.setStartECMPPosition(middleCoP);
      secondContact.setEndECMPPosition(middleCoP);
      secondContact.setLinearECMPVelocity();
      secondContact.addContact(firstPose, contactPolygon);

      ContactPlaneProvider thirdContact = new ContactPlaneProvider();
      thirdContact.getTimeInterval().setInterval(0.25, 0.5);
      thirdContact.setContactState(ContactState.FLIGHT);

      ContactPlaneProvider fourthContact = new ContactPlaneProvider();
      fourthContact.getTimeInterval().setInterval(0.5, 0.75);
      fourthContact.setStartECMPPosition(endCoP);
      fourthContact.setEndECMPPosition(endCoP);
      fourthContact.setLinearECMPVelocity();
      fourthContact.addContact(firstPose, contactPolygon);
      fourthContact.addContact(secondPose, contactPolygon);

      List<ContactPlaneProvider> contacts = new ArrayList<>();
      contacts.add(firstContact);
      contacts.add(secondContact);
      contacts.add(thirdContact);
      contacts.add(fourthContact);

      double startTime = 0.128;
      previewWindowCalculator.compute(contacts, 0.128);

      assertEquals(0.75 - startTime, previewWindowCalculator.getPreviewWindowDuration(), 1e-5);
      assertEquals(3, previewWindowCalculator.getPlanningWindow().size());

      List<PreviewWindowSegment> previewWindow = previewWindowCalculator.getPlanningWindow();

      for (int i = 0; i < 3; i++)
         assertEquals(1, previewWindow.get(i).getNumberOfContactPhasesInSegment());

      assertEquals(startTime, previewWindow.get(0).getStartTime(), 1e-5);
      assertEquals(0.25, previewWindow.get(0).getEndTime(), 1e-5);
      assertEquals(0.25, previewWindow.get(1).getStartTime(), 1e-5);
      assertEquals(0.5, previewWindow.get(1).getEndTime(), 1e-5);
      assertEquals(0.5, previewWindow.get(2).getStartTime(), 1e-5);
      assertEquals(0.75, previewWindow.get(2).getEndTime(), 1e-5);

      assertEquals(ContactState.FLIGHT, previewWindow.get(1).getContactState());
   }

   @Test
   public void computeTwoContactsInOnePhase()
   {
      YoRegistry testRegistry = new YoRegistry("test");
      PreviewWindowCalculator previewWindowCalculator = new PreviewWindowCalculator(testRegistry);
      setupTest(testRegistry);

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

   @Test
   public void computeLongContactSequence()
   {
      YoRegistry testRegistry = new YoRegistry("test");
      PreviewWindowCalculator previewWindowCalculator = new PreviewWindowCalculator(testRegistry);
      setupTest(testRegistry);

      double segmentDuration = ((YoDouble) testRegistry.findVariable("nominalSegmentDuration")).getDoubleValue();
      double maximumPreviewWindowDuration = ((YoDouble) testRegistry.findVariable("maximumPreviewWindowDuration")).getDoubleValue();

      FramePoint3D startCoP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, 0.25, 0.3);
      FramePoint3D middleCoP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.2, 0.3, 0.32);
      FramePoint3D endCoP = new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.05, -0.1, -0.32);

      FramePose3D firstPose = new FramePose3D();
      firstPose.getPosition().set(startCoP);

      double startTime = 1.25 * segmentDuration;
      double endTime = startTime + 1.3 * maximumPreviewWindowDuration;

      FrameConvexPolygon2D contactPolygon = new FrameConvexPolygon2D();
      contactPolygon.addVertex(0.1, -0.05);
      contactPolygon.addVertex(0.1, 0.05);
      contactPolygon.addVertex(-0.1, 0.05);
      contactPolygon.addVertex(-0.1, -0.05);
      contactPolygon.update();

      FramePose3D secondPose = new FramePose3D();
      secondPose.getPosition().set(endCoP);

      ContactPlaneProvider firstContact = new ContactPlaneProvider();
      firstContact.getTimeInterval().setInterval(0.0, 1.9 * segmentDuration);
      firstContact.setStartECMPPosition(startCoP);
      firstContact.setEndECMPPosition(middleCoP);
      firstContact.setLinearECMPVelocity();
      firstContact.addContact(firstPose, contactPolygon);

      ContactPlaneProvider secondContact = new ContactPlaneProvider();
      secondContact.getTimeInterval().setInterval(1.9 * segmentDuration, 3.1 * segmentDuration);
      secondContact.setStartECMPPosition(middleCoP);
      secondContact.setEndECMPPosition(endCoP);
      secondContact.setLinearECMPVelocity();
      secondContact.addContact(firstPose, contactPolygon);
      secondContact.addContact(secondPose, contactPolygon);

      ContactPlaneProvider thirdContact = new ContactPlaneProvider();
      thirdContact.getTimeInterval().setInterval(3.1 * segmentDuration, 4.5 * segmentDuration);
      thirdContact.setStartECMPPosition(endCoP);
      thirdContact.setEndECMPPosition(middleCoP);
      thirdContact.setLinearECMPVelocity();
      thirdContact.addContact(firstPose, contactPolygon);

      ContactPlaneProvider fourthContact = new ContactPlaneProvider();
      fourthContact.getTimeInterval().setInterval(4.5 * segmentDuration, endTime);
      fourthContact.setStartECMPPosition(middleCoP);
      fourthContact.setEndECMPPosition(endCoP);
      fourthContact.setLinearECMPVelocity();
      fourthContact.addContact(firstPose, contactPolygon);
      fourthContact.addContact(secondPose, contactPolygon);

      List<ContactPlaneProvider> contacts = new ArrayList<>();
      contacts.add(firstContact);
      contacts.add(secondContact);
      contacts.add(thirdContact);
      contacts.add(fourthContact);

      previewWindowCalculator.compute(contacts, startTime);

      assertEquals(maximumPreviewWindowDuration, previewWindowCalculator.getPreviewWindowDuration(), 1e-5);
      assertEquals(5, previewWindowCalculator.getPlanningWindow().size());

      List<PreviewWindowSegment> previewWindow = previewWindowCalculator.getPlanningWindow();

      for (int i = 0; i < 3; i++)
         assertEquals(1, previewWindow.get(i).getNumberOfContactPhasesInSegment());

      double absoluteEndTime = startTime + maximumPreviewWindowDuration;
      double lastGroupDuration = absoluteEndTime - 4.5 * segmentDuration;
      double middleOfEnd = 0.5 * lastGroupDuration + 4.5 * segmentDuration;

      assertEquals(1.25 * segmentDuration, previewWindow.get(0).getStartTime(), 1e-5);
      assertEquals(1.9 * segmentDuration, previewWindow.get(0).getEndTime(), 1e-5);
      assertEquals(1.9 * segmentDuration, previewWindow.get(1).getStartTime(), 1e-5);
      assertEquals(3.1 * segmentDuration, previewWindow.get(1).getEndTime(), 1e-5);
      assertEquals(3.1 * segmentDuration, previewWindow.get(2).getStartTime(), 1e-5);
      assertEquals(4.5 * segmentDuration, previewWindow.get(2).getEndTime(), 1e-5);
      assertEquals(4.5 * segmentDuration, previewWindow.get(3).getStartTime(), 1e-5);
      assertEquals(middleOfEnd, previewWindow.get(3).getEndTime(), 1e-5);
      assertEquals(middleOfEnd, previewWindow.get(4).getStartTime(), 1e-5);
      assertEquals(absoluteEndTime, previewWindow.get(4).getEndTime(), 1e-5);

      FramePoint3D copAtStartOfWindow = new FramePoint3D();
      copAtStartOfWindow.interpolate(startCoP, middleCoP, startTime / firstContact.getTimeInterval().getDuration());
      FramePoint3D superMiddle = new FramePoint3D();
      superMiddle.interpolate(startCoP, middleCoP, 0.5);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(copAtStartOfWindow, previewWindow.get(0).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(middleCoP, previewWindow.get(0).getContactPhase(0).getECMPEndPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(middleCoP, previewWindow.get(1).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(secondContact.getECMPEndPosition(), previewWindow.get(1).getContactPhase(0).getECMPEndPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(thirdContact.getECMPStartPosition(), previewWindow.get(2).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(thirdContact.getECMPEndPosition(), previewWindow.get(2).getContactPhase(0).getECMPEndPosition(), 1e-4);

      FramePoint3D middleOfEndPosition = new FramePoint3D();
      middleOfEndPosition.interpolate(middleCoP, endCoP, (middleOfEnd - 4.5 * segmentDuration) / (endTime - 4.5 * segmentDuration));
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(fourthContact.getECMPStartPosition(), previewWindow.get(3).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(middleOfEndPosition, previewWindow.get(3).getContactPhase(0).getECMPEndPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(middleOfEndPosition, previewWindow.get(4).getContactPhase(0).getECMPStartPosition(), 1e-4);

      middleOfEndPosition.interpolate(middleCoP, endCoP, (absoluteEndTime - 4.5 * segmentDuration) / (endTime - 4.5 * segmentDuration));
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(middleOfEndPosition, previewWindow.get(4).getContactPhase(0).getECMPEndPosition(), 1e-4);
   }

   @Test
   public void computeInfiniteContactSequence()
   {
      YoRegistry testRegistry = new YoRegistry("test");
      PreviewWindowCalculator previewWindowCalculator = new PreviewWindowCalculator(testRegistry);
      setupTest(testRegistry);

      double segmentDuration = ((YoDouble) testRegistry.findVariable("nominalSegmentDuration")).getDoubleValue();
      double maximumPreviewWindowDuration = ((YoDouble) testRegistry.findVariable("maximumPreviewWindowDuration")).getDoubleValue();

      FramePoint3D startCoP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, 0.25, 0.3);
      FramePoint3D middleCoP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.2, 0.3, 0.32);
      FramePoint3D endCoP = new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.05, -0.1, -0.32);

      FramePose3D firstPose = new FramePose3D();
      firstPose.getPosition().set(startCoP);

      double startTime = 1.25 * segmentDuration;
      double endTime = startTime + 1.3 * maximumPreviewWindowDuration;

      FrameConvexPolygon2D contactPolygon = new FrameConvexPolygon2D();
      contactPolygon.addVertex(0.1, -0.05);
      contactPolygon.addVertex(0.1, 0.05);
      contactPolygon.addVertex(-0.1, 0.05);
      contactPolygon.addVertex(-0.1, -0.05);
      contactPolygon.update();

      FramePose3D secondPose = new FramePose3D();
      secondPose.getPosition().set(endCoP);

      ContactPlaneProvider firstContact = new ContactPlaneProvider();
      firstContact.getTimeInterval().setInterval(0.0, 1.9 * segmentDuration);
      firstContact.setStartECMPPosition(startCoP);
      firstContact.setEndECMPPosition(middleCoP);
      firstContact.setLinearECMPVelocity();
      firstContact.addContact(firstPose, contactPolygon);

      ContactPlaneProvider secondContact = new ContactPlaneProvider();
      secondContact.getTimeInterval().setInterval(1.9 * segmentDuration, Double.POSITIVE_INFINITY);
      secondContact.setStartECMPPosition(middleCoP);
      secondContact.setEndECMPPosition(endCoP);
      secondContact.setLinearECMPVelocity();
      secondContact.addContact(firstPose, contactPolygon);
      secondContact.addContact(secondPose, contactPolygon);

      List<ContactPlaneProvider> contacts = new ArrayList<>();
      contacts.add(firstContact);
      contacts.add(secondContact);

      previewWindowCalculator.compute(contacts, startTime);

      assertEquals(maximumPreviewWindowDuration, previewWindowCalculator.getPreviewWindowDuration(), 1e-5);
      assertEquals(5, previewWindowCalculator.getPlanningWindow().size());

      List<PreviewWindowSegment> previewWindow = previewWindowCalculator.getPlanningWindow();

      for (int i = 0; i < 3; i++)
         assertEquals(1, previewWindow.get(i).getNumberOfContactPhasesInSegment());

      double absoluteEndTime = startTime + maximumPreviewWindowDuration;
      double lastGroupDuration = absoluteEndTime - 1.9 * segmentDuration;
      double lastSegmentDurations = lastGroupDuration / Math.round(lastGroupDuration / segmentDuration);


      assertEquals(1.25 * segmentDuration, previewWindow.get(0).getStartTime(), 1e-5);
      assertEquals(1.9 * segmentDuration, previewWindow.get(0).getEndTime(), 1e-5);
      assertEquals(1.9 * segmentDuration, previewWindow.get(1).getStartTime(), 1e-5);
      assertEquals(1.9 * segmentDuration + lastSegmentDurations, previewWindow.get(1).getEndTime(), 1e-5);
      assertEquals(1.9 * segmentDuration + lastSegmentDurations, previewWindow.get(2).getStartTime(), 1e-5);
      assertEquals(1.9 * segmentDuration + 2.0 * lastSegmentDurations, previewWindow.get(2).getEndTime(), 1e-5);
      assertEquals(1.9 * segmentDuration + 2.0 * lastSegmentDurations, previewWindow.get(3).getStartTime(), 1e-5);
      assertEquals(1.9 * segmentDuration + 3.0 * lastSegmentDurations, previewWindow.get(3).getEndTime(), 1e-5);
      assertEquals(1.9 * segmentDuration + 3.0 * lastSegmentDurations, previewWindow.get(4).getStartTime(), 1e-5);
      assertEquals(1.9 * segmentDuration + 4.0 * lastSegmentDurations, previewWindow.get(4).getEndTime(), 1e-5);

      FramePoint3D copAtStartOfWindow = new FramePoint3D();
      copAtStartOfWindow.interpolate(startCoP, middleCoP, startTime / firstContact.getTimeInterval().getDuration());


      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(copAtStartOfWindow, previewWindow.get(0).getContactPhase(0).getECMPStartPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(middleCoP, previewWindow.get(0).getContactPhase(0).getECMPEndPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(middleCoP, previewWindow.get(1).getContactPhase(0).getECMPStartPosition(), 1e-4);

      FramePoint3D superMiddle = new FramePoint3D();
      double alpha = lastSegmentDurations / 10.0;
      superMiddle.interpolate(middleCoP, endCoP, alpha);

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(superMiddle, previewWindow.get(1).getContactPhase(0).getECMPEndPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(superMiddle, previewWindow.get(2).getContactPhase(0).getECMPStartPosition(), 1e-4);

      alpha *= 2.0;
      superMiddle.interpolate(middleCoP, endCoP, alpha);

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(superMiddle, previewWindow.get(2).getContactPhase(0).getECMPEndPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(superMiddle, previewWindow.get(3).getContactPhase(0).getECMPStartPosition(), 1e-4);

      alpha *= 3.0 / 2.0;
      superMiddle.interpolate(middleCoP, endCoP, alpha);

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(superMiddle, previewWindow.get(3).getContactPhase(0).getECMPEndPosition(), 1e-4);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(superMiddle, previewWindow.get(4).getContactPhase(0).getECMPStartPosition(), 1e-4);

      alpha = 4.0 * lastSegmentDurations / 10.0;
      superMiddle.interpolate(middleCoP, endCoP, alpha);

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(superMiddle, previewWindow.get(4).getContactPhase(0).getECMPEndPosition(), 1e-4);

   }

   @Test
   public void testOneLongSegment()
   {
      YoRegistry testRegistry = new YoRegistry("test");
      PreviewWindowCalculator previewWindowCalculator = new PreviewWindowCalculator(testRegistry);
      setupTest(testRegistry);

      double segmentDuration = ((YoDouble) testRegistry.findVariable("nominalSegmentDuration")).getDoubleValue();
      double maximumPreviewWindowDuration = ((YoDouble) testRegistry.findVariable("maximumPreviewWindowDuration")).getDoubleValue();

      FramePoint3D startCoP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, 0.25, 0.3);

      FramePose3D firstPose = new FramePose3D();
      firstPose.getPosition().set(startCoP);

      double startTime = 0.2 * maximumPreviewWindowDuration;

      FrameConvexPolygon2D contactPolygon = new FrameConvexPolygon2D();
      contactPolygon.addVertex(0.1, -0.05);
      contactPolygon.addVertex(0.1, 0.05);
      contactPolygon.addVertex(-0.1, 0.05);
      contactPolygon.addVertex(-0.1, -0.05);
      contactPolygon.update();

      ContactPlaneProvider firstContact = new ContactPlaneProvider();
      firstContact.getTimeInterval().setInterval(0.2 * maximumPreviewWindowDuration, 2.0 * maximumPreviewWindowDuration);
      firstContact.setStartECMPPosition(startCoP);
      firstContact.setEndECMPPosition(startCoP);
      firstContact.setLinearECMPVelocity();
      firstContact.addContact(firstPose, contactPolygon);


      List<ContactPlaneProvider> contacts = new ArrayList<>();
      contacts.add(firstContact);

      previewWindowCalculator.compute(contacts, startTime);

      assertEquals(maximumPreviewWindowDuration, previewWindowCalculator.getPreviewWindowDuration(), 1e-5);
      assertEquals((int) Math.round(maximumPreviewWindowDuration / segmentDuration), previewWindowCalculator.getPlanningWindow().size());

      List<PreviewWindowSegment> previewWindow = previewWindowCalculator.getPlanningWindow();

      FrameVector3DBasics zeroVector = new FrameVector3D();

      for (int i = 0; i < (int) Math.round(maximumPreviewWindowDuration / segmentDuration); i++)
      {
         assertEquals(1, previewWindow.get(i).getNumberOfContactPhasesInSegment());

         assertEquals(startTime, previewWindow.get(i).getStartTime(), 1e-5);
         assertEquals(startTime + segmentDuration, previewWindow.get(i).getEndTime(), 1e-5);

         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(startCoP, previewWindow.get(i).getContactPhase(0).getECMPEndPosition(), 1e-4);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(startCoP, previewWindow.get(i).getContactPhase(0).getECMPStartPosition(), 1e-4);

         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(zeroVector, previewWindow.get(i).getContactPhase(0).getECMPStartVelocity(), 1e-4);
         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(zeroVector, previewWindow.get(i).getContactPhase(0).getECMPEndVelocity(), 1e-4);

         startTime += segmentDuration;
      }
   }

   @Test
   public void testStandingBug()
   {
      YoRegistry testRegistry = new YoRegistry("test");
      PreviewWindowCalculator previewWindowCalculator = new PreviewWindowCalculator(testRegistry);

      setupTest(testRegistry, 0.25, 0.5);

      double segmentDuration = ((YoDouble) testRegistry.findVariable("nominalSegmentDuration")).getDoubleValue();
      double maximumPreviewWindowDuration = ((YoDouble) testRegistry.findVariable("maximumPreviewWindowDuration")).getDoubleValue();

      FramePoint3D startCoP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, 0.25, 0.3);

      FramePose3D firstPose = new FramePose3D();
      firstPose.getPosition().set(startCoP);

      double startTime = 00.0;

      FrameConvexPolygon2D contactPolygon = new FrameConvexPolygon2D();
      contactPolygon.addVertex(0.1, -0.05);
      contactPolygon.addVertex(0.1, 0.05);
      contactPolygon.addVertex(-0.1, 0.05);
      contactPolygon.addVertex(-0.1, -0.05);
      contactPolygon.update();

      ContactPlaneProvider firstContactFirstTick = new ContactPlaneProvider();
      firstContactFirstTick.getTimeInterval().setInterval(0.0, 0.125);
      firstContactFirstTick.setStartECMPPosition(startCoP);
      firstContactFirstTick.setEndECMPPosition(startCoP);
      firstContactFirstTick.setLinearECMPVelocity();
      firstContactFirstTick.addContact(firstPose, contactPolygon);


      ContactPlaneProvider secondContact = new ContactPlaneProvider();
      secondContact.getTimeInterval().setInterval(0.125, 0.25);
      secondContact.setStartECMPPosition(startCoP);
      secondContact.setEndECMPPosition(startCoP);
      secondContact.setLinearECMPVelocity();
      secondContact.addContact(firstPose, contactPolygon);

      ContactPlaneProvider thirdContactFirstTick = new ContactPlaneProvider();
      thirdContactFirstTick.getTimeInterval().setInterval(0.25, Double.POSITIVE_INFINITY);
      thirdContactFirstTick.setStartECMPPosition(startCoP);
      thirdContactFirstTick.setEndECMPPosition(startCoP);
      thirdContactFirstTick.setLinearECMPVelocity();
      thirdContactFirstTick.addContact(firstPose, contactPolygon);


      List<ContactPlaneProvider> contacts = new ArrayList<>();
      contacts.add(firstContactFirstTick);
      contacts.add(secondContact);
      contacts.add(thirdContactFirstTick);

      previewWindowCalculator.compute(contacts, 0.0);

      assertEquals(maximumPreviewWindowDuration, previewWindowCalculator.getPreviewWindowDuration(), 1e-5);
      assertEquals((int) Math.round(maximumPreviewWindowDuration / segmentDuration), previewWindowCalculator.getPlanningWindow().size());

      List<PreviewWindowSegment> previewWindow = previewWindowCalculator.getPlanningWindow();

      FrameVector3DBasics zeroVector = new FrameVector3D();

      for (int i = 0; i < (int) Math.round(maximumPreviewWindowDuration / segmentDuration); i++)
      {
         assertEquals(1, previewWindow.get(i).getNumberOfContactPhasesInSegment());

         assertEquals(startTime, previewWindow.get(i).getStartTime(), 1e-5);
         assertEquals(startTime + segmentDuration, previewWindow.get(i).getEndTime(), 1e-5);

         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(startCoP, previewWindow.get(i).getContactPhase(0).getECMPEndPosition(), 1e-4);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(startCoP, previewWindow.get(i).getContactPhase(0).getECMPStartPosition(), 1e-4);

         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(zeroVector, previewWindow.get(i).getContactPhase(0).getECMPStartVelocity(), 1e-4);
         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(zeroVector, previewWindow.get(i).getContactPhase(0).getECMPEndVelocity(), 1e-4);

         startTime += segmentDuration;
      }

      previewWindowCalculator.compute(contacts, 0.004);

      assertEquals(2, previewWindowCalculator.getPlanningWindow().size());
      assertEquals(maximumPreviewWindowDuration, previewWindowCalculator.getPreviewWindowDuration(), 1e-4);

      assertEquals(2, previewWindowCalculator.getPlanningWindow().get(0).getNumberOfContactPhasesInSegment());
      assertEquals(0.004, previewWindowCalculator.getPlanningWindow().get(0).getContactPhase(0).getTimeInterval().getStartTime(), 1e-4);
      assertEquals(0.25, previewWindowCalculator.getPlanningWindow().get(0).getContactPhase(0).getTimeInterval().getEndTime(), 1e-4);
      assertEquals(0.25, previewWindowCalculator.getPlanningWindow().get(0).getContactPhase(1).getTimeInterval().getStartTime(), 1e-4);
      assertEquals(0.254, previewWindowCalculator.getPlanningWindow().get(0).getContactPhase(1).getTimeInterval().getEndTime(), 1e-4);
      assertEquals(0.254, previewWindowCalculator.getPlanningWindow().get(1).getContactPhase(0).getTimeInterval().getStartTime(), 1e-4);
      assertEquals(0.504, previewWindowCalculator.getPlanningWindow().get(1).getContactPhase(0).getTimeInterval().getEndTime(), 1e-4);

      assertEquals(0.004, previewWindowCalculator.getPlanningWindow().get(0).getStartTime(), 1e-4);
      assertEquals(0.254, previewWindowCalculator.getPlanningWindow().get(0).getEndTime(), 1e-4);
      assertEquals(0.254, previewWindowCalculator.getPlanningWindow().get(1).getStartTime(), 1e-4);
      assertEquals(0.504, previewWindowCalculator.getPlanningWindow().get(1).getEndTime(), 1e-4);
   }
}
