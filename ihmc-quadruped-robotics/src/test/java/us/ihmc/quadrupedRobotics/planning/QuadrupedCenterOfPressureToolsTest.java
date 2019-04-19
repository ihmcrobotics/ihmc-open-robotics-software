package us.ihmc.quadrupedRobotics.planning;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class QuadrupedCenterOfPressureToolsTest
{
   private static final double epsilon = 1e-9;

   @Test
   public void testComputeNominalNormalizedContactPressure()
   {
      QuadrantDependentList<MutableDouble> normalizedContactPressures = new QuadrantDependentList<>();
      QuadrantDependentList<MutableDouble> normalizedContactPressures2 = new QuadrantDependentList<>();
      QuadrantDependentList<ContactState> contactStates = new QuadrantDependentList<>();
      List<RobotQuadrant> feetInContact = new ArrayList<>();

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         normalizedContactPressures.put(quadrant, new MutableDouble());
         normalizedContactPressures2.put(quadrant, new MutableDouble());
      }

      // all four in contact
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         contactStates.put(quadrant, ContactState.IN_CONTACT);
         feetInContact.add(quadrant);
      }

      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures, contactStates);
      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures2, feetInContact);

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         assertEquals(0.25, normalizedContactPressures.get(quadrant).getValue(), epsilon);
         assertEquals(0.25, normalizedContactPressures2.get(quadrant).getValue(), epsilon);
      }

      // back two and front right in contact
      feetInContact.clear();
      feetInContact.add(RobotQuadrant.FRONT_LEFT);
      feetInContact.add(RobotQuadrant.HIND_LEFT);
      feetInContact.add(RobotQuadrant.HIND_RIGHT);

      contactStates.clear();
      contactStates.put(RobotQuadrant.FRONT_LEFT, ContactState.IN_CONTACT);
      contactStates.put(RobotQuadrant.FRONT_RIGHT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.HIND_LEFT, ContactState.IN_CONTACT);
      contactStates.put(RobotQuadrant.HIND_RIGHT, ContactState.IN_CONTACT);

      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures, contactStates);
      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures2, feetInContact);

      assertEquals(0.25, normalizedContactPressures.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);
      assertEquals(0.25, normalizedContactPressures2.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);

      assertEquals(0.25, normalizedContactPressures.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);
      assertEquals(0.25, normalizedContactPressures2.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);

      assertEquals(0.5, normalizedContactPressures.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);
      assertEquals(0.5, normalizedContactPressures2.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);

      // back two and front left in contact
      feetInContact.clear();
      feetInContact.add(RobotQuadrant.FRONT_RIGHT);
      feetInContact.add(RobotQuadrant.HIND_LEFT);
      feetInContact.add(RobotQuadrant.HIND_RIGHT);

      contactStates.clear();
      contactStates.put(RobotQuadrant.FRONT_LEFT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.FRONT_RIGHT, ContactState.IN_CONTACT);
      contactStates.put(RobotQuadrant.HIND_LEFT, ContactState.IN_CONTACT);
      contactStates.put(RobotQuadrant.HIND_RIGHT, ContactState.IN_CONTACT);

      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures, contactStates);
      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures2, feetInContact);

      assertEquals(0.25, normalizedContactPressures.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);
      assertEquals(0.25, normalizedContactPressures2.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);

      assertEquals(0.25, normalizedContactPressures.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);
      assertEquals(0.25, normalizedContactPressures2.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);

      assertEquals(0.5, normalizedContactPressures.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);
      assertEquals(0.5, normalizedContactPressures2.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);

      // front two and back right in contact
      feetInContact.clear();
      feetInContact.add(RobotQuadrant.FRONT_LEFT);
      feetInContact.add(RobotQuadrant.FRONT_RIGHT);
      feetInContact.add(RobotQuadrant.HIND_RIGHT);

      contactStates.clear();
      contactStates.put(RobotQuadrant.FRONT_LEFT, ContactState.IN_CONTACT);
      contactStates.put(RobotQuadrant.FRONT_RIGHT, ContactState.IN_CONTACT);
      contactStates.put(RobotQuadrant.HIND_LEFT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.HIND_RIGHT, ContactState.IN_CONTACT);

      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures, contactStates);
      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures2, feetInContact);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);

      assertEquals(0.5, normalizedContactPressures.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);
      assertEquals(0.5, normalizedContactPressures2.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);

      assertEquals(0.25, normalizedContactPressures.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);
      assertEquals(0.25, normalizedContactPressures2.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);

      assertEquals(0.25, normalizedContactPressures.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);
      assertEquals(0.25, normalizedContactPressures2.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);

      // front two and back left in contact
      feetInContact.clear();
      feetInContact.add(RobotQuadrant.FRONT_LEFT);
      feetInContact.add(RobotQuadrant.FRONT_RIGHT);
      feetInContact.add(RobotQuadrant.HIND_LEFT);

      contactStates.clear();
      contactStates.put(RobotQuadrant.FRONT_LEFT, ContactState.IN_CONTACT);
      contactStates.put(RobotQuadrant.FRONT_RIGHT, ContactState.IN_CONTACT);
      contactStates.put(RobotQuadrant.HIND_LEFT, ContactState.IN_CONTACT);
      contactStates.put(RobotQuadrant.HIND_RIGHT, ContactState.NO_CONTACT);

      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures, contactStates);
      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures2, feetInContact);

      assertEquals(0.5, normalizedContactPressures.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);
      assertEquals(0.5, normalizedContactPressures2.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);

      assertEquals(0.25, normalizedContactPressures.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);
      assertEquals(0.25, normalizedContactPressures2.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);

      assertEquals(0.25, normalizedContactPressures.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);
      assertEquals(0.25, normalizedContactPressures2.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);

      // front two in contact
      feetInContact.clear();
      feetInContact.add(RobotQuadrant.FRONT_LEFT);
      feetInContact.add(RobotQuadrant.FRONT_RIGHT);

      contactStates.clear();
      contactStates.put(RobotQuadrant.FRONT_LEFT, ContactState.IN_CONTACT);
      contactStates.put(RobotQuadrant.FRONT_RIGHT, ContactState.IN_CONTACT);
      contactStates.put(RobotQuadrant.HIND_LEFT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.HIND_RIGHT, ContactState.NO_CONTACT);

      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures, contactStates);
      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures2, feetInContact);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);

      assertEquals(0.5, normalizedContactPressures.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);
      assertEquals(0.5, normalizedContactPressures2.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);

      assertEquals(0.5, normalizedContactPressures.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);
      assertEquals(0.5, normalizedContactPressures2.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);

      // back two in contact
      feetInContact.clear();
      feetInContact.add(RobotQuadrant.HIND_LEFT);
      feetInContact.add(RobotQuadrant.HIND_RIGHT);

      contactStates.clear();
      contactStates.put(RobotQuadrant.FRONT_LEFT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.FRONT_RIGHT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.HIND_LEFT, ContactState.IN_CONTACT);
      contactStates.put(RobotQuadrant.HIND_RIGHT, ContactState.IN_CONTACT);

      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures, contactStates);
      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures2, feetInContact);

      assertEquals(0.5, normalizedContactPressures.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);
      assertEquals(0.5, normalizedContactPressures2.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);

      assertEquals(0.5, normalizedContactPressures.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);
      assertEquals(0.5, normalizedContactPressures2.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);

      // left two in contact
      feetInContact.clear();
      feetInContact.add(RobotQuadrant.HIND_LEFT);
      feetInContact.add(RobotQuadrant.FRONT_LEFT);

      contactStates.clear();
      contactStates.put(RobotQuadrant.FRONT_LEFT, ContactState.IN_CONTACT);
      contactStates.put(RobotQuadrant.FRONT_RIGHT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.HIND_LEFT, ContactState.IN_CONTACT);
      contactStates.put(RobotQuadrant.HIND_RIGHT, ContactState.NO_CONTACT);

      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures, contactStates);
      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures2, feetInContact);

      assertEquals(0.5, normalizedContactPressures.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);
      assertEquals(0.5, normalizedContactPressures2.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);

      assertEquals(0.5, normalizedContactPressures.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);
      assertEquals(0.5, normalizedContactPressures2.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);

      // right two in contact
      feetInContact.clear();
      feetInContact.add(RobotQuadrant.HIND_RIGHT);
      feetInContact.add(RobotQuadrant.FRONT_RIGHT);

      contactStates.clear();
      contactStates.put(RobotQuadrant.FRONT_LEFT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.FRONT_RIGHT, ContactState.IN_CONTACT);
      contactStates.put(RobotQuadrant.HIND_LEFT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.HIND_RIGHT, ContactState.IN_CONTACT);

      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures, contactStates);
      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures2, feetInContact);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);

      assertEquals(0.5, normalizedContactPressures.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);
      assertEquals(0.5, normalizedContactPressures2.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);

      assertEquals(0.5, normalizedContactPressures.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);
      assertEquals(0.5, normalizedContactPressures2.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);

      // front left hind right in contact
      feetInContact.clear();
      feetInContact.add(RobotQuadrant.HIND_RIGHT);
      feetInContact.add(RobotQuadrant.FRONT_LEFT);

      contactStates.clear();
      contactStates.put(RobotQuadrant.FRONT_LEFT, ContactState.IN_CONTACT);
      contactStates.put(RobotQuadrant.FRONT_RIGHT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.HIND_LEFT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.HIND_RIGHT, ContactState.IN_CONTACT);

      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures, contactStates);
      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures2, feetInContact);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);

      assertEquals(0.5, normalizedContactPressures.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);
      assertEquals(0.5, normalizedContactPressures2.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);

      assertEquals(0.5, normalizedContactPressures.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);
      assertEquals(0.5, normalizedContactPressures2.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);

      // front right hind left in contact
      feetInContact.clear();
      feetInContact.add(RobotQuadrant.HIND_LEFT);
      feetInContact.add(RobotQuadrant.FRONT_RIGHT);

      contactStates.clear();
      contactStates.put(RobotQuadrant.FRONT_LEFT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.FRONT_RIGHT, ContactState.IN_CONTACT);
      contactStates.put(RobotQuadrant.HIND_LEFT, ContactState.IN_CONTACT);
      contactStates.put(RobotQuadrant.HIND_RIGHT, ContactState.NO_CONTACT);

      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures, contactStates);
      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures2, feetInContact);

      assertEquals(0.5, normalizedContactPressures.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);
      assertEquals(0.5, normalizedContactPressures2.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);

      assertEquals(0.5, normalizedContactPressures.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);
      assertEquals(0.5, normalizedContactPressures2.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);

      // hind left in contact
      feetInContact.clear();
      feetInContact.add(RobotQuadrant.HIND_LEFT);

      contactStates.clear();
      contactStates.put(RobotQuadrant.FRONT_LEFT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.FRONT_RIGHT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.HIND_LEFT, ContactState.IN_CONTACT);
      contactStates.put(RobotQuadrant.HIND_RIGHT, ContactState.NO_CONTACT);

      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures, contactStates);
      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures2, feetInContact);

      assertEquals(1.0, normalizedContactPressures.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);
      assertEquals(1.0, normalizedContactPressures2.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);

      // hind right in contact
      feetInContact.clear();
      feetInContact.add(RobotQuadrant.HIND_RIGHT);

      contactStates.clear();
      contactStates.put(RobotQuadrant.FRONT_LEFT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.FRONT_RIGHT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.HIND_LEFT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.HIND_RIGHT, ContactState.IN_CONTACT);

      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures, contactStates);
      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures2, feetInContact);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);

      assertEquals(1.0, normalizedContactPressures.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);
      assertEquals(1.0, normalizedContactPressures2.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);

      // front left in contact
      feetInContact.clear();
      feetInContact.add(RobotQuadrant.FRONT_LEFT);

      contactStates.clear();
      contactStates.put(RobotQuadrant.FRONT_LEFT, ContactState.IN_CONTACT);
      contactStates.put(RobotQuadrant.FRONT_RIGHT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.HIND_LEFT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.HIND_RIGHT, ContactState.NO_CONTACT);

      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures, contactStates);
      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures2, feetInContact);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);

      assertEquals(1.0, normalizedContactPressures.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);
      assertEquals(1.0, normalizedContactPressures2.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);

      // front right in contact
      feetInContact.clear();
      feetInContact.add(RobotQuadrant.FRONT_RIGHT);

      contactStates.clear();
      contactStates.put(RobotQuadrant.FRONT_LEFT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.FRONT_RIGHT, ContactState.IN_CONTACT);
      contactStates.put(RobotQuadrant.HIND_LEFT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.HIND_RIGHT, ContactState.NO_CONTACT);

      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures, contactStates);
      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures2, feetInContact);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);

      assertEquals(1.0, normalizedContactPressures.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);
      assertEquals(1.0, normalizedContactPressures2.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);

      // no contact
      feetInContact.clear();

      contactStates.clear();
      contactStates.put(RobotQuadrant.FRONT_LEFT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.FRONT_RIGHT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.HIND_LEFT, ContactState.NO_CONTACT);
      contactStates.put(RobotQuadrant.HIND_RIGHT, ContactState.NO_CONTACT);

      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures, contactStates);
      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures2, feetInContact);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.HIND_LEFT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.HIND_RIGHT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.FRONT_LEFT).getValue(), epsilon);

      assertEquals(0.0, normalizedContactPressures.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);
      assertEquals(0.0, normalizedContactPressures2.get(RobotQuadrant.FRONT_RIGHT).getValue(), epsilon);
   }

   @Test
   public void testComputeCenterOfPressure()
   {
      Random random = new Random(1738L);

      QuadrantDependentList<FramePoint3D> solePositions = new QuadrantDependentList<>();
      QuadrantDependentList<MutableDouble> normalizedPressures = new QuadrantDependentList<>();

      for (int iter = 0; iter < 100; iter++)
      {
         double pressure = 0.0;
         for (RobotQuadrant quadrant : RobotQuadrant.values)
         {
            solePositions.put(quadrant, EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame(), 0.5));
            normalizedPressures.put(quadrant, new MutableDouble(RandomNumbers.nextDouble(random, 0.0, 1.0)));
            pressure += normalizedPressures.get(quadrant).getValue();
         }

         FramePoint3D copPosition = new FramePoint3D();
         QuadrupedCenterOfPressureTools.computeCenterOfPressure(copPosition, solePositions, normalizedPressures);

         FramePoint3D expectedCoPPosition = new FramePoint3D();
         for (RobotQuadrant quadrant : RobotQuadrant.values)
         {
            FramePoint3D point = new FramePoint3D(solePositions.get(quadrant));
            point.scale(normalizedPressures.get(quadrant).getValue());
            expectedCoPPosition.add(point);
         }

         expectedCoPPosition.scale(1.0 / pressure);

         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(expectedCoPPosition, copPosition, epsilon);
      }
   }
}
