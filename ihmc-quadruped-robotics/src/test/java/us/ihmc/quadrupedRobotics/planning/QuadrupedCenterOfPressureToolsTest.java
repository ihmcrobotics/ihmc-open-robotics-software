package us.ihmc.quadrupedRobotics.planning;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertEquals;

public class QuadrupedCenterOfPressureToolsTest
{
   private static final double epsilon = 1e-9;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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
}
