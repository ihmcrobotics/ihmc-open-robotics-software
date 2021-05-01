package us.ihmc.quadrupedRobotics.planning;

import org.apache.commons.lang3.mutable.MutableDouble;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.quadrupedRobotics.estimator.PitchEstimator;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.List;

public class QuadrupedCenterOfPressureTools
{
   /**
    * Compute center of pressure position given sole positions and vertical ground reaction forces.
    * @param copPosition center of pressure position (output)
    * @param solePosition contact position for each quadrant (input)
    * @param contactPressure vertical ground reaction forces for each quadrant (input)
    */
   public static void computeCenterOfPressure(FixedFramePoint3DBasics copPosition, QuadrantDependentList<FramePoint3D> solePosition,
                                              QuadrantDependentList<MutableDouble> contactPressure)
   {
      // Compute center of pressure given the vertical force at each contact.
      double pressure = 0.0;
      copPosition.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      copPosition.setToZero();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         pressure += contactPressure.get(robotQuadrant).doubleValue();
         solePosition.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
         copPosition.scaleAdd(contactPressure.get(robotQuadrant).doubleValue(), solePosition.get(robotQuadrant), copPosition);
      }
      if (pressure > 0.0)
         copPosition.scale(1.0 / pressure);
      else
         copPosition.setToNaN();
   }

   /**
    * Compute nominal pressure distribution for a given contact state.
    * @param contactPressureToPack nominal vertical ground reaction forces for each quadrant
    * @param contactState contact state for each quadrant
    */
   public static void computeNominalNormalizedContactPressure(QuadrantDependentList<MutableDouble> contactPressureToPack,
                                                              QuadrantDependentList<ContactState> contactState)
   {
      // Compute vertical force distribution assuming equal loading of hind and front ends.
      int numberOfHindFeetInContact = 0;
      int numberOfFrontFeetInContact = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant) == ContactState.IN_CONTACT)
         {
            if (robotQuadrant.isQuadrantInFront())
            {
               numberOfFrontFeetInContact++;
            }
            else
            {
               numberOfHindFeetInContact++;
            }
            contactPressureToPack.get(robotQuadrant).setValue(1.0);
         }
         else
         {
            contactPressureToPack.get(robotQuadrant).setValue(0.0);
         }
      }

      double numberOfEndsInContact = 0.0;
      if ((numberOfHindFeetInContact > 0) ^ (numberOfFrontFeetInContact > 0))
      {
         numberOfEndsInContact = 1.0;
      }
      if ((numberOfHindFeetInContact > 0) && (numberOfFrontFeetInContact > 0))
      {
         numberOfEndsInContact = 2.0;
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         double pressure = contactPressureToPack.get(robotQuadrant).doubleValue();
         pressure /= Math.max(numberOfEndsInContact, 1.0);
         pressure /= Math.max((robotQuadrant.isQuadrantInFront() ? numberOfFrontFeetInContact : numberOfHindFeetInContact), 1.0);
         contactPressureToPack.get(robotQuadrant).setValue(pressure);
      }
   }

   /**
    * Compute nominal pressure distribution for a given contact state.
    * @param contactPressuresToPack nominal vertical ground reaction forces for each quadrant
    * @param feetInContact feet that are in contact
    */
   public static void computeNominalNormalizedContactPressure(QuadrantDependentList<MutableDouble> contactPressuresToPack,
                                                              List<RobotQuadrant> feetInContact)
   {
      // Compute vertical force distribution assuming equal loading of hind and front ends.
      int numberOfHindFeetInContact = 0;
      int numberOfFrontFeetInContact = 0;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         contactPressuresToPack.get(robotQuadrant).setValue(0.0);

      for (int footIndex = 0; footIndex < feetInContact.size(); footIndex++)
      {
         RobotQuadrant quadrantInContact = feetInContact.get(footIndex);
         if (quadrantInContact.isQuadrantInFront())
            numberOfFrontFeetInContact++;
         else
            numberOfHindFeetInContact++;

         contactPressuresToPack.get(quadrantInContact).setValue(1.0);
      }


      double numberOfEndsInContact = 0.0;
      if ((numberOfHindFeetInContact > 0) ^ (numberOfFrontFeetInContact > 0))
      {
         numberOfEndsInContact = 1.0;
      }
      if ((numberOfHindFeetInContact > 0) && (numberOfFrontFeetInContact > 0))
      {
         numberOfEndsInContact = 2.0;
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         double pressure = contactPressuresToPack.get(robotQuadrant).doubleValue();
         pressure /= Math.max(numberOfEndsInContact, 1.0);
         pressure /= Math.max((robotQuadrant.isQuadrantInFront() ? numberOfFrontFeetInContact : numberOfHindFeetInContact), 1.0);
         contactPressuresToPack.get(robotQuadrant).setValue(pressure);
      }
   }

   /**
    * Compute nominal pressure distribution for a given contact state.
    * @param contactPressure nominal vertical ground reaction forces for each quadrant
    * @param contactState contact state for each quadrant
    */
   public static void computeNominalNormalizedContactPressure(QuadrantDependentList<MutableDouble> contactPressure,
                                                              QuadrantDependentList<ContactState> contactState,
                                                              QuadrantDependentList<? extends Point3DReadOnly> contactPositions,
                                                              WeightDistributionCalculator weightDistributionCalculator)
   {
      double nominalPitch = PitchEstimator.computeGroundPitchFromContacts(contactPositions);

      // If +1.0, all the weight on the front. If -1.0, all the weight on the back
      double forwardFraction = weightDistributionCalculator.getFractionOfWeightForward(nominalPitch);

      // Compute vertical force distribution assuming equal loading of hind and front ends.
      int numberOfHindFeetInContact = 0;
      int numberOfFrontFeetInContact = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant) == ContactState.IN_CONTACT)
         {
            if (robotQuadrant.isQuadrantInFront())
            {
               numberOfFrontFeetInContact++;
            }
            else
            {
               numberOfHindFeetInContact++;
            }
            contactPressure.get(robotQuadrant).setValue(1.0);
         }
         else
         {
            contactPressure.get(robotQuadrant).setValue(0.0);
         }
      }

      double forwardWeightDistribution = 1.0;
      double rearWeightDistribution = 1.0;
      if ((numberOfHindFeetInContact > 0) ^ (numberOfFrontFeetInContact > 0))
      {
         forwardWeightDistribution = 1.0;
         rearWeightDistribution = 1.0;
      }
      if ((numberOfHindFeetInContact > 0) && (numberOfFrontFeetInContact > 0))
      {
         forwardWeightDistribution = 0.5 * forwardFraction + 0.5;
         rearWeightDistribution = 1.0 - forwardWeightDistribution;
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         double pressure = contactPressure.get(robotQuadrant).doubleValue();
         pressure *= robotQuadrant.isQuadrantInFront() ? forwardWeightDistribution : rearWeightDistribution;
         pressure /= Math.max((robotQuadrant.isQuadrantInFront() ? numberOfFrontFeetInContact : numberOfHindFeetInContact), 1.0);
         contactPressure.get(robotQuadrant).setValue(pressure);
      }
   }
}
