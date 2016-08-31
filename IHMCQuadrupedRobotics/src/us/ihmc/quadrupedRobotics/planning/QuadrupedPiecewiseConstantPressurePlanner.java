package us.ihmc.quadrupedRobotics.planning;

import org.apache.commons.lang3.mutable.MutableBoolean;
import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedPiecewiseConstantPressurePlanner
{
   private final QuadrantDependentList<ContactState> initialContactState;
   private final QuadrantDependentList<MutableBoolean> isInitialContactState;

   public QuadrupedPiecewiseConstantPressurePlanner()
   {
      initialContactState = new QuadrantDependentList<>();
      isInitialContactState = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         initialContactState.set(robotQuadrant, ContactState.IN_CONTACT);
         isInitialContactState.set(robotQuadrant, new MutableBoolean(true));
      }
   }

   /**
    * compute piecewise constant center of pressure plan given the upcoming contact states
    * @param piecewiseConstantPressurePlan piecewise constant pressure plan (output)
    * @param contactStatePlan contact state plan (input)
    */
   public void compute(QuadrupedPiecewiseConstantPressurePlan piecewiseConstantPressurePlan, QuadrupedContactStatePlan contactStatePlan)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         initialContactState.set(robotQuadrant, contactStatePlan.getContactStateAtStartOfInterval(0, robotQuadrant));
         isInitialContactState.get(robotQuadrant).setTrue();
      }

      for (int interval = 0; interval < contactStatePlan.getNumberOfIntervals(); interval++)
      {
         MutableDouble timeAtStartOfInterval = contactStatePlan.getTimeAtStartOfInterval().get(interval);
         QuadrantDependentList<FramePoint> solePosition = contactStatePlan.getSolePositionAtStartOfInterval().get(interval);
         QuadrantDependentList<ContactState> contactState = contactStatePlan.getContactStateAtStartOfInterval().get(interval);
         FramePoint centerOfPressure = piecewiseConstantPressurePlan.getCenterOfPressureAtStartOfInterval().get(interval);
         QuadrantDependentList<MutableDouble> normalizedPressure = piecewiseConstantPressurePlan.getNormalizedPressureAtStartOfInterval().get(interval);
         MutableDouble normalizedPressureContributedByQueuedSteps = piecewiseConstantPressurePlan.getNormalizedPressureContributedByQueuedSteps().get(interval);
         MutableDouble normalizedPressureContributedByInitialContacts = piecewiseConstantPressurePlan
               .getNormalizedPressureContributedByInitialContacts().get(interval);

         computeNormalizedContactPressure(normalizedPressure, contactState);
         computeCenterOfPressure(centerOfPressure, solePosition, normalizedPressure);
         normalizedPressureContributedByQueuedSteps.setValue(0.0);
         normalizedPressureContributedByInitialContacts.setValue(0.0);
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            if (contactState.get(robotQuadrant) != initialContactState.get(robotQuadrant))
            {
               isInitialContactState.get(robotQuadrant).setFalse();
            }
            if (isInitialContactState.get(robotQuadrant).booleanValue())
            {
               normalizedPressureContributedByInitialContacts.add(normalizedPressure.get(robotQuadrant));
            }
            else
            {
               normalizedPressureContributedByQueuedSteps.add(normalizedPressure.get(robotQuadrant));
            }
         }

         piecewiseConstantPressurePlan.getTimeAtStartOfInterval().get(interval).setValue(timeAtStartOfInterval);
      }

      piecewiseConstantPressurePlan.setNumberOfIntervals(contactStatePlan.getNumberOfIntervals());
   }

   private void computeNormalizedContactPressure(QuadrantDependentList<MutableDouble> contactPressure, QuadrantDependentList<ContactState> contactState)
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
            contactPressure.get(robotQuadrant).setValue(1.0);
         }
         else
         {
            contactPressure.get(robotQuadrant).setValue(0.0);
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
         double pressure = contactPressure.get(robotQuadrant).doubleValue();
         pressure /= Math.max(numberOfEndsInContact, 1.0);
         pressure /= Math.max((robotQuadrant.isQuadrantInFront() ? numberOfFrontFeetInContact : numberOfHindFeetInContact), 1.0);
         contactPressure.get(robotQuadrant).setValue(pressure);
      }
   }

   private void computeCenterOfPressure(FramePoint copPosition, QuadrantDependentList<FramePoint> solePosition,
         QuadrantDependentList<MutableDouble> contactPressure)
   {
      // Compute center of pressure given the vertical force at each contact.
      double pressure = 1e-6;
      copPosition.setToZero(ReferenceFrame.getWorldFrame());
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         pressure += contactPressure.get(robotQuadrant).doubleValue();
         solePosition.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
         addPointWithScaleFactor(copPosition, solePosition.get(robotQuadrant), contactPressure.get(robotQuadrant).doubleValue());
      }
      copPosition.scale(1.0 / pressure);
   }

   private void addPointWithScaleFactor(FramePoint point, FramePoint pointToAdd, double scaleFactor)
   {
      point.checkReferenceFrameMatch(pointToAdd);
      point.add(scaleFactor * pointToAdd.getX(), scaleFactor * pointToAdd.getY(), scaleFactor * pointToAdd.getZ());
   }
}
