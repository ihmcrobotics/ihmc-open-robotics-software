package us.ihmc.quadrupedRobotics.planning;

import org.apache.commons.lang3.mutable.MutableBoolean;
import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;

public class QuadrupedPiecewiseConstantPressureSequence
{
   // internal
   private final QuadrantDependentList<ContactState> initialContactState;
   private final QuadrantDependentList<MutableBoolean> isInitialContactState;

   // external
   private int numberOfIntervals;
   private final ArrayList<MutableDouble> timeAtStartOfInterval;
   private final ArrayList<FramePoint> centerOfPressureAtStartOfInterval;
   private final ArrayList<QuadrantDependentList<MutableDouble>> normalizedPressureAtStartOfInterval;
   private final ArrayList<MutableDouble> normalizedPressureContributedByInitialContacts;
   private final ArrayList<MutableDouble> normalizedPressureContributedByQueuedSteps;

   public QuadrupedPiecewiseConstantPressureSequence(int maxSteps)
   {
      int maxIntervals = 2 * maxSteps + 2;

      // internal
      initialContactState = new QuadrantDependentList<>();
      isInitialContactState = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         initialContactState.set(robotQuadrant, ContactState.IN_CONTACT);
         isInitialContactState.set(robotQuadrant, new MutableBoolean(true));
      }

      // external
      numberOfIntervals = 0;
      timeAtStartOfInterval = new ArrayList<>(maxIntervals);
      centerOfPressureAtStartOfInterval = new ArrayList<>(maxIntervals);
      normalizedPressureContributedByInitialContacts = new ArrayList<>(maxIntervals);
      normalizedPressureContributedByQueuedSteps = new ArrayList<>(maxIntervals);
      normalizedPressureAtStartOfInterval = new ArrayList<>(maxIntervals);
      for (int i = 0; i < maxIntervals; i++)
      {
         timeAtStartOfInterval.add(i, new MutableDouble(0.0));
         centerOfPressureAtStartOfInterval.add(i, new FramePoint());
         normalizedPressureContributedByInitialContacts.add(i, new MutableDouble(0.0));
         normalizedPressureContributedByQueuedSteps.add(i, new MutableDouble(0.0));
         normalizedPressureAtStartOfInterval.add(i, new QuadrantDependentList<MutableDouble>());
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            normalizedPressureAtStartOfInterval.get(i).set(robotQuadrant, new MutableDouble(0.0));
         }
      }
   }

   public int getNumberOfIntervals()
   {
      return numberOfIntervals;
   }

   public double getTimeAtStartOfInterval(int interval)
   {
      return timeAtStartOfInterval.get(interval).getValue();
   }

   public FramePoint getCenterOfPressureAtStartOfInterval(int interval)
   {
      return centerOfPressureAtStartOfInterval.get(interval);
   }

   public QuadrantDependentList<MutableDouble> getNormalizedPressureAtStartOfInterval(int interval)
   {
      return normalizedPressureAtStartOfInterval.get(interval);
   }

   public double getNormalizedPressureContributedByInitialContacts(int interval)
   {
      return normalizedPressureContributedByInitialContacts.get(interval).getValue();
   }

   public double getNormalizedPressureContributedByQueuedSteps(int interval)
   {
      return normalizedPressureContributedByQueuedSteps.get(interval).getValue();
   }


   public ArrayList<MutableDouble> getTimeAtStartOfInterval()
   {
      return timeAtStartOfInterval;
   }

   public ArrayList<FramePoint> getCenterOfPressureAtStartOfInterval()
   {
      return centerOfPressureAtStartOfInterval;
   }

   public ArrayList<QuadrantDependentList<MutableDouble>> getNormalizedPressureAtStartOfInterval()
   {
      return normalizedPressureAtStartOfInterval;
   }

   public ArrayList<MutableDouble> getNormalizedPressureContributedByInitialContacts()
   {
      return normalizedPressureContributedByInitialContacts;
   }

   public ArrayList<MutableDouble> getNormalizedPressureContributedByQueuedSteps()
   {
      return normalizedPressureContributedByQueuedSteps;
   }

   /**
    * compute piecewise constant center of pressure plan given the upcoming contact states
    * @param contactStatePlan contact state plan
    */
   public void compute(QuadrupedContactStateSequence contactStatePlan)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         initialContactState.set(robotQuadrant, contactStatePlan.getContactStateAtStartOfInterval(0).get(robotQuadrant));
         isInitialContactState.get(robotQuadrant).setTrue();
      }

      for (int interval = 0; interval < contactStatePlan.getNumberOfIntervals(); interval++)
      {
         numberOfIntervals = contactStatePlan.getNumberOfIntervals();
         QuadrantDependentList<FramePoint> solePosition = contactStatePlan.getSolePositionAtStartOfInterval().get(interval);
         QuadrantDependentList<ContactState> contactState = contactStatePlan.getContactStateAtStartOfInterval().get(interval);

         computeNormalizedContactPressure(normalizedPressureAtStartOfInterval.get(interval), contactState);
         computeCenterOfPressure(centerOfPressureAtStartOfInterval.get(interval), solePosition, normalizedPressureAtStartOfInterval.get(interval));
         normalizedPressureContributedByQueuedSteps.get(interval).setValue(0.0);
         normalizedPressureContributedByInitialContacts.get(interval).setValue(0.0);
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            if (contactState.get(robotQuadrant) != initialContactState.get(robotQuadrant))
            {
               isInitialContactState.get(robotQuadrant).setFalse();
            }
            if (isInitialContactState.get(robotQuadrant).booleanValue())
            {
               normalizedPressureContributedByInitialContacts.get(interval).add(normalizedPressureAtStartOfInterval.get(interval).get(robotQuadrant));
            }
            else
            {
               normalizedPressureContributedByQueuedSteps.add(normalizedPressureAtStartOfInterval.get(interval).get(robotQuadrant));
            }
         }
         timeAtStartOfInterval.get(interval).setValue(contactStatePlan.getTimeAtStartOfInterval().get(interval));
      }
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