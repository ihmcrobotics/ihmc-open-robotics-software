package us.ihmc.quadrupedRobotics.planning.trajectory;

import java.util.ArrayList;

import org.apache.commons.lang3.mutable.MutableBoolean;
import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedContactSequence;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedPiecewiseConstantCopTrajectory
{
   private final QuadrantDependentList<ContactState> initialContactState;
   private final QuadrantDependentList<MutableBoolean> isInitialContactState;
   private boolean initialized;

   private final FramePoint copPositionAtCurrentTime;
   private int numberOfIntervals;
   private final ArrayList<MutableDouble> timeAtStartOfInterval;
   private final ArrayList<FramePoint> copPositionAtStartOfInterval;
   private final ArrayList<QuadrantDependentList<MutableDouble>> normalizedPressureAtStartOfInterval;
   private final ArrayList<MutableDouble> normalizedPressureContributedByInitialContacts;
   private final ArrayList<MutableDouble> normalizedPressureContributedByQueuedSteps;

   public QuadrupedPiecewiseConstantCopTrajectory(int maxIntervals)
   {
      initialContactState = new QuadrantDependentList<>();
      isInitialContactState = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         initialContactState.set(robotQuadrant, ContactState.IN_CONTACT);
         isInitialContactState.set(robotQuadrant, new MutableBoolean(true));
      }
      initialized = false;

      copPositionAtCurrentTime = new FramePoint();
      numberOfIntervals = 0;
      timeAtStartOfInterval = new ArrayList<>(maxIntervals);
      copPositionAtStartOfInterval = new ArrayList<>(maxIntervals);
      normalizedPressureContributedByInitialContacts = new ArrayList<>(maxIntervals);
      normalizedPressureContributedByQueuedSteps = new ArrayList<>(maxIntervals);
      normalizedPressureAtStartOfInterval = new ArrayList<>(maxIntervals);
      for (int i = 0; i < maxIntervals; i++)
      {
         timeAtStartOfInterval.add(i, new MutableDouble(0.0));
         copPositionAtStartOfInterval.add(i, new FramePoint());
         normalizedPressureContributedByInitialContacts.add(i, new MutableDouble(0.0));
         normalizedPressureContributedByQueuedSteps.add(i, new MutableDouble(0.0));
         normalizedPressureAtStartOfInterval.add(i, new QuadrantDependentList<MutableDouble>());
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            normalizedPressureAtStartOfInterval.get(i).set(robotQuadrant, new MutableDouble(0.0));
         }
      }
   }

   /**
    * compute piecewise constant center of pressure plan given the upcoming contact states
    * @param timedContactSequence upcoming contact states (input)
    */
   public void initializeTrajectory(QuadrupedTimedContactSequence timedContactSequence)
   {
      if (timedContactSequence.size() < 1)
      {
         throw new RuntimeException("Input contact sequence must have at least one time interval.");
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         initialContactState.set(robotQuadrant, timedContactSequence.get(0).getContactState().get(robotQuadrant));
         isInitialContactState.get(robotQuadrant).setTrue();
      }

      numberOfIntervals = timedContactSequence.size();
      for (int interval = 0; interval < numberOfIntervals; interval++)
      {
         QuadrantDependentList<FramePoint> solePosition = timedContactSequence.get(interval).getSolePosition();
         QuadrantDependentList<ContactState> contactState = timedContactSequence.get(interval).getContactState();

         computeNormalizedContactPressure(normalizedPressureAtStartOfInterval.get(interval), contactState);
         computeCenterOfPressure(copPositionAtStartOfInterval.get(interval), solePosition, normalizedPressureAtStartOfInterval.get(interval));
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
               normalizedPressureContributedByInitialContacts.get(interval)
                     .add(normalizedPressureAtStartOfInterval.get(interval).get(robotQuadrant).doubleValue());
            }
            else
            {
               normalizedPressureContributedByQueuedSteps.get(interval).add(normalizedPressureAtStartOfInterval.get(interval).get(robotQuadrant).doubleValue());
            }
         }
         timeAtStartOfInterval.get(interval).setValue(timedContactSequence.get(interval).getTimeInterval().getStartTime());
      }

      initialized = true;
      computeTrajectory(timeAtStartOfInterval.get(0).doubleValue());
   }

   public void computeTrajectory(double currentTime)
   {
      if (!initialized)
      {
         throw new RuntimeException("trajectory must be initialized before calling computeTrajectory");
      }

      for (int interval = numberOfIntervals - 1; interval >= 0; interval++)
      {
         if (currentTime >= timeAtStartOfInterval.get(interval).doubleValue())
         {
            copPositionAtCurrentTime.setIncludingFrame(copPositionAtStartOfInterval.get(interval));
            return;
         }
      }
      copPositionAtCurrentTime.setIncludingFrame(copPositionAtStartOfInterval.get(0));
   }

   public void getPosition(FramePoint copPosition)
   {
      copPosition.setIncludingFrame(copPositionAtCurrentTime);
   }

   public void getVelocity(FrameVector copVelocity)
   {
      copVelocity.setToZero();
   }

   public int getNumberOfIntervals()
   {
      return numberOfIntervals;
   }

   public double getTimeAtStartOfInterval(int interval)
   {
      return timeAtStartOfInterval.get(interval).doubleValue();
   }

   public FramePoint getCopPositionAtStartOfInterval(int interval)
   {
      return copPositionAtStartOfInterval.get(interval);
   }

   public QuadrantDependentList<MutableDouble> getNormalizedPressureAtStartOfInterval(int interval)
   {
      return normalizedPressureAtStartOfInterval.get(interval);
   }

   public double getNormalizedPressureContributedByInitialContacts(int interval)
   {
      return normalizedPressureContributedByInitialContacts.get(interval).doubleValue();
   }

   public double getNormalizedPressureContributedByQueuedSteps(int interval)
   {
      return normalizedPressureContributedByQueuedSteps.get(interval).doubleValue();
   }

   public ArrayList<MutableDouble> getTimeAtStartOfInterval()
   {
      return timeAtStartOfInterval;
   }

   public ArrayList<FramePoint> getCopPositionAtStartOfInterval()
   {
      return copPositionAtStartOfInterval;
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