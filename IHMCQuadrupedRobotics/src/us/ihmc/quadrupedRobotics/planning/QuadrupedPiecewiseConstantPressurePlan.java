package us.ihmc.quadrupedRobotics.planning;

import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;

public class QuadrupedPiecewiseConstantPressurePlan
{
   // pressure plan
   private int numberOfIntervals;
   private final ArrayList<MutableDouble> timeAtStartOfInterval;
   private final ArrayList<FramePoint> centerOfPressureAtStartOfInterval;
   private final ArrayList<QuadrantDependentList<MutableDouble>> normalizedPressureAtStartOfInterval;
   private final ArrayList<MutableDouble> normalizedPressureContributedByInitialContacts;
   private final ArrayList<MutableDouble> normalizedPressureContributedByQueuedSteps;

   public QuadrupedPiecewiseConstantPressurePlan(int maxSteps)
   {
      int maxIntervals = 2 * maxSteps + 2;

      // pressure plan
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

   public void setNumberOfIntervals(int numberOfIntervals)
   {
      this.numberOfIntervals = numberOfIntervals;
   }

   public double getTimeAtStartOfInterval(int interval)
   {
      return timeAtStartOfInterval.get(interval).getValue();
   }

   public ArrayList<MutableDouble> getTimeAtStartOfInterval()
   {
      return timeAtStartOfInterval;
   }

   public FramePoint getCenterOfPressureAtStartOfInterval(int interval)
   {
      return centerOfPressureAtStartOfInterval.get(interval);
   }

   public ArrayList<FramePoint> getCenterOfPressureAtStartOfInterval()
   {
      return centerOfPressureAtStartOfInterval;
   }

   public double getNormalizedPressureAtStartOfInterval(int interval, RobotQuadrant robotQuadrant)
   {
      return normalizedPressureAtStartOfInterval.get(interval).get(robotQuadrant).getValue();
   }

   public QuadrantDependentList<MutableDouble> getNormalizedPressureAtStartOfInterval(int interval)
   {
      return normalizedPressureAtStartOfInterval.get(interval);
   }

   public ArrayList<QuadrantDependentList<MutableDouble>> getNormalizedPressureAtStartOfInterval()
   {
      return normalizedPressureAtStartOfInterval;
   }

   public double getNormalizedPressureContributedByInitialContacts(int interval)
   {
      return normalizedPressureContributedByInitialContacts.get(interval).getValue();
   }

   public ArrayList<MutableDouble> getNormalizedPressureContributedByInitialContacts()
   {
      return normalizedPressureContributedByInitialContacts;
   }

   public double getNormalizedPressureContributedByQueuedSteps(int interval)
   {
      return normalizedPressureContributedByQueuedSteps.get(interval).getValue();
   }

   public ArrayList<MutableDouble> getNormalizedPressureContributedByQueuedSteps()
   {
      return normalizedPressureContributedByQueuedSteps;
   }
}