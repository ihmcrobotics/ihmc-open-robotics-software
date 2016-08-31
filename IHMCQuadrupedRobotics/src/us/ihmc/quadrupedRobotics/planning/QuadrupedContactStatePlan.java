package us.ihmc.quadrupedRobotics.planning;

import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;

public class QuadrupedContactStatePlan
{
   // contact state plan
   private int numberOfIntervals;
   private final ArrayList<MutableDouble> timeAtStartOfInterval;
   private final ArrayList<QuadrantDependentList<ContactState>> contactStateAtStartOfInterval;
   private final ArrayList<QuadrantDependentList<FramePoint>> solePositionAtStartOfInterval;

   public QuadrupedContactStatePlan(int maximumNumberOfSteps)
   {
      int maximumNumberOfIntervals = 2 * maximumNumberOfSteps + 2;

      numberOfIntervals = 0;
      timeAtStartOfInterval = new ArrayList<>(maximumNumberOfIntervals);
      solePositionAtStartOfInterval = new ArrayList<>(maximumNumberOfIntervals);
      contactStateAtStartOfInterval = new ArrayList<>(maximumNumberOfIntervals);
      for (int i = 0; i < maximumNumberOfIntervals; i++)
      {
         timeAtStartOfInterval.add(i, new MutableDouble(0.0));
         contactStateAtStartOfInterval.add(i, new QuadrantDependentList<ContactState>());
         solePositionAtStartOfInterval.add(i, new QuadrantDependentList<>(new FramePoint(), new FramePoint(), new FramePoint(), new FramePoint()));
      }
   }

   public void setNumberOfIntervals(int numberOfIntervals)
   {
      this.numberOfIntervals = numberOfIntervals;
   }

   public int getNumberOfIntervals()
   {
      return numberOfIntervals;
   }

   public double getTimeAtStartOfInterval(int interval)
   {
      return timeAtStartOfInterval.get(interval).getValue();
   }

   public ArrayList<MutableDouble> getTimeAtStartOfInterval()
   {
      return timeAtStartOfInterval;
   }

   public ContactState getContactStateAtStartOfInterval(int interval, RobotQuadrant robotQuadrant)
   {
      return contactStateAtStartOfInterval.get(interval).get(robotQuadrant);
   }

   public QuadrantDependentList<ContactState> getContactStateAtStartOfInterval(int interval)
   {
      return contactStateAtStartOfInterval.get(interval);
   }

   public ArrayList<QuadrantDependentList<ContactState>> getContactStateAtStartOfInterval()
   {
      return contactStateAtStartOfInterval;
   }

   public FramePoint getSolePositionAtStartOfInterval(int interval, RobotQuadrant robotQuadrant)
   {
      return solePositionAtStartOfInterval.get(interval).get(robotQuadrant);
   }

   public QuadrantDependentList<FramePoint> getSolePositionAtStartOfInterval(int interval)
   {
      return solePositionAtStartOfInterval.get(interval);
   }

   public ArrayList<QuadrantDependentList<FramePoint>> getSolePositionAtStartOfInterval()
   {
      return solePositionAtStartOfInterval;
   }
}