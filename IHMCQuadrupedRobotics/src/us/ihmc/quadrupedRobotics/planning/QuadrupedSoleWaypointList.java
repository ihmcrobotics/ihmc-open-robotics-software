package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;

public class QuadrupedSoleWaypointList extends QuadrantDependentList<ArrayList<SoleWaypoint>>
{
   public QuadrupedSoleWaypointList()
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values())
      {
         this.set(quadrant, new ArrayList<SoleWaypoint>());
      }
   }

   public QuadrupedSoleWaypointList(QuadrupedSoleWaypointList other)
   {
      set(other);
   }

   public void set(QuadrupedSoleWaypointList other)
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values())
      {
         ArrayList<SoleWaypoint> soleWaypoints = new ArrayList<>();
         for (int i = 0; i < other.get(quadrant).size(); ++i)
         {
            soleWaypoints.add(new SoleWaypoint(other.get(quadrant).get(i)));
         }
         set(quadrant, soleWaypoints);
      }
   }

   public boolean epsilonEquals(QuadrupedSoleWaypointList other, double epsilon)
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values())
      {
         if (size(quadrant) != other.size(quadrant))
         {
            return false;
         }
         for (int i = 0; i < size(quadrant); ++i)
         {
            if (!this.get(quadrant).get(i).epsilonEquals(other.get(quadrant).get(i), epsilon))
            {
               return false;
            }
         }
      }
      return true;
   }

   public QuadrantDependentList<ArrayList<SoleWaypoint>> get()
   {
      return this;
   }

   public int size(RobotQuadrant key)
   {
      return this.get(key).size();
   }
}
