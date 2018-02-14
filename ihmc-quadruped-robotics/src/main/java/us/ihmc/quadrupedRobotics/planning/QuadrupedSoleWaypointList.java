package us.ihmc.quadrupedRobotics.planning;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedSoleWaypointList extends ArrayList<SoleWaypoint>
{
   private final Point3D solePosition = new Point3D();

   public QuadrupedSoleWaypointList()
   {
   }

   public QuadrupedSoleWaypointList(QuadrupedSoleWaypointList other)
   {
      copy(other);
   }

   public void copy(QuadrupedSoleWaypointList other)
   {
      clear();
      for (int i = 0; i < other.size(); i++)
      {
         add(new SoleWaypoint(other.get(i)));
      }
   }

   public void set(QuadrupedSoleWaypointList other)
   {
      clear();
      for (int i = 0; i < other.size(); i++)
      {
         add(other.get(i));
      }
   }

   public boolean epsilonEquals(QuadrupedSoleWaypointList other, double epsilon)
   {
      if (size() != other.size())
      {
         return false;
      }
      for (int i = 0; i < size(); ++i)
      {
         if (!this.get(i).epsilonEquals(other.get(i), epsilon))
         {
            return false;
         }
      }

      return true;
   }

   public Point3D getSolePosition(int index)
   {
      solePosition.set(this.get(index).getPosition());
      return solePosition;
   }

   public double getFinalTime()
   {
      return (size() == 0.0) ? 0.0 : get(size() - 1).getTime();
   }
}
