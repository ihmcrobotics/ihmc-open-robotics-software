package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
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

   public int size(RobotQuadrant key)
   {
      return get(key).size();
   }

   public int getMaxSize(){
      int maxSize = 0;
      for (RobotQuadrant quadrant : RobotQuadrant.values())
      {
         if (size(quadrant) > maxSize){
            maxSize =  size(quadrant);
         }
      }
      return  maxSize;
   }

   public int getMinSize(){
      int minSize = Integer.MAX_VALUE;
      for (RobotQuadrant quadrant : RobotQuadrant.values())
      {
         if (size(quadrant) < minSize){
            minSize =  size(quadrant);
         }
      }
      return  minSize;
   }

   public Point3d getFinalPosition(RobotQuadrant key){
      return get(key).get(size(key)-1).getPosition();
   }

   public Vector3d getFinalVelocity(RobotQuadrant key){
      return get(key).get(size(key)-1).getVelocity();
   }

   public double getFinalTime(RobotQuadrant key){
      return get(key).get(size(key)-1).getTime();
   }

   public double getFinalTime(){
      double finalTime = 0;
      for (RobotQuadrant quadrant : RobotQuadrant.values())
      {
         if(getFinalTime(quadrant) > finalTime){
            finalTime = getFinalTime(quadrant);
         }
      }
      return finalTime;
   }
}
