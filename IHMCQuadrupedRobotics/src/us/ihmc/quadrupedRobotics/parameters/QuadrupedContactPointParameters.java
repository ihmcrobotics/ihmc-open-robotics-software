package us.ihmc.quadrupedRobotics.parameters;

import java.util.ArrayList;

import javax.vecmath.Point2d;

import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedContactPointParameters
{
   private final QuadrantDependentList<ArrayList<Point2d>> footGroundContactPoints = new QuadrantDependentList<>();
   
   public QuadrupedContactPointParameters()
   {
      for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ArrayList<Point2d> contactPoints = new ArrayList<Point2d>();
         contactPoints.add(new Point2d(0,0));
         footGroundContactPoints.set(robotQuadrant, contactPoints);
      }
   }
   
   public QuadrantDependentList<ArrayList<Point2d>> getFootGroundContactPoints()
   {
      return footGroundContactPoints;
   }
}
