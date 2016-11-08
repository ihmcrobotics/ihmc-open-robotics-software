package us.ihmc.llaQuadrupedController.model;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class LLAQuadrupedPhysicalProperties implements QuadrupedPhysicalProperties
{
   private static final double SHIN_LENGTH = 0.19;
   private final QuadrantDependentList<Vector3d> jointBeforeFootToSoleOffsets = new QuadrantDependentList<>();
   private final QuadrantDependentList<ArrayList<Point2d>> footGroundContactPoints = new QuadrantDependentList<>();
   
   public LLAQuadrupedPhysicalProperties()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         jointBeforeFootToSoleOffsets.set(robotQuadrant, new Vector3d(0.0, 0.0, -SHIN_LENGTH));
         
         ArrayList<Point2d> contactPoints = new ArrayList<>();
         contactPoints.add(new Point2d(0.0, 0.0));
         footGroundContactPoints.set(robotQuadrant, contactPoints);
      }
   }
   
   @Override
   public Vector3d getOffsetFromJointBeforeFootToSole(RobotQuadrant robotQuadrant)
   {
      return jointBeforeFootToSoleOffsets.get(robotQuadrant);
   }

   @Override
   public ArrayList<Point2d> getFootGroundContactPoints(RobotQuadrant robotQuadrant)
   {
      return footGroundContactPoints.get(robotQuadrant);
   }
}
