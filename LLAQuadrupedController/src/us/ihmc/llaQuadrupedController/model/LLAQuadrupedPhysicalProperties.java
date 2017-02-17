package us.ihmc.llaQuadrupedController.model;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Vector3D;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class LLAQuadrupedPhysicalProperties implements QuadrupedPhysicalProperties
{
   private static final double SHIN_LENGTH = 0.19;
   private final QuadrantDependentList<Vector3D> jointBeforeFootToSoleOffsets = new QuadrantDependentList<>();
   private final QuadrantDependentList<ArrayList<Point2D>> footGroundContactPoints = new QuadrantDependentList<>();
   
   public LLAQuadrupedPhysicalProperties()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         jointBeforeFootToSoleOffsets.set(robotQuadrant, new Vector3D(0.0, 0.0, -SHIN_LENGTH));
         
         ArrayList<Point2D> contactPoints = new ArrayList<>();
         contactPoints.add(new Point2D(0.0, 0.0));
         footGroundContactPoints.set(robotQuadrant, contactPoints);
      }
   }
   
   @Override
   public Vector3D getOffsetFromJointBeforeFootToSole(RobotQuadrant robotQuadrant)
   {
      return jointBeforeFootToSoleOffsets.get(robotQuadrant);
   }

   @Override
   public ArrayList<Point2D> getFootGroundContactPoints(RobotQuadrant robotQuadrant)
   {
      return footGroundContactPoints.get(robotQuadrant);
   }
}
