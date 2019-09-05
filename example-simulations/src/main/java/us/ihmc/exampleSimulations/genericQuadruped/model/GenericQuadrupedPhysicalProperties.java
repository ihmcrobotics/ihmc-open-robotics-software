package us.ihmc.exampleSimulations.genericQuadruped.model;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;

public class GenericQuadrupedPhysicalProperties implements QuadrupedPhysicalProperties
{
   private static final double SHIN_LENGTH = 0.31;
   private static final double NOMINAL_BODY_HEIGHT = 0.6;

   private final QuadrantDependentList<Vector3D> jointBeforeFootToSoleOffsets = new QuadrantDependentList<>();
   private final QuadrantDependentList<ArrayList<Point2D>> footGroundContactPoints = new QuadrantDependentList<>();
   
   public GenericQuadrupedPhysicalProperties()
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

   @Override
   public QuadrantDependentList<ArrayList<Point2D>> getFeetGroundContactPoints()
   {
      return footGroundContactPoints;
   }

   @Override
   public double getNominalBodyHeight()
   {
      return NOMINAL_BODY_HEIGHT;
   }

   @Override
   public boolean trustFootSwitchesInSwing()
   {
      return true;
   }

   @Override
   public boolean trustFootSwitchesInSupport()
   {
      return false;
   }
}
